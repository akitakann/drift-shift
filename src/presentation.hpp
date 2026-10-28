#pragma once

#include "car.hpp"
#include "tuning.hpp"

#include <algorithm>
#include <cmath>

enum class CameraMotionPhase { Grip, Entry, Held, Exit, Recovery };

// Presentation hypotheses, independent of the vehicle's forces and controls.
struct CameraPresentation {
    Vec3 eye, look, anchor;
    float heading = 0;
    bool ready = false;
    int mode = 0;
    float fovDegrees = 60;
    float lookHeading = 0;
    float boostPreview = 0; // Predict the manual exit direction while sliding; not a boost event.
    float boostAmount = 0; // Normalized, finite visual cue; never changes car speed.
    float chargeAmount = 0;
    float boostViewAmount = 0; // Independent, tier-aware camera dolly amount, 0..1.
    float boostFovAmount = 0; // A longer lens tail, separate from jet and camera position.
    float chargeFovOffset = 0, boostFovOffset = 0;
    float boostSetback = 0;
    float steerLookOffset = 0; // Actual optical yaw preview in radians; never rotates the orbit.
    Vec3 chargeShake;
    bool boostTriggered = false;
    unsigned boostEventSerial = 0;
    // Small, authored charge/release contrast requested for this prototype.
    static constexpr float chargeFovPeak = 1;
    static constexpr float boostFovPeak = 5;
    static constexpr float boostSetbackPeak = 0.25f;
    static constexpr float boostSetbackAttackTime = 0.10f;
    static constexpr float boostSetbackHoldTime = 0.20f;
    static constexpr float boostSetbackReturnTime = 0.80f;
    static constexpr float chargeShakeHorizontal = 0.006f;
    static constexpr float chargeShakeVertical = 0.004f;
    static constexpr float chargeResponseTime = 0.18f;
    static constexpr float chargeReleaseTime = 0.10f;
    static constexpr float boostAttackTime = 0.05f;
    static constexpr float boostReturnTime = 0.30f;
    static constexpr float boostFovAttackTime = 0.10f;
    static constexpr float boostFovHoldTime = 0.20f;
    static constexpr float boostFovReturnTime = 1.80f;
    CameraMotionPhase motionPhase = CameraMotionPhase::Grip;
    float motionElapsed = 0;
    // Authored timing hypotheses, not camera parameters recovered from the footage.
    static constexpr float entryDuration = 0.42f;
    static constexpr float entryDelay = 0.06f;
    static constexpr float exitDuration = 0.55f;
    static constexpr float recoveryDuration = 0.33f;
    static constexpr float boostLookWeight = 0.35f;
    static constexpr float boostPreviewResponseTime = 0.18f;

    void update(float dt, const Car& car, const FeelSettings& rawSettings, bool active = true) {
        const FeelSettings settings = sanitizeFeelSettings(rawSettings);
        boostTriggered = false;
        const float targetHeading = cameraFacingRelease(car) ? releaseCarrier(car) :
            wrapPi(car.yaw - car.slipAngle * settings.cameraTravelWeight);
        const Vec3 position{car.x, car.y, car.z};
        if (!ready) {
            // Priming an existing pose/serial must not replay an earlier release.
            heading = targetHeading;
            anchor = position;
            observedSerial_ = car.boostSerial;
            boostEventSerial = car.boostSerial;
            clearBoostCue();
            wasRealBoost_ = false;
            lastBoostRemaining_ = 0;
            wasSliding_ = sliding(car);
            boostPreview = wasSliding_ ? 1.0f : 0.0f;
            wasReleasing_ = car.releaseEaseActive;
            lastDriftAge_ = car.driftAge;
            motionPhase = wasSliding_ ? CameraMotionPhase::Held : CameraMotionPhase::Grip;
            motionElapsed = 0;
            motionCarrier_ = car.travelYaw;
            motionFrom_ = wrapPi(heading - motionCarrier_);
            motionEaseFrom_ = 0;
            exitRemainingFrom_ = 1;
            cameraReleaseExit_ = preserveMotionLook_ = false;
            motionLookFrom_ = 0;
            steerLookOffset = 0;
            ready = true;
            compose(car, settings);
            return;
        }
        // Pauses freeze the whole view and leave a pending event for resume.
        if (!active || !(dt > 0) || !std::isfinite(dt)) return;

        const bool realBoost = boostIntent(car);
        if (car.boostSerial != observedSerial_) {
            observedSerial_ = car.boostSerial;
            if (realBoost) {
                cueAttackFrom_ = boostAmount;
                cueLevel_ = car.boostLevel;
                cueElapsed_ = 0;
                cancelElapsed_ = -1;
                boostTriggered = true;
                boostEventSerial = car.boostSerial;
                fovAttackFrom_ = boostFovAmount;
                fovLevel_ = car.boostLevel;
                fovElapsed_ = 0;
                fovCancelElapsed_ = -1;
                setbackAttackFrom_ = boostViewAmount;
                // A fresh weaker reward must not pull the camera forward while
                // its preceding dolly is still farther out. Replace, never add.
                setbackTarget_ = std::fmax(boostViewAmount, (2.0f + car.boostLevel) / 5.0f);
                setbackElapsed_ = 0;
                setbackCancelElapsed_ = -1;
            }
        }

        // The physical burst is deliberately short. Its normal expiry must not
        // truncate the longer lens return, but an interrupted reward still fades.
        const bool interrupted = car.driftHeld || car.driftHoldBlocked ||
                                 !car.onAsphalt || car.impact > 0.05f || car.speed <= 0;
        const bool lostEarly = wasRealBoost_ && !realBoost &&
                               lastBoostRemaining_ > dt + 1.0f / 60.0f + 0.0001f;
        if (fovElapsed_ >= 0 && fovCancelElapsed_ < 0 && (interrupted || lostEarly)) {
            fovCancelFrom_ = boostFovAmount;
            fovCancelElapsed_ = 0;
        }
        if (fovElapsed_ >= 0) {
            if (fovCancelElapsed_ >= 0) {
                fovCancelElapsed_ += dt;
                boostFovAmount = fovCancelFrom_ * (1 - smootherStep(fovCancelElapsed_ / settings.boostKickReturn));
                if (fovCancelElapsed_ + 0.000001f >= settings.boostKickReturn) clearFovEnvelope();
            } else {
                fovElapsed_ += dt;
                const float fovTarget = (2.0f + fovLevel_) / 5.0f;
                boostFovAmount = fovElapsed_ < settings.boostFovAttack ?
                    fovAttackFrom_ + (fovTarget - fovAttackFrom_) * smoothStep(fovElapsed_ / settings.boostFovAttack) :
                    fovTarget * (1 - smootherStep((fovElapsed_ - settings.boostFovAttack - settings.boostFovHold) / settings.boostFovReturn));
                if (fovElapsed_ + 0.0001f >= settings.boostFovAttack + settings.boostFovHold + settings.boostFovReturn) clearFovEnvelope();
            }
        }
        boostFovAmount = clampf(boostFovAmount, 0, 1);
        // Camera travel has its own authored attack and long return. Neither
        // the short jet nor natural physical expiry may truncate this motion.
        if (setbackElapsed_ >= 0 && setbackCancelElapsed_ < 0 && (interrupted || lostEarly)) {
            setbackCancelFrom_ = boostViewAmount;
            setbackCancelElapsed_ = 0;
        }
        if (setbackElapsed_ >= 0) {
            if (setbackCancelElapsed_ >= 0) {
                setbackCancelElapsed_ += dt;
                boostViewAmount = setbackCancelFrom_ * (1 - smootherStep(setbackCancelElapsed_ / settings.boostSetbackReturn));
                if (setbackCancelElapsed_ + 0.0001f >= settings.boostSetbackReturn) clearSetbackEnvelope();
            } else {
                setbackElapsed_ += dt;
                const float target = setbackTarget_;
                boostViewAmount = setbackElapsed_ < settings.boostSetbackAttack ?
                    setbackAttackFrom_ + (target - setbackAttackFrom_) * smootherStep(setbackElapsed_ / settings.boostSetbackAttack) :
                    target * (1 - smootherStep((setbackElapsed_ - settings.boostSetbackAttack - settings.boostSetbackHold) / settings.boostSetbackReturn));
                if (setbackElapsed_ + 0.0001f >= settings.boostSetbackAttack + settings.boostSetbackHold + settings.boostSetbackReturn) clearSetbackEnvelope();
            }
        }
        boostViewAmount = clampf(boostViewAmount, 0, 1);
        wasRealBoost_ = realBoost;
        lastBoostRemaining_ = realBoost ? car.boostRemaining : 0;
        if (!realBoost && cueElapsed_ >= 0 && cancelElapsed_ < 0) {
            cancelFrom_ = boostAmount;
            cancelElapsed_ = 0;
        }
        if (cueElapsed_ >= 0) {
            if (cancelElapsed_ >= 0) {
                cancelElapsed_ += dt;
                boostAmount = cancelFrom_ * (1.0f - smoothStep(cancelElapsed_ / settings.boostKickReturn));
                if (cancelElapsed_ >= settings.boostKickReturn) clearBoostEnvelope();
            } else {
                cueElapsed_ += dt;
                boostAmount = cueElapsed_ < boostAttackTime ?
                    cueAttackFrom_ + (1 - cueAttackFrom_) * smoothStep(cueElapsed_ / boostAttackTime) :
                    1.0f - easeOutCubic((cueElapsed_ - boostAttackTime) / settings.boostKickReturn);
                if (cueElapsed_ + 0.000001f >= boostAttackTime + settings.boostKickReturn) clearBoostEnvelope();
            }
        }

        const bool moving = std::isfinite(car.speed) && car.speed > 5;
        const bool charging = moving && sliding(car) && car.onAsphalt && !realBoost &&
                              std::isfinite(car.driftCharge) && car.driftCharge > 0;
        const float chargeTarget = charging ? clampf(car.driftCharge / 2.20f, 0, 1) : 0;
        if (!moving) chargeAmount = 0;
        else chargeAmount += (chargeTarget - chargeAmount) *
                             response(dt, charging ? chargeResponseTime : chargeReleaseTime);
        if (chargeAmount < 0.0001f) chargeAmount = 0;
        // A bounded active-time clock keeps the tiny translation reproducible
        // across render rates; zero amplitude makes it invisible when uncharged.
        effectTime_ = std::fmod(effectTime_ + dt, 1.0f);

        updateHeading(dt, car, settings, targetHeading);
        const float steerTarget = steeringPreviewTarget(car, settings);
        const bool buildingPreview = steerTarget * steerLookOffset >= 0 &&
                                     std::fabs(steerTarget) > std::fabs(steerLookOffset);
        const float steerResponse = buildingPreview ? .18f : std::fmax(.18f, settings.cameraResponseTime);
        steerLookOffset += (steerTarget - steerLookOffset) * response(dt, steerResponse);
        const float steerLimit = settings.cameraSteerLookMax * 3.14159265359f / 180;
        steerLookOffset = clampf(steerLookOffset, -steerLimit, steerLimit);
        if (settings.cameraSteerLookAhead == 0 || settings.cameraSteerLookMax == 0) steerLookOffset = 0;
        const float previewTarget = sliding(car) || motionPhase == CameraMotionPhase::Exit ? 1.0f : 0.0f;
        boostPreview += (previewTarget - boostPreview) * response(dt, boostPreviewResponseTime);
        const float previousAnchorY=anchor.y;
        anchor = anchor + (position - anchor) * response(dt, 0.20f);
        if(!car.grounded) anchor.y=previousAnchorY+(position.y-previousAnchorY)*response(dt,0.12f);
        const Vec3 forward{std::sin(heading), 0, std::cos(heading)};
        const Vec3 right{std::cos(heading), 0, -std::sin(heading)};
        // Longitudinal damping would add metres of chase distance at speed and shrink the car.
        // Follow this axis directly; retain only a small, damped sideways composition change.
        anchor = anchor + forward * dot(position - anchor, forward);
        const float lateralLag = dot(position - anchor, right);
        anchor = anchor + right * (lateralLag - clampf(lateralLag, -0.75f, 0.75f));
        compose(car, settings);
    }

    // Explicit live tuning changes the current composition without simulating a
    // frame. Ordinary inactive updates still freeze the view. The caller applies
    // any physical tuning first and passes the same, unmoved car pose here.
    void applySettings(const Car& car, const FeelSettings& previousSettings, const FeelSettings& newSettings) {
        if (!ready) return;
        const FeelSettings before = sanitizeFeelSettings(previousSettings);
        const FeelSettings after = sanitizeFeelSettings(newSettings);
        boostTriggered = false;

        if (motionPhase == CameraMotionPhase::Entry &&
            (before.cameraEntryTime != after.cameraEntryTime || before.cameraTravelWeight != after.cameraTravelWeight)) {
            const float progress = clampf((motionElapsed - entryDelay) / (before.cameraEntryTime - entryDelay), 0, 1);
            if (motionElapsed > entryDelay)
                motionElapsed = entryDelay + progress * (after.cameraEntryTime - entryDelay);
            motionCarrier_ = car.travelYaw;
            motionFrom_ = wrapPi(heading - motionCarrier_);
            if (preserveMotionLook_) motionLookFrom_ = wrapPi(lookHeading - motionCarrier_);
            motionEaseFrom_ = smootherStep(progress);
        } else if (motionPhase == CameraMotionPhase::Exit &&
                   (before.cameraExitTime != after.cameraExitTime || before.cameraExitPower != after.cameraExitPower)) {
            const float progress = clampf(motionElapsed / before.cameraExitTime, 0, 1);
            motionElapsed = progress * after.cameraExitTime;
            motionCarrier_ = releaseCarrier(car);
            motionFrom_ = wrapPi(heading - motionCarrier_);
            if (preserveMotionLook_) motionLookFrom_ = wrapPi(lookHeading - motionCarrier_);
            exitRemainingFrom_ = 1 - progress;
        } else if (motionPhase == CameraMotionPhase::Recovery && before.cameraTravelWeight != after.cameraTravelWeight) {
            motionCarrier_ = car.yaw;
            motionFrom_ = wrapPi(heading - motionCarrier_);
            if (preserveMotionLook_) motionLookFrom_ = wrapPi(lookHeading - motionCarrier_);
            motionEaseFrom_ = smootherStep(motionElapsed / recoveryDuration);
        }

        const bool kickTimeChanged = before.boostKickReturn != after.boostKickReturn;
        if (kickTimeChanged && cueElapsed_ >= boostAttackTime)
            cueElapsed_ = boostAttackTime + scaledClock(cueElapsed_ - boostAttackTime, before.boostKickReturn, after.boostKickReturn);
        if (kickTimeChanged && cancelElapsed_ >= 0)
            cancelElapsed_ = scaledClock(cancelElapsed_, before.boostKickReturn, after.boostKickReturn);
        if (fovElapsed_ >= 0 && (before.boostFovAttack != after.boostFovAttack || before.boostFovHold != after.boostFovHold || before.boostFovReturn != after.boostFovReturn)) {
            fovElapsed_ = retimeEnvelope(fovElapsed_, before.boostFovAttack, before.boostFovHold, before.boostFovReturn,
                                        after.boostFovAttack, after.boostFovHold, after.boostFovReturn);
        }
        if (kickTimeChanged && fovCancelElapsed_ >= 0)
            fovCancelElapsed_ = scaledClock(fovCancelElapsed_, before.boostKickReturn, after.boostKickReturn);
        if (setbackElapsed_ >= 0 &&
            (before.boostSetbackAttack != after.boostSetbackAttack || before.boostSetbackHold != after.boostSetbackHold || before.boostSetbackReturn != after.boostSetbackReturn)) {
            setbackElapsed_ = retimeEnvelope(setbackElapsed_, before.boostSetbackAttack, before.boostSetbackHold, before.boostSetbackReturn,
                                            after.boostSetbackAttack, after.boostSetbackHold, after.boostSetbackReturn);
        }
        if (setbackCancelElapsed_ >= 0 && before.boostSetbackReturn != after.boostSetbackReturn)
            setbackCancelElapsed_ = scaledClock(setbackCancelElapsed_, before.boostSetbackReturn, after.boostSetbackReturn);

        // A live physical duration edit is not an early cancelled reward. Refresh
        // its expiry baseline without observing a pending serial or starting cues.
        wasRealBoost_ = boostIntent(car);
        lastBoostRemaining_ = wasRealBoost_ ? car.boostRemaining : 0;
        if (before.cameraSteerLookAhead != after.cameraSteerLookAhead || before.cameraSteerLookMax != after.cameraSteerLookMax) {
            // A deliberate paused preview edit is immediately reviewable. Other
            // settings leave its existing target/filtered pose untouched.
            steerLookOffset = steeringPreviewTarget(car, after);
        }
        compose(car, after);
    }

    // End a run explicitly. Restore the cached undecorated pose immediately because
    // the next inactive update freezes it; pause itself retains every current cue.
    void clearBoostCue() {
        clearBoostEnvelope();
        clearFovEnvelope();
        clearSetbackEnvelope();
        chargeAmount = chargeFovOffset = 0;
        chargeShake = {};
        effectTime_ = 0;
        eye = baseEye_;
        look = baseLook_;
        lookHeading = std::atan2(look.x - eye.x, look.z - eye.z);
        fovDegrees = baseFov_;
    }

private:
    float cueElapsed_ = -1, cancelElapsed_ = -1, cancelFrom_ = 0;
    float cueAttackFrom_ = 0, effectTime_ = 0;
    int cueLevel_ = 0;
    float fovElapsed_ = -1, fovCancelElapsed_ = -1, fovCancelFrom_ = 0, fovAttackFrom_ = 0;
    int fovLevel_ = 0;
    float setbackElapsed_ = -1, setbackCancelElapsed_ = -1, setbackCancelFrom_ = 0, setbackAttackFrom_ = 0;
    float setbackTarget_ = 0;
    bool wasRealBoost_ = false;
    float lastBoostRemaining_ = 0;
    Vec3 baseEye_, baseLook_;
    void clearBoostEnvelope() {
        boostAmount = 0;
        boostTriggered = false;
        cueElapsed_ = cancelElapsed_ = -1;
        cancelFrom_ = cueAttackFrom_ = 0;
        cueLevel_ = 0;
    }
    void clearFovEnvelope() {
        boostFovAmount = boostFovOffset = 0;
        fovElapsed_ = fovCancelElapsed_ = -1;
        fovCancelFrom_ = fovAttackFrom_ = 0;
        fovLevel_ = 0;
    }
    void clearSetbackEnvelope() {
        boostViewAmount = boostSetback = 0;
        setbackElapsed_ = setbackCancelElapsed_ = -1;
        setbackCancelFrom_ = setbackAttackFrom_ = 0;
        setbackTarget_ = 0;
    }

    float baseFov_ = 60;
    unsigned observedSerial_ = 0;
    bool wasSliding_ = false, wasReleasing_ = false;
    float lastDriftAge_ = 0, motionFrom_ = 0, motionCarrier_ = 0;
    float motionEaseFrom_ = 0, exitRemainingFrom_ = 1;
    bool cameraReleaseExit_ = false, preserveMotionLook_ = false;
    float motionLookFrom_ = 0;

    static float response(float dt, float time90) {
        return 1.0f - std::exp(-2.302585093f * dt / std::fmax(time90, 0.001f));
    }
    static float smoothStep(float t) {
        t = clampf(t, 0, 1);
        return t * t * (3.0f - 2.0f * t);
    }
    static float smootherStep(float t) {
        t = clampf(t, 0, 1);
        return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
    }
    static float scaledClock(float elapsed, float before, float after) {
        return clampf(elapsed / before, 0, 1) * after;
    }
    static float retimeEnvelope(float elapsed, float oldAttack, float oldHold, float oldReturn,
                               float newAttack, float newHold, float newReturn) {
        if (elapsed < oldAttack) return scaledClock(elapsed, oldAttack, newAttack);
        if (oldHold > 0 && elapsed < oldAttack + oldHold)
            return newAttack + scaledClock(elapsed - oldAttack, oldHold, newHold);
        return newAttack + newHold + scaledClock(elapsed - oldAttack - oldHold, oldReturn, newReturn);
    }
    static bool boostIntent(const Car& car) {
        return car.boostRemaining > 0 && car.boostLevel > 0 && car.boostLevel <= 3 &&
               car.speed > 0 && car.boostSpeedScale > 0;
    }
    static Vec3 previewLook(const Vec3& eyePosition, Vec3 lookPosition, float offset) {
        const float dx = lookPosition.x - eyePosition.x, dz = lookPosition.z - eyePosition.z;
        const float turnCos = std::cos(offset), turnSin = std::sin(offset);
        lookPosition.x = eyePosition.x + dx * turnCos + dz * turnSin;
        lookPosition.z = eyePosition.z + dz * turnCos - dx * turnSin;
        return lookPosition;
    }
    float steeringPreviewTarget(const Car& car, const FeelSettings& settings) const {
        if (motionPhase != CameraMotionPhase::Grip || car.drifting || car.releaseEaseActive ||
            car.speed <= 5 || !std::isfinite(car.speed) || !std::isfinite(car.yawRate)) return 0;
        constexpr float radiansPerDegree = 3.14159265359f / 180;
        const float limit = settings.cameraSteerLookMax * radiansPerDegree;
        return clampf(car.yawRate * settings.cameraSteerLookAhead, -limit, limit);
    }
    float remainingMotionEase(float eased) const {
        const float remaining = 1 - motionEaseFrom_;
        return remaining > 0 ? clampf((eased - motionEaseFrom_) / remaining, 0, 1) : 1;
    }
    static float easeOutCubic(float t) {
        t = 1 - clampf(t, 0, 1);
        return 1 - t * t * t;
    }
    static float angleMix(float from, float to, float amount) {
        return wrapPi(from + wrapPi(to - from) * amount);
    }
    static bool sliding(const Car& car) {
        return car.driftHoldActive && car.driftPhase != DriftPhase::None;
    }
    static bool cameraFacingRelease(const Car& car) {
        return car.releaseEaseActive && car.releaseUsesCameraHeading && std::isfinite(car.releaseHeading);
    }
    static float releaseCarrier(const Car& car) {
        return cameraFacingRelease(car) ? car.releaseHeading : car.yaw;
    }
    void beginMotion(CameraMotionPhase phase, float carrier, bool cameraRelease = false) {
        // A camera-directed launch begins at the already displayed optical ray.
        // Keep that continuity if it is interrupted by recovery or another entry.
        preserveMotionLook_ = cameraRelease || preserveMotionLook_;
        cameraReleaseExit_ = cameraRelease;
        if (preserveMotionLook_) motionLookFrom_ = wrapPi(lookHeading - carrier);
        motionPhase = phase;
        motionElapsed = 0;
        // A phase switch changes coordinate frames, not the displayed pose.
        // Rebase from the actual heading using this frame's new carrier; adding
        // a delta between the old and new carriers would apply the turn twice.
        motionCarrier_ = carrier;
        motionFrom_ = wrapPi(heading - carrier);
        motionEaseFrom_ = 0;
        exitRemainingFrom_ = 1;
    }
    void updateHeading(float dt, const Car& car, const FeelSettings& settings, float target) {
        const bool held = sliding(car);
        const bool restarted = held && wasSliding_ && car.driftAge + 0.0001f < lastDriftAge_;
        if (held && (!wasSliding_ || restarted)) {
            // Interrupt an exit at its actual pose; never restart from an old entry pose.
            beginMotion(CameraMotionPhase::Entry, car.travelYaw);
        } else if (!held && car.releaseEaseActive && !wasReleasing_) {
            // An uncharged manual release has exactly the same camera return as a boost.
            beginMotion(CameraMotionPhase::Exit, releaseCarrier(car), cameraFacingRelease(car));
        } else if ((!held && wasSliding_ && !car.releaseEaseActive) ||
                   (motionPhase == CameraMotionPhase::Exit && cameraReleaseExit_ && !car.releaseUsesCameraHeading) ||
                   (motionPhase == CameraMotionPhase::Exit && wasReleasing_ &&
                    !car.releaseEaseActive && car.drifting)) {
            // Brake/wall cancellation uses recovery rather than masquerading as release.
            beginMotion(CameraMotionPhase::Recovery, car.yaw);
        }
        motionElapsed = std::fmin(motionElapsed + dt, 1000.0f);
        if (motionPhase == CameraMotionPhase::Entry) {
            // The delay and easing affect only the orbit around the live travel
            // direction. The line itself keeps turning even during the delay.
            // Interpolate local scalars: a frozen world origin and live endpoint
            // would change the shortest-arc branch after half a world rotation.
            const float t = (motionElapsed - entryDelay) / (settings.cameraEntryTime - entryDelay);
            const float localTarget = (1 - settings.cameraTravelWeight) * car.slipAngle;
            heading = wrapPi(car.travelYaw + motionFrom_ + (localTarget - motionFrom_) * remainingMotionEase(smootherStep(t)));
            motionCarrier_ = car.travelYaw;
            if (motionElapsed + 0.000001f >= settings.cameraEntryTime) {
                heading = wrapPi(car.travelYaw + localTarget);
                motionPhase = held ? CameraMotionPhase::Held : CameraMotionPhase::Grip;
                motionElapsed = 0;
                preserveMotionLook_ = false;
            }
        } else if (motionPhase == CameraMotionPhase::Exit) {
            // Carry the view with current steering while its captured local
            // residual decays. Repeated manual releases cannot freeze an old
            // world-space starting point or accumulate a chase-heading backlog.
            const float remaining = 1 - clampf(motionElapsed / settings.cameraExitTime, 0, 1);
            const float relativeRemaining = exitRemainingFrom_ > 0 ? clampf(remaining / exitRemainingFrom_, 0, 1) : 0;
            const float carrier = releaseCarrier(car);
            heading = wrapPi(carrier + motionFrom_ * std::pow(relativeRemaining, settings.cameraExitPower));
            motionCarrier_ = carrier;
            if (motionElapsed + 0.00001f >= settings.cameraExitTime) {
                heading = wrapPi(carrier);
                motionPhase = CameraMotionPhase::Grip;
                motionElapsed = 0;
                cameraReleaseExit_ = preserveMotionLook_ = false;
            }
        } else if (motionPhase == CameraMotionPhase::Recovery) {
            const float localTarget = -settings.cameraTravelWeight * car.slipAngle;
            heading = wrapPi(car.yaw + motionFrom_ + (localTarget - motionFrom_) * remainingMotionEase(smootherStep(motionElapsed / recoveryDuration)));
            motionCarrier_ = car.yaw;
            if (motionElapsed + 0.000001f >= recoveryDuration) {
                heading = target;
                motionPhase = CameraMotionPhase::Grip;
                motionElapsed = 0;
                preserveMotionLook_ = false;
            }
        } else if (held) {
            // Smooth changes in drift angle, not accumulated world rotation.
            // Recover the actual local pose so an interrupted or externally
            // sampled view remains continuous; the scalar delta is never wrapped.
            const float local = wrapPi(heading - motionCarrier_);
            const float localTarget = (1 - settings.cameraTravelWeight) * car.slipAngle;
            heading = wrapPi(car.travelYaw + local + (localTarget - local) * response(dt, settings.cameraResponseTime));
            motionCarrier_ = car.travelYaw;
            motionPhase = CameraMotionPhase::Held;
        } else {
            // Preserve ordinary driving's existing yaw response.
            heading = angleMix(heading, target, response(dt, settings.cameraResponseTime));
            motionPhase = CameraMotionPhase::Grip;
        }
        wasSliding_ = held;
        wasReleasing_ = car.releaseEaseActive;
        lastDriftAge_ = car.driftAge;
    }
    Vec3 motionLook(const Car& car, const FeelSettings& settings, const Vec3& eyePosition, Vec3 lookPosition) const {
        if (!preserveMotionLook_) {
            // The orbit may settle before the body's independently timed turn.
            // Continue following the captured launch line for that last interval.
            if (!cameraFacingRelease(car)) return lookPosition;
            const float optical = std::atan2(lookPosition.x - eyePosition.x, lookPosition.z - eyePosition.z);
            return previewLook(eyePosition, lookPosition, wrapPi(releaseCarrier(car) - optical));
        }
        const float carrier = motionPhase == CameraMotionPhase::Entry ? car.travelYaw :
                              motionPhase == CameraMotionPhase::Exit ? releaseCarrier(car) : car.yaw;
        const float optical = std::atan2(lookPosition.x - eyePosition.x, lookPosition.z - eyePosition.z);
        const float target = cameraReleaseExit_ ? 0 : wrapPi(optical - carrier);
        float progress = 1;
        if (motionPhase == CameraMotionPhase::Entry)
            progress = remainingMotionEase(smootherStep((motionElapsed - entryDelay) / (settings.cameraEntryTime - entryDelay)));
        else if (motionPhase == CameraMotionPhase::Exit) {
            const float remaining = 1 - clampf(motionElapsed / settings.cameraExitTime, 0, 1);
            const float relativeRemaining = exitRemainingFrom_ > 0 ? clampf(remaining / exitRemainingFrom_, 0, 1) : 0;
            progress = 1 - std::pow(relativeRemaining, settings.cameraExitPower);
        } else if (motionPhase == CameraMotionPhase::Recovery)
            progress = remainingMotionEase(smootherStep(motionElapsed / recoveryDuration));
        const float desired = carrier + angleMix(motionLookFrom_, target, progress);
        return previewLook(eyePosition, lookPosition, wrapPi(desired - optical));
    }
    void compose(const Car& car, const FeelSettings& settings) {
        // Mode 0 matches the observed screen composition with this different car model.
        // These are equivalent framing values, not recovered World Tour camera intrinsics.
        const float back[] = {5.0f, 4.1f, 16.0f};
        const float up[] = {2.05f, 1.55f, 7.2f};
        const int cameraMode = std::max(0, std::min(2, mode));
        const Vec3 forward{std::sin(heading), 0, std::cos(heading)};
        eye = anchor - forward * (back[cameraMode] * settings.cameraDistance / 5.0f) +
              Vec3{0, up[cameraMode] * settings.cameraHeight / 2.05f, 0};
        // Keep the chase position oriented mainly along the sliding line, but reveal
        // a little of the road the nose will launch toward when Space is released.
        // Aim only partway there; a full nose view would conceal the sideways slide.
        const float previewHeading = releaseCarrier(car);
        const float noseDelta = clampf(wrapPi(previewHeading - heading), -1.570796327f, 1.570796327f);
        const float aimHeading = heading + noseDelta * settings.cameraPreviewWeight * boostPreview;
        const Vec3 lookForward{std::sin(aimHeading), 0, std::cos(aimHeading)};
        look = anchor + lookForward * settings.cameraLookAhead + Vec3{0, 0.17f, 0};
        baseEye_ = eye;
        // Clearing reward layers must retain the normal steering preview.
        baseLook_ = motionLook(car, settings, baseEye_, previewLook(baseEye_, look, steerLookOffset));
        chargeFovOffset = settings.chargeFovPeak * clampf(chargeAmount, 0, 1);
        boostFovOffset = settings.boostFovPeak * clampf(boostFovAmount, 0, 1);
        boostSetback = settings.boostSetbackPeak * boostViewAmount;
        const Vec3 right{std::cos(heading), 0, -std::sin(heading)};
        constexpr float twoPi = 6.283185307f;
        chargeShake = right * (chargeShakeHorizontal * settings.chargeShakeScale * chargeAmount * std::sin(twoPi * 7 * effectTime_)) +
                      Vec3{0, chargeShakeVertical * settings.chargeShakeScale * chargeAmount * std::sin(twoPi * 9 * effectTime_), 0};
        eye = eye - forward * boostSetback + chargeShake;
        look = look + chargeShake;
        // Rotate the complete optical ray so the configured maximum is the
        // actual visible preview, independent of chase distance and lookahead.
        look = previewLook(eye, look, steerLookOffset);
        look = motionLook(car, settings, eye, look);
        lookHeading = std::atan2(look.x - eye.x, look.z - eye.z);
        baseFov_ = settings.baseFov + clampf(settings.fovGain, 0, 6) *
                         clampf((car.speed * 3.6f - 40) / 80, 0, 1);
        fovDegrees = baseFov_ - chargeFovOffset + boostFovOffset;
    }
};
