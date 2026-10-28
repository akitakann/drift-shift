#pragma once

#include "car.hpp"
#include "tuning.hpp"

#include <cmath>

// Sprung-body presentation only. It observes the car's velocity; it never changes
// the trajectory, grip, tire contacts, or camera. Angles are radians; heave is m.
struct VehiclePresentation {
    float roll = 0;
    float pitch = 0;
    float heave = 0;
    float landingCompression = 0; // Body-only suspension compression, never wheel/physical height.

    void reset() {
        roll = pitch = 0;
        previousVelocity_ = {};
        previousYaw_ = 0;
        previousSpeed_ = 0;
        observedBoostSerial_ = 0;
        observedLandingSerial_ = 0;
        brakeHeld_ = false;
        wasEntry_ = false;
        brakeCooldown_ = 0;
        clearBounce();
        clearLanding();
        ready_ = false;
    }

    // Live tuning can directly replace the physical speed while paused. Such an
    // edit is not an acceleration: keep the visible suspension pose, but start
    // measuring its future velocity changes from the edited physical state.
    // Ordinary inactive/zero-dt observations must not call this implicitly.
    void rebaseVelocity(const Car& car) {
        if (!std::isfinite(car.speed) || !std::isfinite(car.travelYaw) || !std::isfinite(car.yaw)) {
            previousVelocity_ = {};
            previousYaw_ = 0;
            ready_ = false;
            return;
        }
        previousVelocity_ = {std::sin(car.travelYaw) * car.speed, 0,
                             std::cos(car.travelYaw) * car.speed};
        previousYaw_ = float(std::remainder(double(car.yaw), 6.28318530717958647692));
        previousSpeed_ = car.speed;
        observedBoostSerial_ = car.boostSerial;
        observedLandingSerial_ = car.landingSerial;
        ready_ = true;
    }

    // Call after Car::update at the simulation's fixed step. Inactive/zero-dt
    // calls only apply live angle limits; they do not consume velocity samples
    // or advance the pose. Reset this observer when resetting the physical car.
    void update(float dt, const Car& car, const FeelSettings& rawSettings, bool active = true, float brake = 0) {
        const FeelSettings settings = sanitizeFeelSettings(rawSettings);
        constexpr float radians = 3.14159265358979323846f / 180.0f;
        const float maxRoll = settings.bodyRollMax * radians;
        const float maxPitch = settings.bodyPitchMax * radians;
        roll = boundedAngle(roll, maxRoll);
        pitch = boundedAngle(pitch, maxPitch);
        boundBounce(settings);
        boundLanding(settings);
        // Turning off the layer also discards an event pending during pause.
        // Re-enabling it must never replay a landing which happened while off.
        if (!(settings.landingImpactScale > 0)) observedLandingSerial_ = car.landingSerial;
        if (!active || !(dt > 0) || !std::isfinite(dt)) return;

        if (!std::isfinite(car.speed) || !std::isfinite(car.travelYaw) || !std::isfinite(car.yaw)) {
            // A malformed sample must not poison the next valid velocity delta.
            reset();
            return;
        }
        const Vec3 velocity{std::sin(car.travelYaw) * car.speed, 0,
                            std::cos(car.travelYaw) * car.speed};
        const float bodyYaw = float(std::remainder(double(car.yaw), 6.28318530717958647692));
        const bool braking = std::isfinite(brake) && brake > 0.05f;
        if (!ready_) {
            previousVelocity_ = velocity;
            previousYaw_ = bodyYaw;
            previousSpeed_ = car.speed;
            observedBoostSerial_ = car.boostSerial;
            observedLandingSerial_ = car.landingSerial;
            brakeHeld_ = braking;
            ready_ = true;
            return;
        }

        // The midpoint body frame avoids a fictitious longitudinal acceleration
        // from a finite difference around a constant-speed corner. wrapPi keeps
        // this frame continuous when either angle crosses +/-180 degrees.
        const float midYaw = previousYaw_ + 0.5f * wrapPi(bodyYaw - previousYaw_);
        const double ax = (double(velocity.x) - double(previousVelocity_.x)) / double(dt);
        const double az = (double(velocity.z) - double(previousVelocity_.z)) / double(dt);
        const double rightAcceleration = ax * std::cos(midYaw) - az * std::sin(midYaw);
        const double forwardAcceleration = ax * std::sin(midYaw) + az * std::cos(midYaw);
        const float previousAbsSpeed = std::fabs(previousSpeed_);
        const double speedAcceleration = (double(std::fabs(car.speed)) - previousAbsSpeed) / double(dt);
        previousVelocity_ = velocity;
        previousYaw_ = bodyYaw;
        previousSpeed_ = car.speed;

        brakeCooldown_ = std::fmax(0, brakeCooldown_ - dt);
        const bool newBoost = car.boostSerial != observedBoostSerial_;
        observedBoostSerial_ = car.boostSerial;
        const bool validBoost = settings.boostSpeedScale > 0 && car.boostLevel > 0 && car.boostLevel <= 3 &&
            std::isfinite(car.boostRemaining) && car.boostRemaining > 0 &&
            car.speed > 0.5f && car.grounded && car.onAsphalt && !car.driftHeld && car.impact <= 0.05f;
        const bool brakeOnset = braking && !brakeHeld_;
        brakeHeld_ = braking;
        const bool entry = car.driftPhase == DriftPhase::Entry;
        const bool entryOnset = entry && !wasEntry_;
        wasEntry_ = entry;
        if (newBoost && validBoost) {
            kickBounce(1.0f, settings);
        } else if (entryOnset && car.grounded) {
            // Throwing the car into a slide unloads it for a moment.
            kickBounce(0.7f, settings);
        } else if (car.grounded && brakeOnset && brakeCooldown_ <= 0 && previousAbsSpeed > 0.5f &&
                   speedAcceleration < -0.5 && forwardAcceleration < -0.5) {
            // Both a real brake edge and loss of speed are required. A changing
            // drift direction or a constant-speed body rotation is not braking.
            kickBounce(-0.6f, settings);
            brakeCooldown_ = 0.18f;
        }
        advanceBounce(dt, settings);

        const bool newLanding = car.landingSerial != observedLandingSerial_;
        observedLandingSerial_ = car.landingSerial;
        if (newLanding && car.grounded && std::isfinite(car.landingImpact) && car.landingImpact > 0.5f)
            kickLanding(clampf((car.landingImpact - 0.5f) / 5.5f, 0, 1), settings);
        advanceLanding(dt, settings);

        // These load scales are presentation hypotheses: full roll at 12m/s^2,
        // full pitch at 18m/s^2. Small deadbands remove numeric/low-speed chatter.
        // Positive roll lifts the car's right side in carModelMatrix, so a right
        // turn leans the body left, towards the outside of the corner.
        // A held slide also leans out by its angle, so the body stays loaded
        // even when the player straightens the line mid-drift.
        const float slideLean = car.drifting && car.driftHoldActive && car.onAsphalt ?
            clampf(float(std::sin(car.slipAngle)) / 0.766f, -1, 1) * 0.7f : 0;
        const float rollTarget = !car.grounded || std::fabs(car.speed) < 0.25f ? 0 :
            maxRoll * clampf(signedLoad(rightAcceleration, 12.0, 0.10) + slideLean, -1, 1);
        const float pitchTarget = car.grounded ? maxPitch * signedLoad(forwardAcceleration, 18.0, 0.20) : 0;
        roll = settle(roll, rollTarget, dt, settings.bodyResponseTime, settings.bodyRecoveryTime);
        pitch = settle(pitch, pitchTarget, dt, settings.bodyResponseTime, settings.bodyRecoveryTime);
        roll = boundedAngle(roll, maxRoll);
        pitch = boundedAngle(pitch, maxPitch);
    }

private:
    Vec3 previousVelocity_;
    float previousYaw_ = 0;
    float previousSpeed_ = 0;
    unsigned observedBoostSerial_ = 0;
    unsigned observedLandingSerial_ = 0;
    bool brakeHeld_ = false;
    bool wasEntry_ = false;
    float brakeCooldown_ = 0;
    float heaveVelocity_ = 0;
    float bounceAge_ = -1;
    float bounceReturn_ = 0.45f;
    float landingVelocity_ = 0;
    float landingAge_ = -1;
    float landingReturn_ = 0.45f;
    bool ready_ = false;

    void clearBounce() {
        heave = heaveVelocity_ = 0;
        bounceAge_ = -1;
    }
    void boundBounce(const FeelSettings& settings) {
        if (!(settings.bodyBounceHeight > 0)) {
            clearBounce();
            return;
        }
        if (bounceAge_ >= 0 && bounceReturn_ != settings.bodyBounceReturn) {
            // Preserve normalized spring phase when editing its duration. Pose
            // stays continuous, and velocity scales with the new time axis.
            heaveVelocity_ *= bounceReturn_ / settings.bodyBounceReturn;
            bounceAge_ *= settings.bodyBounceReturn / bounceReturn_;
            bounceReturn_ = settings.bodyBounceReturn;
        }
        heave = boundedAngle(heave, settings.bodyBounceHeight);
        const float speedLimit = 24 * settings.bodyBounceHeight / settings.bodyBounceReturn;
        heaveVelocity_ = boundedAngle(heaveVelocity_, speedLimit);
    }
    void kickBounce(float strength, const FeelSettings& settings) {
        if (!(settings.bodyBounceHeight > 0)) return;
        constexpr float dampingRatio = 0.72f;
        const float decay = 5.5f / settings.bodyBounceReturn;
        const float frequency = decay * std::sqrt(1 - dampingRatio * dampingRatio) / dampingRatio;
        const float peakTime = std::atan(frequency / decay) / frequency;
        const float unitPeak = std::exp(-decay * peakTime) * std::sin(frequency * peakTime) / frequency;
        // Replace rather than accumulate event velocity. The unstacked positive
        // kick reaches the configured height, the brake kick about60% of it.
        heaveVelocity_ = strength * settings.bodyBounceHeight / unitPeak;
        bounceAge_ = 0;
        bounceReturn_ = settings.bodyBounceReturn;
        boundBounce(settings);
    }
    void advanceBounce(float dt, const FeelSettings& settings) {
        if (bounceAge_ < 0 || !(settings.bodyBounceHeight > 0)) return;
        bounceAge_ += dt;
        // There is one small opposite recoil. By this deadline the remaining
        // unstacked offset is under1mm even at the8cm setting; end the tiny tail.
        if (bounceAge_ + 0.000001f >= settings.bodyBounceReturn) {
            clearBounce();
            return;
        }
        constexpr float dampingRatio = 0.72f;
        const float decay = 5.5f / settings.bodyBounceReturn;
        const float frequency = decay * std::sqrt(1 - dampingRatio * dampingRatio) / dampingRatio;
        const float attenuation = std::exp(-decay * dt);
        const float sine = std::sin(frequency * dt);
        const float cosine = std::cos(frequency * dt);
        const float oldHeave = heave;
        const float oldVelocity = heaveVelocity_;
        heave = attenuation * (oldHeave * cosine + (oldVelocity + decay * oldHeave) * sine / frequency);
        heaveVelocity_ = attenuation * (oldVelocity * cosine -
            (decay * oldVelocity + (decay * decay + frequency * frequency) * oldHeave) * sine / frequency);
        const float height = settings.bodyBounceHeight;
        if (heave > height) { heave = height; heaveVelocity_ = std::fmin(heaveVelocity_, 0); }
        if (heave < -height) { heave = -height; heaveVelocity_ = std::fmax(heaveVelocity_, 0); }
        boundBounce(settings);
    }

    void clearLanding() {
        landingCompression = landingVelocity_ = 0;
        landingAge_ = -1;
    }
    void boundLanding(const FeelSettings& settings) {
        if (!(settings.landingImpactScale > 0)) {
            clearLanding();
            return;
        }
        if (landingAge_ >= 0 && landingReturn_ != settings.bodyBounceReturn) {
            landingVelocity_ *= landingReturn_ / settings.bodyBounceReturn;
            landingAge_ *= settings.bodyBounceReturn / landingReturn_;
            landingReturn_ = settings.bodyBounceReturn;
        }
        const float depth = 0.10f * settings.landingImpactScale;
        landingCompression = std::isfinite(landingCompression) ? clampf(landingCompression, -depth, 0) : 0;
        landingVelocity_ = boundedAngle(landingVelocity_, 30 * depth / settings.bodyBounceReturn);
    }
    void kickLanding(float strength, const FeelSettings& settings) {
        if (!(settings.landingImpactScale > 0)) return;
        // A critically damped impulse compresses rapidly then returns without
        // launching the body upwards again. About6m/s reaches10cm at scale1.
        const float decay = 9.0f / settings.bodyBounceReturn;
        landingVelocity_ = -0.10f * settings.landingImpactScale * strength * decay * 2.718281828f;
        landingAge_ = 0;
        landingReturn_ = settings.bodyBounceReturn;
        boundLanding(settings);
    }
    void advanceLanding(float dt, const FeelSettings& settings) {
        if (landingAge_ < 0 || !(settings.landingImpactScale > 0)) return;
        landingAge_ += dt;
        if (landingAge_ + 0.000001f >= settings.bodyBounceReturn) {
            clearLanding();
            return;
        }
        const float decay = 9.0f / settings.bodyBounceReturn;
        const float attenuation = std::exp(-decay * dt);
        const float load = landingVelocity_ + decay * landingCompression;
        landingCompression = attenuation * (landingCompression + load * dt);
        landingVelocity_ = attenuation * (landingVelocity_ - decay * load * dt);
        const float depth = 0.10f * settings.landingImpactScale;
        if (landingCompression < -depth) {
            landingCompression = -depth;
            landingVelocity_ = std::fmax(landingVelocity_, 0);
        }
        if (landingCompression > 0) clearLanding();
        boundLanding(settings);
    }

    static float boundedAngle(float value, float limit) {
        return limit > 0 && std::isfinite(value) ? clampf(value, -limit, limit) : 0;
    }
    static float signedLoad(double acceleration, double full, double deadband) {
        if (std::isnan(acceleration)) return 0;
        const double magnitude = std::fabs(acceleration);
        if (magnitude <= deadband) return 0;
        const float amount = clampf(float((magnitude - deadband) / (full - deadband)), 0, 1);
        return acceleration < 0 ? -amount : amount;
    }
    static float settle(float value, float target, float dt, float responseTime, float recoveryTime) {
        const bool recovering = value * target >= 0 && std::fabs(target) < std::fabs(value);
        // Time means 90% response, consistent with the game's other damping.
        const float time90 = std::fmax(recovering ? recoveryTime : responseTime, 0.001f);
        const float blend = -std::expm1(-2.302585093f * dt / time90);
        const float result = value + (target - value) * blend;
        return std::fabs(result) < 0.000001f && std::fabs(target) < 0.000001f ? 0 : result;
    }
};
