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
        rollVelocity_ = pitchVelocity_ = 0;
        wasEntry_ = wasHolding_ = false;
        previousVelocity_ = {};
        previousYaw_ = previousTravelYaw_ = 0;
        previousSpeed_ = previousBoostBonus_ = 0;
        observedBoostSerial_ = 0;
        observedLandingSerial_ = 0;
        brakeHeld_ = false;
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
            previousYaw_ = previousTravelYaw_ = 0;
            ready_ = false;
            return;
        }
        previousVelocity_ = {std::sin(car.travelYaw) * car.speed, 0,
                             std::cos(car.travelYaw) * car.speed};
        previousYaw_ = float(std::remainder(double(car.yaw), 6.28318530717958647692));
        previousTravelYaw_ = float(std::remainder(double(car.travelYaw), 6.28318530717958647692));
        previousSpeed_ = car.speed;
        previousBoostBonus_ = boostBonus(car);
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
        bindLean(roll, rollVelocity_, maxRoll);
        bindLean(pitch, pitchVelocity_, maxPitch);
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
        const float travelYaw = float(std::remainder(double(car.travelYaw), 6.28318530717958647692));
        const bool braking = std::isfinite(brake) && brake > 0.05f;
        if (!ready_) {
            previousVelocity_ = velocity;
            previousYaw_ = bodyYaw;
            previousTravelYaw_ = travelYaw;
            previousSpeed_ = car.speed;
            previousBoostBonus_ = boostBonus(car);
            observedBoostSerial_ = car.boostSerial;
            observedLandingSerial_ = car.landingSerial;
            brakeHeld_ = braking;
            wasEntry_ = car.driftPhase == DriftPhase::Entry;
            wasHolding_ = car.driftHoldActive;
            ready_ = true;
            return;
        }

        // The midpoint body frame avoids a fictitious longitudinal acceleration
        // from a finite difference around a constant-speed corner. wrapPi keeps
        // this frame continuous when either angle crosses +/-180 degrees.
        const float midYaw = previousYaw_ + 0.5f * wrapPi(bodyYaw - previousYaw_);
        const double ax = (double(velocity.x) - double(previousVelocity_.x)) / double(dt);
        const double az = (double(velocity.z) - double(previousVelocity_.z)) / double(dt);
        const double forwardAcceleration = ax * std::sin(midYaw) + az * std::cos(midYaw);
        const float previousAbsSpeed = std::fabs(previousSpeed_);
        const double speedAcceleration = (double(std::fabs(car.speed)) - previousAbsSpeed) / double(dt);
        // Sideways load comes only from the path turning, seen along the body's
        // own right axis. A boost's thrust along a slipped path is not a
        // cornering force and must not throw the body over on release.
        const float pathTurn = wrapPi(travelYaw - previousTravelYaw_);
        const float midTravel = previousTravelYaw_ + 0.5f * pathTurn;
        const double cornerAcceleration = 0.5 * (double(car.speed) + double(previousSpeed_)) *
            (double(pathTurn) / double(dt)) * std::cos(double(wrapPi(midYaw - midTravel)));
        // Fore-aft load follows the driven speed only. A boost's own push is
        // shown separately below, and its fading is not a brake.
        const float bonus = boostBonus(car);
        const double driveAcceleration =
            ((double(std::fabs(car.speed)) - bonus) - (double(previousAbsSpeed) - previousBoostBonus_)) / double(dt);
        previousVelocity_ = velocity;
        previousYaw_ = bodyYaw;
        previousTravelYaw_ = travelYaw;
        previousSpeed_ = car.speed;
        previousBoostBonus_ = bonus;

        brakeCooldown_ = std::fmax(0, brakeCooldown_ - dt);
        const bool newBoost = car.boostSerial != observedBoostSerial_;
        observedBoostSerial_ = car.boostSerial;
        const bool validBoost = settings.boostSpeedScale > 0 && car.boostLevel > 0 && car.boostLevel <= 3 &&
            std::isfinite(car.boostRemaining) && car.boostRemaining > 0 &&
            car.speed > 0.5f && car.grounded && car.onAsphalt && !car.driftHeld && car.impact <= 0.05f;
        const bool brakeOnset = braking && !brakeHeld_;
        brakeHeld_ = braking;
        if (newBoost && validBoost) {
            kickBounce(1.0f, settings);
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

        // The body sits on a spring: loads set where it wants to lean, and an
        // underdamped spring carries it there, a little past, and back once.
        // Steady loads use 75% of the limit (soft-saturated) so that headroom
        // stays for that overshoot instead of pinning the body at its stop.
        // Positive roll lifts the car's right side in carModelMatrix, so a right
        // turn leans the body left, towards the outside of the corner.
        // A held slide also scrubs its tyres sideways, adding a mild outward lean.
        const float slide = car.drifting && car.driftHoldActive && car.onAsphalt ?
            kSlideLean * clampf(std::sin(car.slipAngle) / 0.766f, -1, 1) : 0;
        const float rollTarget = !car.grounded || std::fabs(car.speed) < 0.25f ? 0 :
            maxRoll * kTargetShare * std::tanh(softLoad(cornerAcceleration, 10.0, 0.10) + slide);
        // Pitch is the elastic, exaggerated axis. Throttle lifts the nose,
        // braking throws the tail up, and a boost rears the nose up for as long
        // as its push lasts. The sideways part of a slide never pitches the car.
        float pitchTarget = 0;
        if (car.grounded) {
            const float load = std::tanh(softLoad(driveAcceleration, 12.0, 0.20));
            pitchTarget = radians * (load >= 0 ? settings.bodyThrottlePitch : settings.bodyBrakePitch) * load;
            const bool boosting = settings.boostSpeedScale > 0 && car.boostLevel > 0 && car.boostLevel <= 3 &&
                std::isfinite(car.boostRemaining) && car.boostRemaining > 0 && car.speed > 0.5f;
            if (boosting)
                pitchTarget += radians * settings.bodyBoostPitch * kBoostTier[car.boostLevel] * car.boostEnvelopeAmount();
        }

        // Weight is thrown outward as the slide starts, and back across as it
        // is released. Both are velocity kicks, so the spring shapes them.
        const bool entry = car.driftPhase == DriftPhase::Entry;
        const float unit = unitPeak(settings.bodyLeanHz, settings.bodyLeanDamping);
        if (entry && !wasEntry_ && car.grounded && car.speed > 5 && car.driftDirection != 0)
            rollVelocity_ += car.driftDirection * kEntryFlick * settings.bodyFlickScale * maxRoll / unit;
        if (wasHolding_ && !car.driftHoldActive && car.releaseEaseActive && car.grounded)
            rollVelocity_ -= roll * kReleaseFlick * settings.bodyFlickScale / unit;
        wasEntry_ = entry;
        wasHolding_ = car.driftHoldActive;

        spring(roll, rollVelocity_, rollTarget, dt, settings.bodyLeanHz, settings.bodyLeanDamping);
        spring(pitch, pitchVelocity_, pitchTarget, dt, settings.bodyPitchHz, settings.bodyPitchDamping);
        bindLean(roll, rollVelocity_, maxRoll);
        bindLean(pitch, pitchVelocity_, maxPitch);
    }

private:
    Vec3 previousVelocity_;
    float previousYaw_ = 0, previousTravelYaw_ = 0;
    float previousSpeed_ = 0, previousBoostBonus_ = 0;
    unsigned observedBoostSerial_ = 0;
    unsigned observedLandingSerial_ = 0;
    bool brakeHeld_ = false;
    float brakeCooldown_ = 0;
    float rollVelocity_ = 0, pitchVelocity_ = 0; // rad/s, carried by the body spring.
    bool wasEntry_ = false, wasHolding_ = false;
    static constexpr float kTargetShare = 0.75f; // Steady lean uses this share of the limit.
    static constexpr float kSlideLean = 0.35f;   // Tyre-scrub lean of a held slide, in load units.
    static constexpr float kEntryFlick = 0.35f;  // Entry kick's own peak, share of the roll limit.
    static constexpr float kReleaseFlick = 0.6f; // Release kick's own peak, share of the current roll.
    static constexpr float kBoostTier[4] = {0, 0.7f, 0.85f, 1}; // Boost nose lift by tier.
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
    static float boostBonus(const Car& car) {
        return std::isfinite(car.boostSpeedBonus) && car.boostSpeedBonus > 0 ? car.boostSpeedBonus : 0;
    }
    // Load beyond a small deadband, in units of "full"; tanh later saturates it softly.
    static float softLoad(double acceleration, double full, double deadband) {
        if (!std::isfinite(acceleration)) return 0;
        const double magnitude = std::fabs(acceleration);
        if (magnitude <= deadband) return 0;
        const float amount = float((magnitude - deadband) / full);
        return acceleration < 0 ? -amount : amount;
    }
    // Advances an underdamped spring towards a target held over dt. The step is
    // the exact solution, so 30/60/120Hz and irregular frames give one motion.
    static void spring(float& value, float& velocity, float target, float dt, float hz, float damping) {
        const float omega = 6.283185307f * hz;
        const float decay = damping * omega;
        const float frequency = omega * std::sqrt(1 - damping * damping);
        const float offset = value - target;
        const float attenuation = std::exp(-decay * dt);
        const float cosine = std::cos(frequency * dt), sine = std::sin(frequency * dt);
        value = target + attenuation * (offset * cosine + (velocity + decay * offset) * sine / frequency);
        velocity = attenuation * (velocity * cosine - (decay * velocity + omega * omega * offset) * sine / frequency);
        // A settled body comes to rest exactly instead of trembling forever.
        if (target == 0 && std::fabs(value) < 0.000001f && std::fabs(velocity) < 0.00001f) value = velocity = 0;
    }
    // Peak displacement produced by a unit velocity kick from rest.
    static float unitPeak(float hz, float damping) {
        const float omega = 6.283185307f * hz;
        const float decay = damping * omega;
        const float frequency = omega * std::sqrt(1 - damping * damping);
        const float peakTime = std::atan(frequency / decay) / frequency;
        return std::exp(-decay * peakTime) * std::sin(frequency * peakTime) / frequency;
    }
    // The limit is a bump stop: the angle cannot pass it and stops moving outward.
    static void bindLean(float& value, float& velocity, float limit) {
        if (!(limit > 0) || !std::isfinite(value) || !std::isfinite(velocity)) { value = velocity = 0; return; }
        if (value > limit) { value = limit; velocity = std::fmin(velocity, 0.0f); }
        if (value < -limit) { value = -limit; velocity = std::fmax(velocity, 0.0f); }
    }
};
