#pragma once

#include "geom.hpp"

// These are experiment hypotheses, not values established by the cited research.
enum class FeelGroup { Handling, Camera, Effects, Audio, Fov, Combined };

struct FeelSettings {
    float recoveryTime = 0.8f;
    float cameraTravelWeight = 0.72f;
    float cameraResponseTime = 0.33f; // time to remove 90% of position/orientation error
    float fovGain = 0;
    float smokeScale = 1;
    float audioScale = 1;
    float engineGain = 1;
    float tireGain = 1;
    float cameraDistance = 5;
    float cameraHeight = 2.05f;
    float cameraLookAhead = 3;
    float cameraEntryTime = 0.42f;
    float cameraExitTime = 0.55f;
    float cameraExitPower = 3;
    float cameraPreviewWeight = 0.35f;
    float baseFov = 60;
    float boostFovPeak = 5;
    float boostFovAttack = 0.10f;
    float boostFovReturn = 1.80f;
    float chargeFovPeak = 1;
    float chargeShakeScale = 1;
    float boostSetbackPeak = 0.25f;
    float boostSetbackAttack = 0.10f;
    float boostSetbackReturn = 0.80f;
    float boostKickReturn = 0.30f;
    float boostSpeedScale = 1;
    float boostDurationScale = 1;
    float chargeRateScale = 1;
    float driftSteerRate = 1.8f; // Full-input line turn rate, rad/s.
    float driftSteerResponse = 0.12f; // B-model time90; A retains 1.6 times the inertia.
    float topSpeedKmh = 200;
    float stageScale = 1.4f;
    float releaseDirectionTime = 0.25f;
    float accelerationTime = 1.0f; // Approximate full-throttle zero-to-cap time, seconds.
    float boostSpeedAttack = 0.10f;
    float boostSpeedHold = 0.20f;
    float boostSpeedReturn = 0.80f;
    float boostSetbackHold = 0.20f;
    float boostFovHold = 0.20f;
    float cameraSteerLookAhead = 0.20f;
    float cameraSteerLookMax = 8.0f; // Degrees.
    float bodyRollMax = 4.0f; // Visual suspension only, degrees.
    float bodyPitchMax = 3.0f;      // Hard stop for fore-aft tilt, degrees.
    float bodyThrottlePitch = 2.0f; // Nose lift under full throttle, degrees.
    float bodyBrakePitch = 2.0f;    // Tail lift under hard braking, degrees.
    float bodyBoostPitch = 2.0f;    // Nose lift at a top-tier boost's peak, degrees.
    float bodyPitchHz = 1.8f;       // Fore-aft spring; lower and looser feels more elastic.
    float bodyPitchDamping = 0.45f;
    float bodyLeanHz = 1.8f;        // Sprung-body roll/pitch frequency; higher feels stiffer.
    float bodyLeanDamping = 0.45f;  // Damping ratio; below 1 the body overshoots once and settles.
    float bodyFlickScale = 1;       // Extra weight shift thrown in at drift entry and release.
    float bodyBounceHeight = 0.035f;
    float bodyBounceReturn = 0.45f;
    float boostBlurStrength = 0.65f;
    float gravity = 9.81f;
    float landingImpactScale = 1;
    float speedCueScale = 1;
    float windGain = 0.6f;
    float suspensionTravel = 0.20f;
    float maxLaunchSpeed = 4.5f;
    float airSteerControl = 0.35f;
    float itemKickKmh = 30;   // Peak push of one dash item, km/h.
    float momentFxScale = 1;  // Strength of the screen accents at GO, items, sector bests, laps and records.
    float engineRevScale = 1;
    float engineRevReturn = 0.65f;
};

inline float finiteFeelValue(float value, float fallback, float low, float high) {
    return clampf(std::isfinite(value) ? value : fallback, low, high);
}

inline FeelSettings sanitizeFeelSettings(FeelSettings settings) {
    const FeelSettings defaults;
#define FEEL_BOUND(field, low, high) settings.field = finiteFeelValue(settings.field, defaults.field, low, high)
    FEEL_BOUND(recoveryTime, 0.3f, 1.5f);
    FEEL_BOUND(cameraTravelWeight, 0.35f, 0.95f);
    FEEL_BOUND(cameraResponseTime, 0.05f, 1.0f);
    FEEL_BOUND(fovGain, 0, 6);
    FEEL_BOUND(smokeScale, 0, 2);
    FEEL_BOUND(audioScale, 0, 1.5f);
    FEEL_BOUND(engineGain, 0, 1.5f);
    FEEL_BOUND(tireGain, 0, 1.5f);
    FEEL_BOUND(cameraDistance, 3, 10);
    FEEL_BOUND(cameraHeight, 1.2f, 5);
    FEEL_BOUND(cameraLookAhead, 0, 12);
    FEEL_BOUND(cameraEntryTime, 0.10f, 1.2f);
    FEEL_BOUND(cameraExitTime, 0.15f, 2);
    FEEL_BOUND(cameraExitPower, 1, 5);
    FEEL_BOUND(cameraPreviewWeight, 0, 0.65f);
    FEEL_BOUND(baseFov, 45, 80);
    FEEL_BOUND(boostFovPeak, 0, 12);
    FEEL_BOUND(boostFovAttack, 0.03f, 0.50f);
    FEEL_BOUND(boostFovReturn, 0.30f, 4);
    FEEL_BOUND(chargeFovPeak, 0, 3);
    FEEL_BOUND(chargeShakeScale, 0, 2);
    FEEL_BOUND(boostSetbackPeak, 0, 0.60f);
    FEEL_BOUND(boostSetbackAttack, 0.03f, 0.50f);
    FEEL_BOUND(boostSetbackReturn, 0.20f, 2.0f);
    FEEL_BOUND(boostKickReturn, 0.10f, 1.2f);
    FEEL_BOUND(boostSpeedScale, 0, 2);
    FEEL_BOUND(boostDurationScale, 0.25f, 3);
    FEEL_BOUND(chargeRateScale, 0.25f, 3);
    FEEL_BOUND(driftSteerRate, 0.3f, 3);
    FEEL_BOUND(driftSteerResponse, 0.05f, 0.5f);
    FEEL_BOUND(topSpeedKmh, 60, 200);
    FEEL_BOUND(stageScale, 1, 2.5f);
    FEEL_BOUND(releaseDirectionTime, 0.10f, 0.60f);
    FEEL_BOUND(accelerationTime, 0.2f, 3);
    FEEL_BOUND(boostSpeedAttack, 0.03f, 0.30f);
    FEEL_BOUND(boostSpeedHold, 0, 0.60f);
    FEEL_BOUND(boostSpeedReturn, 0.20f, 2.0f);
    FEEL_BOUND(boostSetbackHold, 0, 0.60f);
    FEEL_BOUND(boostFovHold, 0, 0.60f);
    FEEL_BOUND(cameraSteerLookAhead, 0, 0.50f);
    FEEL_BOUND(cameraSteerLookMax, 0, 18);
    FEEL_BOUND(bodyRollMax, 0, 10);
    FEEL_BOUND(bodyPitchMax, 0, 15);
    FEEL_BOUND(bodyThrottlePitch, 0, 10);
    FEEL_BOUND(bodyBrakePitch, 0, 10);
    FEEL_BOUND(bodyBoostPitch, 0, 12);
    FEEL_BOUND(bodyPitchHz, 0.8f, 4);
    FEEL_BOUND(bodyPitchDamping, 0.2f, 0.9f);
    FEEL_BOUND(bodyLeanHz, 0.8f, 4);
    FEEL_BOUND(bodyLeanDamping, 0.2f, 0.9f);
    FEEL_BOUND(bodyFlickScale, 0, 2);
    FEEL_BOUND(bodyBounceHeight, 0, 0.08f);
    FEEL_BOUND(bodyBounceReturn, 0.20f, 1.0f);
    FEEL_BOUND(boostBlurStrength, 0, 2);
    FEEL_BOUND(gravity, 6, 22);
    FEEL_BOUND(landingImpactScale, 0, 2);
    FEEL_BOUND(speedCueScale, 0, 2);
    FEEL_BOUND(windGain, 0, 1.5f);
    FEEL_BOUND(suspensionTravel, 0.05f, 0.45f);
    FEEL_BOUND(maxLaunchSpeed, 2, 8);
    FEEL_BOUND(airSteerControl, 0, 0.8f);
    FEEL_BOUND(itemKickKmh, 0, 60);
    FEEL_BOUND(momentFxScale, 0, 2);
    FEEL_BOUND(engineRevScale, 0, 2);
    FEEL_BOUND(engineRevReturn, 0.20f, 1.5f);
#undef FEEL_BOUND
    return settings;
}

inline FeelSettings feelSettings(FeelGroup group, int candidate) {
    FeelSettings out;
    candidate = candidate < 0 ? 0 : (candidate > 2 ? 2 : candidate);
    switch (group) {
        case FeelGroup::Handling: out.recoveryTime = 0.6f + 0.2f * candidate; break;
        case FeelGroup::Camera: {
            const float weights[] = {0.5f, 0.72f, 0.9f};
            out.cameraTravelWeight = weights[candidate]; break;
        }
        case FeelGroup::Effects: out.smokeScale = float(candidate); break;
        case FeelGroup::Audio: {
            const float gains[] = {0, 1, 1.5f};
            out.audioScale = gains[candidate]; break;
        }
        case FeelGroup::Fov: out.fovGain = 3.0f * candidate; break;
        case FeelGroup::Combined: break;
    }
    return out;
}

inline float lateralSlipSpeed(float speed, float slipAngle) {
    return std::fabs(speed * std::sin(slipAngle));
}

inline float slipFeedback(float speed, float slipAngle, bool asphalt) {
    return asphalt && speed > 5.0f ? clampf((lateralSlipSpeed(speed, slipAngle) - 1.5f) / 6.5f, 0, 1) : 0;
}
