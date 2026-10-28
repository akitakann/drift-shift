#pragma once

#include "car.hpp"
#include <algorithm>

// Points follow real slip, not the handbrake button. A short grace period links S bends.
struct DriftRun {
    float points = 0;
    float total = 0;
    float best = 0;
    float seconds = 0;
    float grace = 0;
    float messageTime = 0;
    float lastBank = 0;
    bool active = false;
    bool failed = false;

    float multiplier() const { return 1.0f + std::min(3.0f, seconds / 3.0f); }

    // Returns true exactly once when a completed slide is banked.
    bool update(const Car& car, float dt) {
        messageTime = std::max(0.0f, messageTime - dt);
        float angle = std::fabs(car.slipAngle) * 57.29578f;
        bool qualifying = car.onAsphalt && car.speed > 10.0f && angle > 12.0f && angle < 65.0f;
        if (car.impact > 5.0f || (!car.onAsphalt && points > 0)) {
            if (points > 0) { messageTime = 1.4f; failed = true; }
            points = seconds = grace = 0;
            active = false;
            return false;
        }
        active = qualifying;
        if (qualifying) {
            seconds += dt;
            grace = 0.95f;
            float quality = clampf((angle - 10.0f) / 25.0f, 0.15f, 1.25f);
            points += car.speed * quality * multiplier() * dt * 12.0f;
        } else if (points > 0) {
            grace -= dt;
            if (grace <= 0) {
                lastBank = points;
                total += points;
                best = std::max(best, points);
                points = seconds = grace = 0;
                failed = false;
                messageTime = 2.0f;
                return true;
            }
        }
        return false;
    }
};
