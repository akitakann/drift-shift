#pragma once

#include "car.hpp"

#include <cstddef>
#include <vector>

namespace TrialCourse {
constexpr float entryS = 35.0f;
constexpr float cornerExitS = 113.53981634f;
constexpr float finishS = cornerExitS + 12.0f;
constexpr float entrySpeedKmh = 80.0f;
constexpr float timeLimit = 15.0f;
constexpr float holdMinSlipDeg = 35.0f;
constexpr float holdMaxSlipDeg = 70.0f;
} // namespace TrialCourse

namespace PracticeSession {
constexpr std::size_t maxSamples = 36000; // Keep driving after ten minutes of recorded samples.
} // namespace PracticeSession

struct TrialSample {
    float time = 0;
    DriveInput input;
    float speed = 0;       // m/s
    float slip = 0;        // signed radians: body yaw minus travel yaw
    float slipSpeed = 0;   // absolute lateral speed, m/s
    float s = 0;
    float lateral = 0;
    bool asphalt = true;
    float impact = 0;
    float height = 0;
    float verticalSpeed = 0;
    bool grounded = true;
};

struct TrialMetrics {
    float duration = 0;
    float holdSeconds = 0; // longest uninterrupted hold inside the TrialCourse slip-angle band
    float entrySpeedKmh = 0;
    float exitSpeedKmh = 0;
    float exitSlipDeg = 0;
    float maxSlipDeg = 0;
    float maxLateral = 0;
    int contacts = 0;
    int offroad = 0;
    bool finished = false;
    bool passed = false;
    bool timedOut = false;
};

class TrialSession {
public:
    TrialMetrics metrics;
    std::vector<TrialSample> samples;
    bool samplesTruncated = false;

    void reset();
    // Call once, after each fixed simulation step. Finished sessions are immutable.
    void tick(const Car& car, const DriveInput& input, float dt);
    // Free practice only records motion; it never finishes at a gate or timeout.
    void tickPractice(const Car& car, const DriveInput& input, float dt);

private:
    float previousS_ = 0;
    float currentHold_ = 0;
    float contactQuiet_ = 1;
    bool hasPrevious_ = false;
    bool entered_ = false;
    bool wasOffroad_ = false;
};
