#include "trial.hpp"
#include "tuning.hpp"

#include <algorithm>
#include <cmath>

void TrialSession::reset() {
    metrics = TrialMetrics{};
    samples.clear();
    samplesTruncated = false;
    previousS_ = currentHold_ = 0;
    contactQuiet_ = 1;
    hasPrevious_ = entered_ = wasOffroad_ = false;
}

void TrialSession::tickPractice(const Car& car, const DriveInput& input, float dt) {
    if (metrics.finished || !std::isfinite(dt) || dt <= 0) return;
    const float angle = std::fabs(car.slipAngle) * 57.295779513f;
    metrics.duration += dt;
    metrics.maxSlipDeg = std::max(metrics.maxSlipDeg, angle);
    metrics.maxLateral = std::max(metrics.maxLateral, std::fabs(car.ground.lateral));
    metrics.exitSpeedKmh = car.speed * 3.6f;
    metrics.exitSlipDeg = angle;
    if (samples.size() < PracticeSession::maxSamples) {
        samples.push_back({metrics.duration, input, car.speed, car.slipAngle,
                           lateralSlipSpeed(car.speed, car.slipAngle), car.ground.s,
                           car.ground.lateral, car.onAsphalt, car.impact, car.y, car.verticalSpeed, car.grounded});
    } else {
        samplesTruncated = true;
    }
    if (car.impact > 0.05f) {
        if (contactQuiet_ >= 0.2f) ++metrics.contacts;
        contactQuiet_ = 0;
    } else {
        contactQuiet_ += dt;
    }
    const bool offroad=car.grounded&&!car.onAsphalt;
    if (offroad && !wasOffroad_) ++metrics.offroad;
    wasOffroad_ = offroad;

    const bool holding = car.grounded && car.onAsphalt && car.speed > 0 &&
                         angle >= 8.0f - 0.0001f && car.impact <= 0.05f;
    if (holding && metrics.entrySpeedKmh == 0) metrics.entrySpeedKmh = car.speed * 3.6f;
    currentHold_ = holding ? currentHold_ + dt : 0;
    metrics.holdSeconds = std::max(metrics.holdSeconds, currentHold_);
}

void TrialSession::tick(const Car& car, const DriveInput& input, float dt) {
    if (metrics.finished || !std::isfinite(dt) || dt <= 0) return;
    const float angle = std::fabs(car.slipAngle) * 57.295779513f;
    metrics.duration += dt;
    metrics.maxSlipDeg = std::max(metrics.maxSlipDeg, angle);
    metrics.maxLateral = std::max(metrics.maxLateral, std::fabs(car.ground.lateral));
    samples.push_back({metrics.duration, input, car.speed, car.slipAngle,
                       lateralSlipSpeed(car.speed, car.slipAngle), car.ground.s,
                       car.ground.lateral, car.onAsphalt, car.impact, car.y, car.verticalSpeed, car.grounded});

    // Sustained scraping generates impacts every step; count the episode once.
    if (car.impact > 0.05f) {
        if (contactQuiet_ >= 0.2f) ++metrics.contacts;
        contactQuiet_ = 0;
    } else {
        contactQuiet_ += dt;
    }
    const bool offroad=car.grounded&&!car.onAsphalt;
    if (offroad && !wasOffroad_) ++metrics.offroad;
    wasOffroad_ = offroad;

    const float s = car.ground.s;
    const bool forward = hasPrevious_ && s > previousS_ && s - previousS_ < 10.0f;
    if (!entered_ && forward && previousS_ < TrialCourse::entryS && s >= TrialCourse::entryS) {
        entered_ = true;
        metrics.entrySpeedKmh = car.speed * 3.6f;
    }

    // Only the curve counts. Approach or exit donuts cannot satisfy the hold.
    const bool holding = entered_ && s >= TrialCourse::entryS && s <= TrialCourse::cornerExitS &&
                         car.grounded && car.onAsphalt && car.speed > 0 &&
                         angle >= TrialCourse::holdMinSlipDeg - 0.0001f &&
                         angle <= TrialCourse::holdMaxSlipDeg + 0.0001f && car.impact <= 0.05f;
    currentHold_ = holding ? currentHold_ + dt : 0;
    metrics.holdSeconds = std::max(metrics.holdSeconds, currentHold_);

    if (entered_ && forward && previousS_ < TrialCourse::finishS && s >= TrialCourse::finishS) {
        metrics.finished = true;
        metrics.exitSpeedKmh = car.speed * 3.6f;
        metrics.exitSlipDeg = angle;
        metrics.passed = metrics.contacts == 0 && metrics.offroad == 0 &&
                         metrics.holdSeconds >= 1.5f - 0.0001f &&
                         metrics.exitSlipDeg < 8.0f && metrics.exitSpeedKmh >= 65.0f;
    } else if (metrics.duration >= TrialCourse::timeLimit) {
        metrics.finished = true;
        metrics.timedOut = true;
        metrics.exitSpeedKmh = car.speed * 3.6f;
        metrics.exitSlipDeg = angle;
    }

    previousS_ = s;
    hasPrevious_ = true;
}
