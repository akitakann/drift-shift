#pragma once

#include "geom.hpp"

#include <algorithm>
#include <cmath>

// Screen accents for the moments that deserve them: GO, a dash cell, a sector
// best, a lap and a new record. Between moments every amount is zero, so the
// normal view stays clean. Amounts decay on real time; slow motion only
// changes how fast the simulation is fed, never the lap clock.
struct Moments {
    float flash = 0;  Vec3 flashColor{1, 1, 1};
    float edge = 0;   Vec3 edgeColor{0.25f, 1.0f, 0.45f};
    float zoom = 0;   // Punch-in toward the view centre.
    float burst = 0;  // Extra glow and colour fringing for a record.
    float goAge = -1; // Seconds since GO, for the on-screen "GO!".

    static constexpr Vec3 kSectorGreen{0.25f, 1.0f, 0.45f};
    static constexpr Vec3 kRecordGold{1.0f, 0.78f, 0.25f};
    static constexpr Vec3 kItemWarm{1.0f, 0.86f, 0.45f};

    void reset() {
        flash = edge = zoom = burst = 0;
        goAge = -1;
        slowHold_ = slowEase_ = 0;
        slowScale_ = 1;
    }

    void go() {
        raise(flash, 0.30f); flashColor = {1, 1, 1};
        raise(zoom, 0.6f);
        goAge = 0;
    }
    void item(int chain) {
        // Keep a run of cells lively without strobing: later cells add less light.
        raise(flash, chain == 0 ? 0.14f : 0.08f); flashColor = kItemWarm;
        raise(zoom, 0.55f);
    }
    void sectorBest() {
        raise(edge, 0.7f); edgeColor = kSectorGreen;
    }
    void lap(bool record) {
        if (record) {
            raise(flash, 0.85f); flashColor = kRecordGold;
            raise(edge, 1.0f); edgeColor = kRecordGold;
            raise(burst, 1.0f);
            raise(zoom, 1.0f);
            slow(0.22f, 0.9f, 0.6f);
        } else {
            raise(flash, 0.6f); flashColor = {1, 1, 1};
            raise(zoom, 0.6f);
            slow(0.32f, 0.45f, 0.35f);
        }
    }

    void tick(float realDt) {
        if (!(realDt > 0) || !std::isfinite(realDt)) return;
        flash *= std::exp(-6.0f * realDt);
        edge *= std::exp(-2.6f * realDt);
        zoom *= std::exp(-7.0f * realDt);
        burst *= std::exp(-1.8f * realDt);
        for (float* value : {&flash, &edge, &zoom, &burst}) if (*value < 0.002f) *value = 0;
        if (goAge >= 0) goAge = goAge + realDt > 3 ? -1 : goAge + realDt;
        if (slowHold_ > 0) slowHold_ = std::fmax(0.0f, slowHold_ - realDt);
        else if (slowEase_ > 0) slowEase_ = std::fmax(0.0f, slowEase_ - realDt);
    }

    // How fast simulated time runs against real time right now.
    float timeScale() const {
        if (slowHold_ > 0) return slowScale_;
        if (slowEase_ > 0 && slowEaseTotal_ > 0) {
            const float t = 1 - slowEase_ / slowEaseTotal_;
            const float smooth = t * t * (3 - 2 * t);
            return slowScale_ + (1 - slowScale_) * smooth;
        }
        return 1;
    }

private:
    float slowHold_ = 0, slowEase_ = 0, slowEaseTotal_ = 0, slowScale_ = 1;

    static void raise(float& value, float to) { value = std::max(value, to); }
    void slow(float scale, float hold, float ease) {
        slowScale_ = std::clamp(scale, 0.05f, 1.0f);
        slowHold_ = hold;
        slowEase_ = slowEaseTotal_ = ease;
    }
};
