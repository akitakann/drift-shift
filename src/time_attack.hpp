#pragma once

#include "track.hpp"

#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

// One-lap time attack on the closed course: a 3-2-1 countdown from a standstill,
// then lap after lap. Splits are taken where Suzuka's named corners begin, and
// each sector is compared with the best time ever driven through it. Times are
// simulation seconds, so a slow-motion presentation never changes them.
class TimeAttack {
public:
    enum class Phase { Countdown, Running };
    struct Checkpoint { float s = 0; std::string label; };
    struct Event {
        enum class Type { Count, Go, Sector, Lap } type = Type::Count;
        int index = 0;        // Count: 3/2/1. Sector: sector number from 0. Lap: completed lap number.
        float time = 0;       // Sector or lap time, seconds.
        float delta = 0;      // Against the previous best; meaningful only when hadBest.
        bool hadBest = false; // A previous best existed to compare with.
        bool best = false;    // This time is a new best (always true without a previous one).
    };

    static constexpr float kCountdown = 3.0f; // "3", "2" and "1" each show for a second, then GO.
    static constexpr float kResolution = 0.005f; // A best must be faster by more than the shown 0.01s.

    Phase phase = Phase::Countdown;
    float countdown = 0;   // Seconds since the countdown began.
    float lapTime = 0;     // Current lap, seconds since GO or since the line.
    int lap = 1;           // The lap being driven.
    float lastLap = 0;     // Previous completed lap, 0 before the first.
    float bestLap = 0;     // 0 means no best yet.
    std::vector<Checkpoint> checkpoints; // Split points in lap order; the line closes the lap.
    std::vector<float> bestSectors;      // One per sector (checkpoints + 1), 0 means none yet.

    // Splits where these named sections begin; sections the course lacks are skipped.
    void configure(const Track& track) {
        checkpoints.clear();
        length_ = track.closed ? track.length : 0;
        static const char* const kSplits[][2] = {
            {"'S' Curves", "S CURVES"},
            {"Degner Curve 1", "DEGNER"},
            {"NISSIN Brake Hairpin", "HAIRPIN"},
            {"Spoon Curve", "SPOON"},
            {"130R", "130R"},
            {"Hitachi Automotive Systems Chicane", "CHICANE"},
        };
        for (const auto& split : kSplits) {
            int id = -1;
            for (std::size_t i = 0; i < track.names.size(); ++i)
                if (track.names[i] == split[0]) { id = int(i); break; }
            if (id < 0) continue;
            for (const auto& sample : track.samples) {
                if (sample.nameId != id) continue;
                if (checkpoints.empty() || sample.s > checkpoints.back().s) checkpoints.push_back({sample.s, split[1]});
                break;
            }
        }
        bestSectors.assign(checkpoints.size() + 1, 0.0f);
        bestLap = 0;
    }

    bool enabled() const { return length_ > 0; }
    std::size_t sectorCount() const { return checkpoints.size() + 1; }
    const char* sectorLabel(int sector) const {
        return sector >= 0 && std::size_t(sector) < checkpoints.size() ? checkpoints[std::size_t(sector)].label.c_str() : "FINISH";
    }

    // Starts the countdown from the car's current place on the course.
    void reset(float startS) {
        phase = Phase::Countdown;
        countdown = lapTime = splitTime_ = lastLap = 0;
        lap = 1;
        next_ = 0;
        previousS_ = startS;
        hasPrevious_ = std::isfinite(startS);
    }

    // Advances by dt. During the countdown the car is held, so s is ignored.
    std::vector<Event> update(float dt, float s) {
        std::vector<Event> events;
        if (!enabled() || !(dt > 0) || !std::isfinite(dt)) return events;
        if (phase == Phase::Countdown) {
            const float before = countdown;
            countdown += dt;
            for (int mark = 0; mark < 3; ++mark)
                if ((mark == 0 && before == 0) || (before < float(mark) && countdown >= float(mark)))
                    events.push_back({Event::Type::Count, 3 - mark});
            if (countdown >= kCountdown) {
                phase = Phase::Running;
                events.push_back({Event::Type::Go});
                previousS_ = s;
                hasPrevious_ = std::isfinite(s);
            }
            return events;
        }
        lapTime += dt;
        if (!std::isfinite(s)) return events;
        if (!hasPrevious_) { previousS_ = s; hasPrevious_ = true; return events; }
        const float moved = forwardDistance(previousS_, s);
        if (moved > 0) {
            if (next_ < checkpoints.size() && crossed(previousS_, moved, checkpoints[next_].s)) {
                events.push_back(closeSector(int(next_)));
                ++next_;
            } else if (next_ == checkpoints.size() && crossed(previousS_, moved, 0.0f)) {
                events.push_back(closeSector(int(next_)));
                Event done{Event::Type::Lap, lap, lapTime};
                done.hadBest = bestLap > 0;
                done.delta = done.hadBest ? lapTime - bestLap : 0;
                done.best = !done.hadBest || lapTime < bestLap - kResolution;
                if (done.best) bestLap = lapTime;
                events.push_back(done);
                lastLap = lapTime;
                ++lap;
                lapTime = splitTime_ = 0;
                next_ = 0;
            }
        }
        previousS_ = s;
        return events;
    }

    // Seconds into the countdown digit currently shown, and which digit (3/2/1), or 0 after GO.
    int countdownDigit() const {
        if (phase != Phase::Countdown) return 0;
        return countdown < 1 ? 3 : countdown < 2 ? 2 : 1;
    }

private:
    float length_ = 0;
    float previousS_ = 0;
    float splitTime_ = 0;
    std::size_t next_ = 0;
    bool hasPrevious_ = false;

    // Forward travel along the lap since the last step; zero for reverse or a jump.
    float forwardDistance(float from, float to) const {
        float d = to - from;
        if (d < -0.5f * length_) d += length_;
        if (d > 0.5f * length_) d -= length_;
        return d > 0 && d < 80.0f ? d : 0.0f;
    }
    // True when the forward step [from, from + moved] passes position c (mod length).
    bool crossed(float from, float moved, float c) const {
        float k = std::fmod(c - from, length_);
        if (k < 0) k += length_;
        return k > 0 && k <= moved;
    }
    Event closeSector(int sector) {
        Event e{Event::Type::Sector, sector, lapTime - splitTime_};
        splitTime_ = lapTime;
        float& best = bestSectors[std::size_t(sector)];
        e.hadBest = best > 0;
        e.delta = e.hadBest ? e.time - best : 0;
        e.best = !e.hadBest || e.time < best - kResolution;
        if (e.best) best = e.time;
        return e;
    }
};
