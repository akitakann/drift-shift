#include "dash_items.hpp"
#include "moments.hpp"
#include "time_attack.hpp"
#include "tuning.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

namespace {
int failures = 0;
void check(bool condition, const char* message) {
    if (!condition) { std::fprintf(stderr, "FAIL: %s\n", message); ++failures; }
}
bool near(float a, float b, float tolerance) { return std::fabs(a - b) <= tolerance; }

// A 1000 m loop: the line at 0, "S" Curves from 200 m and Degner from 500 m.
Track loopTrack() {
    Track track;
    track.closed = true;
    track.length = 1000;
    track.names = {"Main straight", "'S' Curves", "Degner Curve 1"};
    for (int i = 0; i < 1000; ++i) {
        TrackSample sample;
        sample.s = float(i);
        sample.x = float(i);
        sample.nameId = i < 200 ? 0 : (i < 500 ? 1 : 2);
        track.samples.push_back(sample);
    }
    return track;
}

using Events = std::vector<TimeAttack::Event>;

// Drives forward at a constant speed for a duration, collecting every event.
Events drive(TimeAttack& attack, float& s, float speed, float seconds, float length = 1000) {
    Events all;
    const float dt = 1.0f / 60.0f;
    for (int frame = 0; frame < int(std::lround(seconds / dt)); ++frame) {
        s = std::fmod(s + speed * dt, length);
        for (const auto& event : attack.update(dt, s)) all.push_back(event);
    }
    return all;
}

// Drives until a lap ends (or a time cap), so the next lap starts at the line.
Events driveLap(TimeAttack& attack, float& s, float speed, float cap = 120) {
    Events all;
    const float dt = 1.0f / 60.0f;
    for (int frame = 0; frame < int(cap / dt); ++frame) {
        s = std::fmod(s + speed * dt, 1000.0f);
        bool lap = false;
        for (const auto& event : attack.update(dt, s)) {
            all.push_back(event);
            lap = lap || event.type == TimeAttack::Event::Type::Lap;
        }
        if (lap) break;
    }
    return all;
}

int count(const Events& events, TimeAttack::Event::Type type) {
    int n = 0;
    for (const auto& event : events) n += event.type == type;
    return n;
}

void testCountdown() {
    TimeAttack attack;
    attack.configure(loopTrack());
    attack.reset(980);
    std::vector<int> digits;
    bool go = false;
    float goAt = -1;
    const float dt = 1.0f / 60.0f;
    for (int frame = 0; frame < 240 && !go; ++frame) {
        for (const auto& event : attack.update(dt, 980)) {
            if (event.type == TimeAttack::Event::Type::Count) digits.push_back(event.index);
            if (event.type == TimeAttack::Event::Type::Go) { go = true; goAt = (frame + 1) * dt; }
        }
        if (!go) check(attack.lapTime == 0, "the lap clock waits for GO");
    }
    check(digits == std::vector<int>{3, 2, 1}, "the countdown announces 3, 2 and 1 once each");
    check(go && near(goAt, TimeAttack::kCountdown, 1.0f / 60.0f + 0.0001f), "GO comes three seconds after the start");
    check(attack.phase == TimeAttack::Phase::Running && attack.countdownDigit() == 0, "after GO the attack is running");
}

void testLapAndSectors() {
    TimeAttack attack;
    attack.configure(loopTrack());
    check(attack.checkpoints.size() == 2 && attack.sectorCount() == 3, "named sections become splits; the line closes a third sector");
    check(attack.checkpoints.size() == 2 && attack.checkpoints[0].label == "S CURVES" && near(attack.checkpoints[0].s, 200, 0.001f) &&
          attack.checkpoints[1].label == "DEGNER" && near(attack.checkpoints[1].s, 500, 0.001f),
          "splits sit where the named sections begin, in lap order");
    float s = 980;
    attack.reset(s);
    drive(attack, s, 0, 3.05f);
    // The line right after GO is part of the first lap, not its end.
    Events first = drive(attack, s, 50, 1.0f);
    check(count(first, TimeAttack::Event::Type::Lap) == 0 && attack.lap == 1, "crossing the line straight after the start does not end a lap");
    Events lap1 = driveLap(attack, s, 50);
    check(count(lap1, TimeAttack::Event::Type::Sector) == 3 && count(lap1, TimeAttack::Event::Type::Lap) == 1,
          "a full lap reports each sector once and then the lap");
    float sum = 0;
    float lapTime = 0;
    for (const auto& e : lap1) {
        if (e.type == TimeAttack::Event::Type::Sector) {
            sum += e.time;
            check(e.best && !e.hadBest, "the first time through a sector is its first best");
        }
        if (e.type == TimeAttack::Event::Type::Lap) {
            lapTime = e.time;
            check(e.index == 1 && e.best && !e.hadBest, "the first lap is the first record");
        }
    }
    check(near(sum, lapTime, 0.001f) && near(lapTime, 1020.0f / 50.0f, 0.05f),
          "sector times add up to the lap, which includes the run from the grid");
    check(attack.lap == 2 && near(attack.bestLap, lapTime, 0.0001f) && near(attack.lastLap, lapTime, 0.0001f),
          "after the line the next lap begins and the record is kept");

    // A faster second lap beats the record and shows how much it gained.
    Events lap2 = driveLap(attack, s, 80);
    bool sawLap = false;
    for (const auto& e : lap2) {
        if (e.type == TimeAttack::Event::Type::Sector) check(e.hadBest && e.best && e.delta < 0, "a faster sector beats its best");
        if (e.type == TimeAttack::Event::Type::Lap) {
            sawLap = true;
            check(e.index == 2 && e.hadBest && e.best && near(e.delta, 1000.0f / 80.0f - lapTime, 0.05f),
                  "a faster lap is a new record by the time it gained");
        }
    }
    check(sawLap && near(attack.bestLap, 1000.0f / 80.0f, 0.05f), "the record moves to the faster lap");

    // A slower lap keeps the record and reports how far behind it was.
    Events lap3 = driveLap(attack, s, 40);
    for (const auto& e : lap3) {
        if (e.type == TimeAttack::Event::Type::Lap) check(!e.best && e.delta > 0, "a slower lap leaves the record alone");
        if (e.type == TimeAttack::Event::Type::Sector) check(!e.best && e.delta > 0, "slower sectors are not bests");
    }
    check(near(attack.bestLap, 1000.0f / 80.0f, 0.05f), "the record survives a slower lap");
}

void testNoShortcuts() {
    TimeAttack attack;
    attack.configure(loopTrack());
    float s = 980;
    attack.reset(s);
    drive(attack, s, 0, 3.05f);
    // Reversing back over the line and forward again cannot fake a lap.
    Events events = drive(attack, s, 50, 0.6f); // over the line to ~10 m
    events = drive(attack, s, -50, 1.0f);       // back to ~960 m
    for (int i = 0; i < 3; ++i) {
        Events more = drive(attack, s, 50, 1.0f);
        more = drive(attack, s, -50, 1.0f);
        events.insert(events.end(), more.begin(), more.end());
    }
    check(count(events, TimeAttack::Event::Type::Lap) == 0 && count(events, TimeAttack::Event::Type::Sector) == 0,
          "rocking back and forth over the line never completes a lap");
    // A jump across the loop (a reset or a teleport) does not count as driving past a split.
    s = 150;
    attack.update(1.0f / 60.0f, s);
    s = 520;
    Events jump = attack.update(1.0f / 60.0f, s);
    check(jump.empty(), "a sudden jump along the course passes no split");
    // Invalid steps are ignored.
    const float before = attack.lapTime;
    attack.update(std::nanf(""), s);
    attack.update(-1, s);
    check(attack.lapTime == before, "invalid time steps do not move the clock");
}

void testSuzuka() {
    Track track;
    bool loaded = false;
    for (const char* path : {"assets/track/suzuka.txt", "../assets/track/suzuka.txt"})
        if (track.load(path)) { loaded = true; break; }
    check(loaded, "the Suzuka course loads");
    if (!loaded) return;
    track.scaleBy(2);
    TimeAttack attack;
    attack.configure(track);
    const char* labels[] = {"S CURVES", "DEGNER", "HAIRPIN", "SPOON", "130R", "CHICANE"};
    check(attack.checkpoints.size() == 6, "Suzuka has six splits");
    for (std::size_t i = 0; i < attack.checkpoints.size() && i < 6; ++i) {
        check(attack.checkpoints[i].label == labels[i], "Suzuka's splits follow the lap in order");
        if (i > 0) check(attack.checkpoints[i].s > attack.checkpoints[i - 1].s + 200, "Suzuka's splits are well apart");
    }

    DashItems dash;
    dash.build(track);
    const float limit = track.halfWidth - DashItems::kEdgeMargin;
    int line = 0, weave = 0, inside = 0, outside = 0, onRoad = 0;
    for (const auto& item : dash.items) {
        const TrackPose pose = track.query(item.position.x, item.position.z, item.position.y - DashItems::kHover, item.s);
        const float surface = pose.centerY + pose.cross * pose.lateral;
        onRoad += pose.found && std::fabs(pose.lateral) <= limit + 0.3f &&
                  std::fabs(item.position.y - DashItems::kHover - surface) < 0.5f;
        if (item.kind == DashItems::Kind::Line) {
            ++line;
            inside += item.lateral > 0.5f * limit;
            outside += item.lateral < -0.5f * limit;
        } else {
            ++weave;
        }
    }
    check(dash.items.size() >= 100 && dash.items.size() <= 200, "a lap carries a readable number of cells");
    check(line >= 60 && weave >= 30, "cells trace the bends and line the long straights");
    check(onRoad == int(dash.items.size()), "every cell floats over the road, clear of its edges");
    check(inside >= 15 && outside >= 15, "the bend lines swing across the road, not down its middle");
    float worstLine = 0;
    for (std::size_t i = 0; i < track.samples.size(); ++i) worstLine = std::fmax(worstLine, std::fabs(dash.line[i]));
    check(dash.line.size() == track.samples.size() && worstLine <= limit + 0.001f, "the drifting line stays on the road");

    // Cells never bunch up, and the rows on a straight switch sides far apart.
    float closest = 1e9f;
    for (std::size_t i = 1; i < dash.items.size(); ++i) closest = std::fmin(closest, dash.items[i].s - dash.items[i - 1].s);
    check(closest >= 10, "cells along the lap keep at least 10 m apart");
    int rows = 0, switches = 0;
    float lastRowS = -1e9f, lastSide = 0, tightest = 1e9f;
    for (const auto& item : dash.items) {
        if (item.kind != DashItems::Kind::Weave) { lastRowS = -1e9f; continue; }
        if (item.s - lastRowS > DashItems::kWeaveSpacing * 1.5f) {
            const float side = item.lateral > 0 ? 1.0f : -1.0f;
            if (lastRowS > -1e8f) {
                tightest = std::fmin(tightest, item.s - lastRowS);
                switches += side != lastSide;
            }
            lastSide = side;
            ++rows;
        }
        lastRowS = item.s;
    }
    check(rows >= 10 && switches >= rows / 2, "rows on the straights alternate sides");
    check(tightest >= DashItems::kWeavePitch - DashItems::kWeaveSpacing * (DashItems::kWeaveRow - 1) - 1,
          "the next row is far enough along to steer across to it");
}

void testPickupAndPush() {
    Track area;
    area.makePracticeArea();
    FeelSettings settings;
    Car car;
    car.reset(area);
    car.applyTuning(settings);
    DriveInput cruise;
    cruise.throttle = 1;
    for (int i = 0; i < 600; ++i) car.update(1.0f / 60.0f, cruise, area);
    const float top = car.speed;
    check(near(top, settings.topSpeedKmh / 3.6f, 0.05f), "the car settles at its top speed before the push test");

    DashItems dash;
    DashItems::Item item;
    item.position = {car.x + std::sin(car.yaw) * 1.0f, car.y + DashItems::kHover, car.z + std::cos(car.yaw) * 1.0f};
    dash.items = {item, item};
    dash.items[1].position = dash.items[1].position + Vec3{std::sin(car.yaw) * 40, 0, std::cos(car.yaw) * 40};
    auto picked = dash.update(1.0f / 60.0f, car);
    check(picked.size() == 1 && picked[0].chain == 0 && dash.items[0].taken && !dash.items[1].taken,
          "driving through a cell takes it, and only it");
    float peak = 0;
    for (int i = 0; i < 30; ++i) { car.update(1.0f / 60.0f, cruise, area); peak = std::fmax(peak, car.speed); }
    check(near(peak - top, settings.itemKickKmh / 3.6f, 0.2f), "a cell pushes the car past its top speed by the configured amount");
    for (int i = 0; i < 60; ++i) car.update(1.0f / 60.0f, cruise, area);
    check(near(car.speed, top, 0.05f), "the push fades back to the normal top speed within a second");

    // A second cell soon after continues the chain; braking ends a push at once.
    car.x = dash.items[1].position.x - std::sin(car.yaw) * 1.0f;
    car.z = dash.items[1].position.z - std::cos(car.yaw) * 1.0f;
    picked = dash.update(0.1f, car);
    check(picked.size() == 1 && picked[0].chain == 1, "cells taken in quick succession count up a chain");
    car.update(1.0f / 60.0f, cruise, area);
    car.update(1.0f / 60.0f, cruise, area);
    DriveInput brake;
    brake.brake = 1;
    car.update(1.0f / 60.0f, brake, area);
    check(car.itemKickTime < 0 && car.itemKickBonus == 0, "braking ends the push");

    dash.respawn();
    check(!dash.items[0].taken && !dash.items[1].taken, "a new lap brings every cell back");
    car.reset(area);
    check(car.itemKickTime < 0 && car.itemKickBonus == 0, "reset clears a push");

    // Outside a drift a cell leaves the charge alone; inside a held drift it adds a step.
    for (int i = 0; i < 300; ++i) car.update(1.0f / 60.0f, cruise, area);
    auto cellAtCar = [&] {
        DashItems::Item cell;
        cell.position = {car.x, car.y + DashItems::kHover, car.z};
        dash.items = {cell};
        return dash.update(1.0f / 60.0f, car).size();
    };
    const float idleCharge = car.driftCharge;
    check(cellAtCar() == 1 && car.driftCharge == idleCharge, "a cell on a straight only pushes");
    DriveInput slide = cruise;
    slide.handbrake = 1;
    slide.steer = 1;
    for (int i = 0; i < 40 && !(car.driftHeld && car.driftHoldActive); ++i) car.update(1.0f / 60.0f, slide, area);
    check(car.driftHeld && car.driftHoldActive, "holding the drift button with steer starts a held drift");
    const float heldCharge = car.driftCharge;
    check(cellAtCar() == 1 && near(car.driftCharge - heldCharge, settings.itemChargeSeconds, 0.0001f),
          "a cell taken in a held drift adds its step of charge");
    car.driftCharge = 2.1f;
    cellAtCar();
    check(near(car.driftCharge, 2.2f, 0.0001f) && car.driftChargeLevel == 3, "item charge stops at the top level");
}
// Movement speed covers more ground at the same speed readout; turning ease
// turns the same steering into a tighter turn. Both start from a car at speed.
void testTravelScales() {
    Track area;
    area.makePracticeArea();
    auto atSpeed = [&](FeelSettings settings) {
        Car car;
        car.reset(area);
        car.applyTuning(settings);
        DriveInput cruise;
        cruise.throttle = 1;
        for (int i = 0; i < 600; ++i) car.update(1.0f / 60.0f, cruise, area);
        return car;
    };
    auto travel = [&](Car car) {
        DriveInput cruise;
        cruise.throttle = 1;
        const float x = car.x, z = car.z;
        for (int i = 0; i < 60; ++i) car.update(1.0f / 60.0f, cruise, area);
        return std::sqrt((car.x - x) * (car.x - x) + (car.z - z) * (car.z - z));
    };
    auto turn = [&](Car car) {
        DriveInput steer;
        steer.throttle = 1;
        steer.steer = 1;
        const float yaw = car.yaw;
        for (int i = 0; i < 30; ++i) car.update(1.0f / 60.0f, steer, area);
        return std::fabs(wrapPi(car.yaw - yaw));
    };
    FeelSettings plain;
    FeelSettings fast = plain;
    fast.moveSpeedScale = 1.45f;
    const Car slowCar = atSpeed(plain), fastCar = atSpeed(fast);
    check(near(fastCar.speed, slowCar.speed, 0.01f), "movement speed leaves the speed and its readout alone");
    check(near(travel(fastCar) / travel(slowCar), 1.45f, 0.01f), "movement speed covers that much more ground");
    check(near(turn(fastCar), turn(slowCar), 0.001f), "movement speed alone turns at the same rate, so arcs widen");
    FeelSettings easy = fast;
    easy.turnRateScale = 1.15f;
    const Car easyCar = atSpeed(easy);
    check(near(turn(easyCar) / turn(fastCar), 1.15f, 0.02f), "turning ease turns the same steering that much more");
}

void testMoments() {
    Moments moments;
    check(moments.timeScale() == 1 && moments.flash == 0 && moments.edge == 0, "between moments the view is untouched");
    moments.sectorBest();
    check(moments.edge > 0.5f && moments.timeScale() == 1, "a sector best lights the edge without slowing time");
    for (int i = 0; i < 120; ++i) moments.tick(1.0f / 60.0f);
    check(moments.edge < 0.1f, "the edge glow fades within two seconds");
    moments.lap(false);
    check(near(moments.timeScale(), 0.32f, 0.0001f) && moments.flash > 0.5f, "a lap flashes and slows time");
    float t = 0;
    while (moments.timeScale() < 1 && t < 5) { moments.tick(1.0f / 60.0f); t += 1.0f / 60.0f; }
    check(t > 0.7f && t < 0.9f, "a lap's slow motion returns to full speed in about 0.8 seconds");
    moments.lap(true);
    check(near(moments.timeScale(), 0.22f, 0.0001f) && moments.burst > 0.9f, "a record slows time further and adds a burst");
    t = 0;
    while (moments.timeScale() < 1 && t < 5) { moments.tick(1.0f / 60.0f); t += 1.0f / 60.0f; }
    check(t > 1.4f && t < 1.6f, "a record's slow motion lasts about 1.5 seconds");
    for (int i = 0; i < 600; ++i) moments.tick(1.0f / 60.0f);
    check(moments.flash == 0 && moments.edge == 0 && moments.zoom == 0 && moments.burst == 0,
          "every accent settles back to exactly nothing");
    moments.item(0);
    moments.reset();
    check(moments.flash == 0 && moments.zoom == 0 && moments.timeScale() == 1 && moments.goAge < 0, "reset clears every accent");
}
} // namespace

int main() {
    testCountdown();
    testLapAndSectors();
    testNoShortcuts();
    testSuzuka();
    testPickupAndPush();
    testTravelScales();
    testMoments();
    if (failures) return EXIT_FAILURE;
    std::puts("time attack tests passed: countdown, splits, laps, records, no shortcuts, Suzuka splits, drift lines, weave rows, pushes, item charge, movement speed and turning ease");
    return EXIT_SUCCESS;
}
