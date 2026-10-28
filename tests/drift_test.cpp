#include "car.hpp"
#include "tuning.hpp"
#include "trial.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

namespace {

constexpr float kDt = 1.0f / 60.0f;
int failures = 0;
bool near(float a, float b, float tolerance = 0.0001f) { return std::fabs(a-b) < tolerance; }

void check(bool condition, const char* label) {
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", label);
        ++failures;
    }
}

Track straight(float halfWidth = 1000.0f) {
    Track track;
    track.halfWidth = halfWidth;
    track.curbWidth = 0;
    track.runoff = 0;
    track.length = 30000;
    track.samples = {
        {0, -5000, 0, 0, 0}, {0, 5000, 0, 0, 10000},
        {5000, 5000, 0, 0, 15000}, {5000, -5000, 0, 0, 25000},
    };
    return track;
}

Car moving(const Track& track, float speed = 24.0f, DriftModel model = DriftModel::Inertia) {
    Car car;
    car.driftModel = model;
    car.reset(track);
    car.speed = speed;
    return car;
}

void run(Car& car, const Track& track, DriveInput input, int frames) {
    for (int i = 0; i < frames; ++i) car.update(kDt, input, track);
}

Car slide(const Track& track, float side, DriftModel model = DriftModel::Inertia) {
    Car car = moving(track, 80.0f / 3.6f, model);
    DriveInput input;
    input.throttle = 1;
    input.steer = side;
    input.handbrake = 1;
    run(car, track, input, 180);
    return car;
}

float heldSpeedAfter(float entrySpeed, float seconds) {
    const float minimum = std::fmin(entrySpeed, std::fmax(entrySpeed * 0.90f, 11.05f));
    return std::fmax(entrySpeed - seconds, minimum);
}

float tirePathSeparation(float slip) {
    float offsets[4];
    int wheel = 0;
    for (float axle : {1.42f, -1.38f}) for (float side : {-0.82f, 0.82f})
        offsets[wheel++] = side * std::cos(slip) + axle * std::sin(slip);
    float separation = 100;
    for (int a = 0; a < 4; ++a) for (int b = a + 1; b < 4; ++b)
        separation = std::fmin(separation, std::fabs(offsets[a] - offsets[b]));
    return separation;
}

void testGrip(const Track& track) {
    Car car = moving(track);
    DriveInput input;
    input.throttle = 1;
    run(car, track, input, 120);
    check(std::fabs(car.x) < 0.001f && car.z > 50, "straight throttle preserves direction");
    input.steer = 0.7f;
    run(car, track, input, 90);
    check(car.x > 15 && car.yawRate > 0, "ordinary steering still turns right");
    check(car.slipAngle == 0 && car.driftIntensity == 0 && car.driftBlend == 0,
          "ordinary steering never silently initiates drift");

    Car slow = moving(track, 4);
    input.handbrake = 1;
    input.throttle = 0;
    run(slow, track, input, 30);
    check(slow.driftBlend == 0 && slow.driftIntensity == 0, "handbrake at walking speed cannot farm drift");
}

void testSustainAndRecover(const Track& track) {
    constexpr float pi = 3.14159265f;
    Car entry = moving(track, 80.0f / 3.6f);
    DriveInput start;
    start.throttle = start.steer = start.handbrake = 1;
    int entryFrame = 0;
    for (int frame = 1; frame <= 9; ++frame) {
        entry.update(kDt, start, track);
        if (!entryFrame && entry.slipAngle > pi / 180) entryFrame = frame;
    }
    std::printf("entry: first 1-degree slip %.0fms, .15s body %.1f / travel %.1f / slip %.1f deg\n",
                entryFrame * kDt * 1000, entry.yaw * 180 / pi,
                entry.travelYaw * 180 / pi, entry.slipAngle * 180 / pi);
    check(entryFrame > 0 && entryFrame <= 6, "held drift button gives visible response within 100 ms");
    check(entry.slipAngle > 20 * pi / 180 && entry.travelYaw > 0 && entry.yaw > entry.slipAngle,
          "entry builds visible sideways attitude while steering starts bending the actual line");

    Car right = slide(track, 1);
    Car left = slide(track, -1);
    std::printf("held inertial drift: %.1f deg at %.1f km/h\n", right.slipAngle * 180 / pi, right.speed * 3.6f);
    check(right.slipAngle > 45 * pi / 180 && right.slipAngle <= 60.01f * pi / 180,
          "holding drift keeps the sideways posture without auto-aligning or spinning");
    check(right.driftIntensity > 0.9f && std::fabs(right.speed - heldSpeedAfter(80.0f / 3.6f, 3)) < 0.003f,
          "inertial slide slows gently to its bounded retained speed and reports actual lateral motion");
    check(std::fabs(right.slipAngle + left.slipAngle) < 0.002f &&
          std::fabs(right.x + left.x) < 0.002f && std::fabs(right.z - left.z) < 0.002f,
          "left and right drift are symmetric");

    Car counter = right;
    Car neutral = right;
    Car inside = right;
    DriveInput unwind;
    unwind.throttle = unwind.handbrake = 1;
    unwind.steer = -1;
    run(counter, track, unwind, 45);
    unwind.steer = 0;
    run(neutral, track, unwind, 45);
    unwind.steer = 1;
    run(inside, track, unwind, 45);
    std::printf("held steering after .75s: counter %.1f / neutral %.1f / inside %.1f deg\n",
                counter.slipAngle * 180 / pi, neutral.slipAngle * 180 / pi, inside.slipAngle * 180 / pi);
    check(counter.slipAngle < neutral.slipAngle - 0.2f,
          "countersteer can turn the body back from the sideways guard");
    check(std::fabs(neutral.slipAngle - right.slipAngle) < 3 * pi / 180 && neutral.driftHeld,
          "neutral steering does not restore grip while Space is held");
    check(wrapPi(counter.travelYaw-right.travelYaw)<-.5f &&
          wrapPi(inside.travelYaw-right.travelYaw)>.9f &&
          std::fabs(wrapPi(neutral.travelYaw-right.travelYaw))<.2f &&
          std::hypot(counter.x-inside.x,counter.z-inside.z)>3,
          "counter, neutral and inside steering produce distinct controllable lines in the inertial model");

    DriveInput coast;
    run(right, track, coast, 120);
    check(std::fabs(right.slipAngle) < 0.01f && right.driftIntensity < 0.02f,
          "manual release returns completely to grip");
    run(right, track, coast, 90);
    check(right.driftBlend == 0 && right.slipAngle == 0,
          "manual release leaves no residual drift state");
    check(std::fabs(left.speed - heldSpeedAfter(80.0f / 3.6f, 3)) < 0.003f &&
          std::fabs(right.driftEntrySpeed - 80.0f / 3.6f) < 0.001f,
          "holding drift keeps the captured entry reference while actual speed decays gently");
}

void testArcadePhases(const Track& track) {
    constexpr float pi = 3.14159265f;
    DriveInput input;
    input.throttle = input.handbrake = 1;
    for (float side : {-1.0f, 1.0f}) {
        Car car = moving(track, 80.0f / 3.6f);
        input.steer = side;
        run(car, track, input, 15);
        const float firstBody = side * car.yaw;
        std::printf("inertial entry %+.0f at .25s: body %.1f / travel %.1f / slip %.1f deg\n",
                    side, firstBody * 180 / pi, car.travelYaw * 180 / pi, side * car.slipAngle * 180 / pi);
        check(firstBody > 30 * pi / 180 && firstBody < 75 * pi / 180 && side*car.travelYaw>0,
              "entry combines a sideways body swing with a progressively steered incoming line");
        for (int i = 0; i < 585; ++i) {
            car.update(kDt, input, track);
            check(std::fabs(car.slipAngle) <= 60.01f * pi / 180 && std::fabs(car.driftTravelRate)<=1.8001f,
                  "ten-second held steering keeps the sideways angle bounded while turning the line continuously");
        }
        check(std::fabs(car.slipAngle) > 45 * pi / 180 && car.driftHeld && car.driftHoldActive,
              "long holding leaves the car sideways until the player releases Space");
        check(std::fabs(car.x)>1 && near(car.driftTravelRate,side*1.8f,.0001f) &&
              std::fabs(car.speed - 20.0f) < 0.003f,
              "ten-second inertial steering follows a curved line with the same bounded gentle speed loss");
        const float age = car.driftAge;
        const float kick = std::fabs(car.driftKick);
        run(car, track, input, 60);
        check(car.driftAge >= age && std::fabs(car.driftKick) <= kick && kick < 0.01f,
              "a continuous hold does not rearm the entry kick");

        Car tap = moving(track, 80.0f / 3.6f);
        run(tap, track, input, 15);
        input.steer = 0;
        run(tap, track, input, 120);
        const float heldSlip = std::fabs(tap.slipAngle);
        const float heldYaw = tap.travelYaw;
        check(heldSlip > 35 * pi / 180 && tap.driftHeld && side*heldYaw>.1f && std::fabs(tap.driftTravelRate)<.00001f,
              "neutral steering eases curvature to zero while keeping the manually held sideways posture intact");
        input.handbrake = 0;
        tap.update(kDt, input, track);
        check(side * wrapPi(tap.travelYaw - heldYaw) > 0 &&
              std::fabs(tap.slipAngle) > heldSlip * 0.69f && std::fabs(tap.slipAngle) < heldSlip * 0.72f,
              "Space release starts a short turn toward the nose while retaining incoming momentum");
        check(tap.boostSerial == 1 && tap.boostRemaining > 0,
              "releasing a held sideways slide awards its one charged boost");
        input.handbrake = 1;
    }

    // Rotating the incoming line or changing throttle/steer must not reveal an
    // implicit forward-direction preference in the held slide.
    for (float heading : {-0.65f, 0.45f}) {
        Car initial = moving(track, 80.0f / 3.6f);
        initial.yaw = initial.travelYaw = heading;
        Car examples[3] = {initial, initial, initial};
        const float throttles[] = {0, 0.3f, 1};
        const float steering[] = {0.4f, 0.4f, 0.4f};
        for (int variant = 0; variant < 3; ++variant) {
            input.throttle = throttles[variant];
            input.steer = steering[variant];
            run(examples[variant], track, input, 180);
            check(wrapPi(examples[variant].travelYaw-heading)>1.5f && near(examples[variant].driftTravelRate,.72f,.0001f) &&
                  std::fabs(examples[variant].speed - heldSpeedAfter(initial.speed, 3)) < 0.003f,
                  "an arbitrary-heading drift bends according to steering independently of throttle with unchanged mild slowdown");
            const float along = examples[variant].x * std::sin(heading) + examples[variant].z * std::cos(heading);
            const float across = examples[variant].x * std::cos(heading) - examples[variant].z * std::sin(heading);
            check(along>10 && across>10 && std::hypot(along,across)>40 && std::hypot(along,across)<60,
                  "arbitrary-heading drift integrates a steered arc rather than a straight body-posture animation");
        }
        check(examples[0].x == examples[1].x && examples[0].x == examples[2].x &&
              examples[0].z == examples[1].z && examples[0].z == examples[2].z &&
              examples[0].yaw == examples[1].yaw,
              "changing throttle alone leaves the same held steering trajectory and sideways attitude");
    }

    // Entry is explicit: an immediate countersteer must still establish a slide,
    // and only releasing Space should choose when to leave that state.
    Car interrupted = moving(track, 80.0f / 3.6f);
    input.throttle = input.handbrake = input.steer = 1;
    run(interrupted, track, input, 3);
    Car uninterrupted = interrupted;
    run(uninterrupted, track, input, 27);
    input.steer = -1;
    interrupted.update(kDt, input, track);
    check(interrupted.driftPhase == DriftPhase::Entry && interrupted.driftKick > 0,
          "early countersteer cannot leave an active entry facing forward indefinitely");
    run(interrupted, track, input, 26);
    check(interrupted.driftPhase == DriftPhase::Held && interrupted.slipAngle >= 45 * pi / 180 &&
          wrapPi(interrupted.travelYaw-uninterrupted.travelYaw)<-.5f,
          "early countersteer bends the actual line while still establishing the selected-side slide");
    const float interruptedAge = interrupted.driftAge;
    input.steer = 1;
    run(interrupted, track, input, 30);
    check(interrupted.driftKick == 0 && interrupted.driftAge > interruptedAge && interrupted.driftPhase == DriftPhase::Held,
          "held angle adjustments cannot restart the entry phase");

}

void testLineControl(const Track& track) {
    constexpr float pi = 3.14159265f;
    constexpr float entrySpeed = 80.0f / 3.6f;
    Car right = moving(track, entrySpeed, DriftModel::LineControl);
    Car left = right;
    Car inertial = moving(track, entrySpeed);
    DriveInput input;
    input.throttle = input.handbrake = input.steer = 1;
    for (int i = 0; i < 60; ++i) {
        const float oldX = right.x, oldZ = right.z, oldHeading = right.travelYaw;
        right.update(kDt, input, track);
        inertial.update(kDt, input, track);
        DriveInput opposite = input;
        opposite.steer = -1;
        left.update(kDt, opposite, track);
        const float dx = right.x - oldX, dz = right.z - oldZ;
        check(std::fabs(right.speed - heldSpeedAfter(entrySpeed, (i + 1) * kDt)) < 0.003f &&
              std::fabs(std::sqrt(dx * dx + dz * dz) / kDt - right.speed) < 0.003f,
              "line control integrates exactly its gently decreasing speed while steering the velocity");
        check(std::fabs(wrapPi(right.travelYaw - oldHeading)) < 3 * pi / 180,
              "held line steering changes heading continuously rather than snapping position or direction");
    }
    std::printf("model comparison at 1s: inertia heading %.1f / line heading %.1f deg, line %.1f km/h\n",
                inertial.travelYaw * 180 / pi, right.travelYaw * 180 / pi, right.speed * 3.6f);
    check(inertial.travelYaw>.6f && right.travelYaw>inertial.travelYaw+.02f && right.x>5 &&
          right.z<inertial.z-.1f,
          "both held models bend the actual line while the quicker model responds ahead of the inertial model");
    check(std::fabs(right.travelYaw + left.travelYaw) < 0.001f &&
          std::fabs(right.x + left.x) < 0.002f && std::fabs(right.z - left.z) < 0.002f,
          "left and right line control follow mirrored trajectories");

    // After the wheels centre, an angled body alone cannot pull the line toward its nose.
    input.steer = 0;
    run(right, track, input, 120);
    const float neutralHeading = right.travelYaw;
    const float neutralSlip = right.slipAngle;
    run(right, track, input, 60);
    check(std::fabs(wrapPi(right.travelYaw-neutralHeading))<.0001f && std::fabs(neutralSlip) > 20 * pi / 180 &&
          std::fabs(right.slipAngle - neutralSlip) < 0.01f,
          "neutral line control holds its current trajectory and sideways attitude without automatic nose capture");

    Car counter = moving(track, entrySpeed, DriftModel::LineControl);
    input.steer = 1;
    run(counter, track, input, 45);
    const float beforeCounter = counter.travelYaw;
    input.steer = -1;
    run(counter, track, input, 30);
    check(wrapPi(counter.travelYaw - beforeCounter) < -0.2f && counter.driftHeld,
          "countersteer opens and reverses the actual line while Space remains held");

    Car longHold = moving(track, entrySpeed, DriftModel::LineControl);
    input.steer = 1;
    float finalTurn = 0;
    for (int i = 0; i < 600; ++i) {
        const float before = longHold.travelYaw;
        longHold.update(kDt, input, track);
        if (i >= 540) finalTurn += wrapPi(longHold.travelYaw - before);
        check(std::fabs(longHold.slipAngle) <= 60.01f * pi / 180 &&
              std::fabs(longHold.speed - heldSpeedAfter(entrySpeed, (i + 1) * kDt)) < 0.003f,
              "long line-control holds retain bounded speed loss and body-to-path angle");
    }
    check(finalTurn > 0.5f && longHold.driftHoldActive,
          "the body-angle guard does not stop a continuously steered line from turning");

    // Configuration survives reset, while an ordinary reset still clears the slide.
    longHold.reset(track);
    check(longHold.driftModel == DriftModel::LineControl && !longHold.drifting &&
          longHold.speed == 0 && longHold.driftChargeLevel == 0,
          "reset preserves LineControl selection while clearing motion and charge");
}

void testFourTraceHold(const Track& track, DriftModel model) {
    constexpr float pi = 3.14159265f;
    for (float side : {-1.0f, 1.0f}) {
        Car car = moving(track, 80.0f / 3.6f, model);
        DriveInput hold;
        hold.throttle = hold.handbrake = 1;
        hold.steer = side;
        float minimumSeparation = 100;
        for (int frame = 0; frame < 600; ++frame) {
            car.update(kDt, hold, track);
            check(std::fabs(car.slipAngle) <= 60.01f * pi / 180,
                  "entry and continued steering stay inside the outward-only 60-degree guard");
            if (frame < 180) continue;
            // Project the actual four wheel locations onto the line's lateral axis.
            // The tires remain at their physical positions; no trace offset is added.
            float offsets[4];
            int wheel = 0;
            for (float axle : {1.42f, -1.38f})
                for (float tireSide : {-0.82f, 0.82f})
                    offsets[wheel++] = tireSide * std::cos(car.slipAngle) + axle * std::sin(car.slipAngle);
            for (int a = 0; a < 4; ++a) for (int b = a + 1; b < 4; ++b)
                minimumSeparation = std::fmin(minimumSeparation, std::fabs(offsets[a] - offsets[b]));
        }
        check(std::fabs(car.slipAngle) > 45 * pi / 180 && minimumSeparation >= 0.815f &&
              std::fabs(car.speed - heldSpeedAfter(80.0f / 3.6f, 10)) < 0.003f,
              "steady held sliding keeps four physically separate wheel traces with only bounded gentle speed loss");
        std::printf("four traces model%d side%+.0f: held %.2fdeg, minimum separation %.3fm\n",
                    int(model), side, std::fabs(car.slipAngle) * 180 / pi, minimumSeparation);
        hold.steer = 0;
        run(car, track, hold, 120);
        const float neutralSlip = car.slipAngle;
        const float neutralHeading = car.travelYaw;
        run(car, track, hold, 120);
        check(car.driftHoldActive && car.driftHeld && std::fabs(car.slipAngle - neutralSlip) < 0.001f &&
              std::fabs(wrapPi(car.travelYaw - neutralHeading)) < 0.001f,
              "neutral steering leaves the line and sideways posture under manual Space control");
        hold.steer = -side;
        const float beforeCounter = car.travelYaw;
        run(car, track, hold, 45);
        check(car.driftHeld && car.driftHoldActive && car.driftKick == 0,
              "countersteering cannot force an automatic exit or rearm the entry kick");
        check(std::fabs(car.slipAngle) < std::fabs(neutralSlip) - 0.15f &&
              std::fabs(car.slipAngle) >= 44.99f * pi / 180,
              "countersteer changes real body-to-path angle in either model without crossing its chosen-side floor");
        check(side * wrapPi(car.travelYaw - beforeCounter) < -0.2f,
              "countersteer bends the actual line in either model while the player keeps holding Space");
    }
}

void testStatefulHold(const Track& track, DriftModel model) {
    constexpr float pi = 3.14159265f;
    int earliestEntry = 1000, latestEntry = 0;
    float minimumSeparation = 100;
    for (float speed : {11.02f, 12.0f, 80.0f / 3.6f, 100.0f / 3.6f, 40.0f, 160.0f / 3.6f})
        for (float heading : {-2.7f, 0.4f, 2.7f}) for (float side : {-1.0f, 1.0f}) {
            Car car = moving(track, speed, model);
            if (speed>100/3.6f) car.topSpeedKmh=160;
            car.yaw = car.travelYaw = heading;
            DriveInput input;
            input.throttle = input.handbrake = 1;
            input.steer = side;
            int frames = 0;
            bool continuous = true, bounded = true, speedContract = true;
            auto tick = [&] {
                const float oldBody = car.yaw, oldSpeed = car.speed;
                car.update(kDt, input, track);
                ++frames;
                continuous = continuous && std::fabs(wrapPi(car.yaw - oldBody)) <= 7.81f * kDt &&
                             std::fabs(car.slipAngle) <= 60.01f * pi / 180;
                speedContract = speedContract && car.speed <= oldSpeed + 0.00001f &&
                                std::fabs(car.speed - heldSpeedAfter(speed, frames * kDt)) < 0.005f;
                if (car.driftPhase == DriftPhase::Held) {
                    const float signedSlip = side * car.slipAngle;
                    bounded = bounded && car.driftHoldActive && car.driftDirection == side &&
                              signedSlip >= 44.99f * pi / 180 && signedSlip <= 60.01f * pi / 180 &&
                              car.driftAngleCommand >= 45 * pi / 180 && car.driftAngleCommand <= 60 * pi / 180;
                    minimumSeparation = std::fmin(minimumSeparation, tirePathSeparation(car.slipAngle));
                }
            };
            tick();
            check(car.driftPhase == DriftPhase::Entry && std::fabs(car.slipAngle) < 8 * pi / 180,
                  "entry starts continuously near zero instead of teleporting the body to the held-angle floor");
            input.steer = -side;
            while (car.driftPhase != DriftPhase::Held && frames < 60) tick();
            check(car.driftPhase == DriftPhase::Held && frames <= 45,
                  "immediate countersteer still establishes the selected-side held slide within750ms");
            earliestEntry = std::min(earliestEntry, frames);
            latestEntry = std::max(latestEntry, frames);
            for (int i = 0; i < 60; ++i) tick();
            const float lowAngle = std::fabs(car.slipAngle);
            input.steer = side;
            for (int i = 0; i < 10; ++i) tick();
            const float middleCommand = car.driftAngleCommand;
            input.steer = 0;
            for (int i = 0; i < 90; ++i) tick();
            check(std::fabs(car.driftAngleCommand - middleCommand) < 0.00001f &&
                  middleCommand > 50 * pi / 180 && middleCommand < 55 * pi / 180 &&
                  std::fabs(std::fabs(car.slipAngle) - middleCommand) < 0.3f * pi / 180,
                  "neutral steering remembers a player-chosen middle angle instead of returning to one fixed pose");
            input.steer = side;
            for (int i = 0; i < 90; ++i) tick();
            check(std::fabs(car.slipAngle) > lowAngle + 10 * pi / 180,
                  "inside steering increases the real body-to-travel angle in either model");
            input.steer = -side;
            float lineChange = 0;
            for (int i = 0; i < 90; ++i) {
                const float oldTravel = car.travelYaw;
                tick();
                lineChange += wrapPi(car.travelYaw - oldTravel);
            }
            check(std::fabs(car.slipAngle) < 45.3f * pi / 180,
                  "sustained countersteer lowers the real slip toward45 degrees without crossing forward");
            check(side * lineChange < -0.2f,
                  "angle control preserves meaningful input-controlled path turning in both held models");
            check(continuous && bounded && speedContract,
                  "both directions, wrapped headings and all practical speeds keep continuous signed45..60 hold and bounded mild slowdown");
        }
    check(minimumSeparation >= 0.815f,
          "every sampled held angle keeps all four physical tire paths at least81cm apart");
    std::printf("stateful hold model%d: entry %.3f..%.3fs, 4-tire separation >=%.3fm\n",
                int(model), earliestEntry * kDt, latestEntry * kDt, minimumSeparation);
}

void testGentleHoldSlowdown(const Track& track, DriftModel model) {
    const float entry = 80.0f / 3.6f;
    Car car = moving(track, entry, model);
    DriveInput input;
    input.throttle = input.handbrake = input.steer = 1;
    run(car, track, input, 30);
    check(std::fabs(car.speed - (entry - 0.5f)) < 0.003f,
          "first half second of drift loses only half a metre per second");
    run(car, track, input, 30);
    check(std::fabs(car.speed - (entry - 1)) < 0.003f,
          "one second of held drift gently loses3.6km/h even with W held");
    run(car, track, input, 120);
    check(std::fabs(car.speed - entry * 0.9f) < 0.003f,
          "uninterrupted drift speed loss stops at10 percent of captured ordinary entry speed");
    run(car, track, input, 600);
    check(car.driftHeld && car.driftHoldActive && car.driftPhase == DriftPhase::Held &&
          std::fabs(car.speed - entry * 0.9f) < 0.003f,
          "long holding never stalls or automatically releases just because time passes");
    car.speed = 15;
    car.update(kDt, input, track);
    check(car.speed == 15 && car.driftEntrySpeed == entry,
          "a speed below the retention floor is never raised back to that floor");
    Car nearThreshold = moving(track, 11.02f, model);
    run(nearThreshold, track, input, 1200);
    check(nearThreshold.driftPhase == DriftPhase::Held && nearThreshold.driftHoldActive &&
          nearThreshold.speed <= 11.02f && nearThreshold.speed > 11,
          "a valid entry just above minimum speed cannot time out through its own gentle slowdown");
}

void testEasedRelease(const Track& track, DriftModel model) {
    constexpr float pi = 3.14159265f;
    DriveInput release;
    release.throttle = 1;
    for (float heading : {-2.6f, 0.45f, 2.6f}) for (float side : {-1.0f, 1.0f})
        for (int holdFrames : {15, 180}) {
            Car car = moving(track, 80.0f / 3.6f, model);
            car.yaw = car.travelYaw = heading;
            DriveInput hold = release;
            hold.steer = side;
            hold.handbrake = 1;
            run(car, track, hold, holdFrames);
            check(car.driftHoldActive && (holdFrames == 15 ? car.driftChargeLevel == 0 : car.driftChargeLevel == 3),
                  "eased release fixtures cover both uncharged taps and fully charged holds");
            const float oldTravel = car.travelYaw, oldBody = car.yaw, oldSlip = car.slipAngle;
            const float oldX = car.x, oldZ = car.z;
            Car halfway = car;
            halfway.update(0.075f, release, track);
            check(halfway.releaseEaseActive && std::fabs(halfway.slipAngle / oldSlip - 0.125f) < 0.0001f,
                  "halfway through the short release only one eighth of the captured sideways angle remains");
            Car ordinary = moving(track, car.speed, model);
            ordinary.x = car.x; ordinary.y = car.y; ordinary.z = car.z;
            ordinary.yaw = ordinary.travelYaw = oldBody;
            ordinary.steerAngle = car.steerAngle;
            ordinary.throttleLevel = car.throttleLevel;
            ordinary.ground = car.ground;
            ordinary.update(kDt, release, track);
            car.update(kDt, release, track);
            check(car.releaseEaseActive && car.drifting && car.driftPhase == DriftPhase::None && car.driftAngleCommand == 0 &&
                  car.driftTravelRate == 0 && car.driftKick == 0 &&
                  std::fabs(car.slipAngle / oldSlip - 0.702332f) < 0.0001f && car.yawRate == ordinary.yawRate,
                  "first60Hz release step retains about70 percent of the old slip with ordinary body steering");
            check(car.yaw == ordinary.yaw && std::fabs(wrapPi(car.yaw - oldBody)) < 2 * pi / 180 &&
                  std::fabs(wrapPi(car.travelYaw - oldTravel)) < std::fabs(oldSlip) * 0.4f + 2 * pi / 180 &&
                  std::fabs(wrapPi(car.travelYaw - car.yaw)) > std::fabs(oldSlip) * 0.69f,
                  "release starts between incoming direction and current nose without a body unwind or fixed-yaw snap");
            const float dx = car.x - oldX, dz = car.z - oldZ;
            check(std::fabs(std::sqrt(dx * dx + dz * dz) - ordinary.speed * kDt) < 0.0001f &&
                  std::fabs(wrapPi(std::atan2(dx, dz) - car.travelYaw)) < 0.0001f,
                  "release integrates exactly one normal-speed movement along the partially blended direction");
            check(std::fabs(car.speed - ordinary.speed) < 0.0001f &&
                  car.boostSerial == unsigned(holdFrames == 180),
                  "the release step starts its boost attack at zero while charge still awards exactly one event");
            Car replay = car;
            for (int paused = 0; paused < 30; ++paused) replay.update(0, release, track);
            check(replay.releaseEaseElapsed == car.releaseEaseElapsed && replay.x == car.x &&
                  replay.travelYaw == car.travelYaw && replay.boostRemaining == car.boostRemaining,
                  "no simulation time during pause advances neither release easing nor boost");
            float previousSlip = std::fabs(car.slipAngle);
            const float firstRemoval = std::fabs(oldSlip) - previousSlip;
            float lastRemoval = 0;
            for (int frame = 2; frame <= 9; ++frame) {
                const float previousX = car.x, previousZ = car.z, previousBody = car.yaw;
                car.update(kDt, release, track);
                replay.update(kDt, release, track);
                const float remainingSlip = std::fabs(car.slipAngle);
                check(remainingSlip < previousSlip && side * wrapPi(car.yaw - previousBody) >= -0.0001f,
                      "old slip decays on every release step while the body follows its ordinary steering");
                check(car.x == replay.x && car.z == replay.z && car.travelYaw == replay.travelYaw &&
                      car.releaseEaseElapsed == replay.releaseEaseElapsed && car.boostRemaining == replay.boostRemaining,
                      "replaying the same fixed steps after a pause gives the same release path and pulse");
                const float stepX = car.x - previousX, stepZ = car.z - previousZ;
                check(std::fabs(wrapPi(std::atan2(stepX, stepZ) - car.travelYaw)) < 0.0001f,
                      "every easing step moves along its current blended travel direction");
                if (frame == 3)
                    check(remainingSlip / std::fabs(oldSlip) > 0.29f && remainingSlip / std::fabs(oldSlip) < 0.30f,
                          "after50ms most of the old slip is already removed");
                if (frame == 6)
                    check(remainingSlip / std::fabs(oldSlip) < 0.04f,
                          "after100ms only a small tail of incoming momentum remains");
                lastRemoval = previousSlip - remainingSlip;
                previousSlip = remainingSlip;
            }
            check(!car.releaseEaseActive && !car.drifting && car.slipAngle == 0 && car.travelYaw == car.yaw &&
                  car.releaseEaseElapsed == 0 && car.releaseEaseSlip == 0 && lastRemoval < firstRemoval * 0.01f,
                  "at150ms release joins ordinary grip exactly with a tiny final angular step");
            run(car, track, release, 90);
            check(!car.drifting && car.slipAngle == 0 && car.boostRemaining == 0,
                  "completed easing stays in ordinary grip and cannot repeatedly trigger its reward");
        }
    Car steerLeft = slide(track, 1, model);
    Car steerRight = steerLeft;
    release.steer = -1;
    run(steerLeft, track, release, 30);
    release.steer = 1;
    run(steerRight, track, release, 30);
    check(wrapPi(steerRight.travelYaw - steerLeft.travelYaw) > 0.5f,
          "ordinary steering still controls the line during and after the short eased exit");
}

void testFallbackRecovery(const Track& track, DriftModel model) {
    constexpr float pi = 3.14159265f;
    const Car entry = slide(track, 1, model);
    float previousRecovery = 0;
    for (float duration : {0.6f, 0.8f, 1.0f}) {
        Car recovering = entry;
        recovering.recoveryTime = duration;
        recovering.yaw = wrapPi(recovering.travelYaw + 30 * pi / 180);
        recovering.slipAngle = 30 * pi / 180;
        recovering.yawRate = recovering.steerAngle = recovering.driftKick = 0;
        recovering.handbrakeHeld = recovering.driftHeld = recovering.driftHoldActive = false;
        recovering.driftPhase = DriftPhase::None;
        recovering.driftAngleCommand = 0;
        recovering.driftHoldBlocked = true;
        recovering.driftCharge = recovering.driftChargeLevel = 0;
        const float threshold = recovering.slipAngle * 0.1f;
        DriveInput input;
        input.throttle = 1;
        int frames = 0;
        do {
            const float oldTravel = recovering.travelYaw, oldBody = recovering.yaw;
            recovering.update(kDt, input, track);
            check(std::fabs(wrapPi(recovering.travelYaw - oldTravel)) < 3 * pi / 180 &&
                  std::fabs(wrapPi(recovering.yaw - oldBody)) < 3 * pi / 180,
                  "uncommanded fallback recovery remains smooth rather than using the manual direction switch");
            ++frames;
        } while (std::fabs(recovering.slipAngle) > threshold && frames < 180);
        const float actual = frames * kDt;
        std::printf("fallback model%d: setting %.1fs, measured90%% %.3fs\n", int(model), duration, actual);
        check(std::fabs(actual - duration) < 0.14f && actual > previousRecovery + 0.1f,
              "fallback .6/.8/1.0 settings retain distinct gradual recovery responses");
        check(recovering.boostSerial == 0, "fallback recovery cannot award a manual release boost");
        previousRecovery = actual;
    }
    Car braking = entry;
    const float oldTravel = braking.travelYaw;
    DriveInput brake;
    brake.brake = 1;
    braking.update(kDt, brake, track);
    check(std::fabs(wrapPi(braking.travelYaw - oldTravel)) < 3 * pi / 180 &&
          braking.drifting && !braking.driftHoldActive && braking.boostSerial == 0,
          "Space release combined with braking cancels reward and uses gradual fallback recovery");

    Car displaced = entry;
    displaced.travelYaw = wrapPi(displaced.yaw - 1.6f);
    displaced.slipAngle = 1.6f;
    displaced.recoveryTime = 0.3f;
    displaced.handbrakeHeld = displaced.driftHeld = displaced.driftHoldActive = false;
    displaced.driftPhase = DriftPhase::None;
    displaced.driftAngleCommand = 0;
    displaced.driftHoldBlocked = true;
    displaced.driftCharge = displaced.driftChargeLevel = 0;
    for (int i = 0; i < 120; ++i) {
        const float oldTravel = displaced.travelYaw, oldBody = displaced.yaw;
        displaced.update(kDt, {}, track);
        check(std::fabs(wrapPi(displaced.travelYaw - oldTravel)) < 3 * pi / 180 &&
              std::fabs(wrapPi(displaced.yaw - oldBody)) < 3 * pi / 180,
              "large post-impact slip still recovers without a one-frame heading jump");
    }
    check(!displaced.drifting && std::fabs(displaced.slipAngle) < 0.01f,
          "bounded fallback recovery from large slip eventually returns to ordinary grip");
}

void testChargeAndBoost(const Track& track, DriftModel model = DriftModel::Inertia) {
    constexpr float thresholds[] = {0, 0.65f, 1.3f, 2.2f};
    constexpr float peaks[] = {0, 10, 14, 18};
    DriveInput hold;
    hold.throttle = 1;
    hold.handbrake = 1;
    hold.steer = 1;
    for (int level = 1; level <= 3; ++level) {
        Car car = moving(track, 80.0f / 3.6f, model);
        float minimumSpeed = car.speed;
        float previousCharge = 0;
        int frames = 0;
        while (car.driftChargeLevel < level && frames < 360) {
            previousCharge = car.driftCharge;
            DriveInput input = hold;
            car.update(kDt, input, track);
            minimumSpeed = std::fmin(minimumSpeed, car.speed);
            ++frames;
        }
        check(car.driftChargeLevel == level && previousCharge < thresholds[level] &&
              car.driftCharge + 0.00001f >= thresholds[level],
              "each charge tier is reached only after enough real sliding time");
        check(minimumSpeed >= (80.0f / 3.6f) * 0.90f - 0.003f &&
              std::fabs(car.speed - heldSpeedAfter(80.0f / 3.6f, frames * kDt)) < 0.003f,
              "charged held drift follows the one-metre-per-second gentle slowdown with at most10 percent loss");
        Car noCharge = car;
        noCharge.driftCharge = 0;
        noCharge.driftChargeLevel = 0;
        DriveInput release;
        release.throttle = 1;
        car.update(kDt, release, track);
        noCharge.update(kDt, release, track);
        check(car.boostLevel == level && near(car.boostRemaining,1.1f) &&
              car.boostSerial == 1 && car.driftCharge == 0 && car.driftChargeLevel == 0,
              "charged release awards exactly one boost and consumes its charge");
        check(car.speed==noCharge.speed && car.boostSpeedBonus==0 && car.boostEnvelopeAmount()==0,
              "release intent is active even at the zero-gain beginning of its finite speed attack");
        check(car.travelYaw == noCharge.travelYaw && car.yaw == noCharge.yaw &&
              car.x == noCharge.x && car.z == noCharge.z && car.releaseEaseActive && car.slipAngle != 0,
              "boost adds speed after the same first easing step without changing position or body aim");
        int boostFrames = 0;
        float previousExcess = 0, extraDistance=0;
        while (car.boostRemaining > 0 && boostFrames < 100) {
            const float previousSpeed = car.speed;
            car.update(kDt, release, track);
            noCharge.update(kDt, release, track);
            ++boostFrames;
            const float excess = car.speed - noCharge.speed;
            extraDistance+=excess*kDt;
            check(excess >= -0.0001f && excess<=peaks[level]+.0002f &&
                  std::fabs(excess - car.boostSpeedBonus) < 0.0001f,
                  "the speed envelope changes only the removable bonus while the underlying drivetrain matches an unboosted run");
            if (boostFrames<=6) check(excess>=previousExcess-.0002f,
                                      "the first100ms build the speed gain monotonically instead of starting with a decaying impulse");
            if (boostFrames>=6 && boostFrames<=18) check(near(excess,peaks[level],.0002f),
                                                        "all charge tiers hold their exact full speed gain from100ms through300ms");
            if (boostFrames>18) check(excess<=previousExcess+.0002f,
                                      "the independent800ms return eases speed gain monotonically back to ordinary driving");
            if (boostFrames==42) check(near(excess,peaks[level]*.5f,.001f),
                                       "speed gain is halfway home at700ms after the attack and full-power hold");
            if (car.boostRemaining == 0) {
                check(std::fabs(car.speed - previousSpeed) < 0.25f && previousExcess < 0.01f,
                      "pulse expiry rejoins ordinary acceleration smoothly without a final speed clamp");
            }
            previousExcess = excess;
        }
        check(near(boostFrames*kDt,1.1f,.025f) && near(extraDistance,peaks[level]*.65f,.002f),
              "each tier uses the common1.1second flow and gains four times the old maximum-tier speed area per unit peak");
        check(std::fabs(car.speed - noCharge.speed) < 0.0001f && car.boostSpeedBonus == 0,
              "expired pulse leaves the normal throttle-driven speed without banked excess");
        run(car, track, release, 120);
        run(noCharge, track, release, 120);
        check(car.boostSerial == 1 && car.boostLevel == 0 && car.boostRemaining == 0 &&
              !car.drifting && car.slipAngle == 0 && !car.driftHeld &&
              std::fabs(car.speed - noCharge.speed) < 0.0001f,
              "release catches to grip with W held and cannot repeatedly award the same charge");
        std::printf("boost%d model%d: attack.10/hold.20/return.80, peak +%.1fkm/h, speed-area +%.2fm\n",
                    level,int(model),peaks[level]*3.6f,extraDistance);
    }

    Car shortHold = moving(track, 80.0f / 3.6f, model);
    run(shortHold, track, hold, 12);
    DriveInput release;
    release.throttle = 1;
    shortHold.update(kDt, release, track);
    check(shortHold.boostSerial == 0 && shortHold.boostRemaining == 0,
          "short uncharged release does not grant a mini boost");

    Car straightCar = moving(track, 24, model);
    DriveInput straightInput = hold;
    straightInput.steer = 0;
    run(straightCar, track, straightInput, 240);
    straightCar.update(kDt, release, track);
    check(straightCar.driftCharge == 0 && straightCar.boostSerial == 0,
          "holding the button on a straight cannot farm charge");
    Car preHeld = moving(track, 24, model);
    run(preHeld, track, straightInput, 30);
    check(!preHeld.drifting && preHeld.driftCharge == 0, "pre-holding drift on a straight creates no fake sliding");
    run(preHeld, track, hold, 30);
    check(preHeld.drifting && preHeld.driftHoldActive && preHeld.slipAngle > 0.1f,
          "a held drift button initiates when steering arrives without requiring another button edge");
    Car stationary = moving(track, 0, model);
    DriveInput stationaryInput = hold;
    stationaryInput.throttle = 0;
    run(stationary, track, stationaryInput, 240);
    check(stationary.speed == 0 && stationary.driftCharge == 0 && !stationary.drifting,
          "stationary steering cannot farm charge or create speed");
    Car reverseCar = moving(track, -8, model);
    run(reverseCar, track, stationaryInput, 120);
    check(reverseCar.driftCharge == 0 && reverseCar.boostSerial == 0 && !reverseCar.drifting,
          "reverse sliding cannot farm charge");
    Car tapping = moving(track, 24, model);
    for (int tap = 0; tap < 12; ++tap) {
        run(tapping, track, hold, 4);
        run(tapping, track, release, 10);
    }
    check(tapping.boostSerial == 0, "repeated short taps cannot accumulate charge across releases");
}

void testCancellation(const Track& track, DriftModel model = DriftModel::Inertia) {
    DriveInput held;
    held.throttle = 1;
    held.handbrake = 1;
    Car brake = slide(track, 1, model);
    check(brake.driftChargeLevel == 3, "cancellation test starts with actual fully charged sliding");
    held.brake = 1;
    brake.update(kDt, held, track);
    check(brake.driftCharge == 0 && brake.driftChargeLevel == 0 && !brake.driftHoldActive && brake.driftKick == 0,
          "brake cancels held charge and speed retention");
    check(brake.driftPhase == DriftPhase::None && brake.driftAngleCommand == 0,
          "brake cancellation clears the held angle state and command");
    held.handbrake = 0;
    held.brake = 0;
    brake.update(kDt, held, track);
    check(brake.boostSerial == 0, "releasing after a brake cancellation gives no boost");

    Car activeBoost = slide(track, 1, model);
    activeBoost.update(kDt, held, track);
    run(activeBoost,track,held,6);
    check(activeBoost.boostRemaining > 0, "active cancellation test starts with a valid boost");
    const float brakeBaseSpeed = activeBoost.speed - activeBoost.boostSpeedBonus;
    held.brake = 1;
    activeBoost.update(kDt, held, track);
    check(activeBoost.boostRemaining == 0 && activeBoost.boostLevel == 0 && activeBoost.boostSerial == 1 &&
          !activeBoost.releaseEaseActive && activeBoost.releaseEaseSlip == 0 && activeBoost.releaseEaseElapsed == 0 &&
          activeBoost.boostSpeedBonus == 0 && activeBoost.speed < brakeBaseSpeed,
          "brake immediately cancels an active boost without creating another event");

    Track grass = straight(6.5f);
    grass.runoff = 8;
    Car offroad = slide(track, 1, model);
    offroad.x = 7;
    offroad.z = 0;
    offroad.travelYaw = 0;
    offroad.yaw = offroad.slipAngle;
    offroad.ground = grass.query(offroad.x, offroad.z, 0);
    held.brake = 0;
    held.handbrake = 1;
    offroad.update(kDt, held, grass);
    check(!offroad.onAsphalt && offroad.impact == 0 && offroad.driftCharge == 0 &&
          offroad.boostRemaining == 0 && !offroad.driftHoldActive,
          "crossing onto grass cancels charge and retention without a wall collision");

    Car grassBoost = slide(track, 1, model);
    held.handbrake = 0;
    grassBoost.update(kDt, held, track);
    run(grassBoost,track,held,6);
    const float grassBaseSpeed=grassBoost.speed-grassBoost.boostSpeedBonus;
    grassBoost.x = 7;
    grassBoost.z = 0;
    grassBoost.travelYaw = 0;
    grassBoost.yaw = grassBoost.slipAngle;
    grassBoost.ground = grass.query(7, 0, 0);
    grassBoost.update(kDt, held, grass);
    check(!grassBoost.onAsphalt && grassBoost.boostRemaining == 0 && grassBoost.boostLevel == 0 &&
          !grassBoost.releaseEaseActive && grassBoost.releaseEaseSlip == 0 && grassBoost.releaseEaseElapsed == 0 &&
          grassBoost.boostSpeedBonus == 0 &&
          grassBoost.speed<=grassBaseSpeed+grassBoost.topSpeedKmh/3.6f/grassBoost.accelerationTime*kDt+.001f &&
          grassBoost.boostSerial == 1, "crossing onto grass cancels an active boost");

    Track wall = straight(8);
    Car releaseAtWall = slide(track, 1, model);
    releaseAtWall.x = 7.99f;
    releaseAtWall.z = 0;
    releaseAtWall.yaw = 0.9f;
    releaseAtWall.travelYaw = -0.1f;
    releaseAtWall.slipAngle = 1;
    releaseAtWall.ground = wall.query(releaseAtWall.x, 0, 0);
    releaseAtWall.update(kDt, held, wall);
    check(releaseAtWall.impact > 0.05f && releaseAtWall.boostSerial == 0 &&
          releaseAtWall.boostRemaining == 0 && !releaseAtWall.releaseEaseActive,
          "release collides when its partially blended direction points into the wall and cancels the award");

    Car releaseAway = slide(track, 1, model);
    releaseAway.x = 7.9f;
    releaseAway.z = 0;
    releaseAway.yaw = -0.4f;
    releaseAway.travelYaw = 0.6f;
    releaseAway.slipAngle = -1;
    releaseAway.ground = wall.query(releaseAway.x, 0, 0);
    releaseAway.update(kDt, held, wall);
    check(releaseAway.impact > 0.05f && releaseAway.boostSerial == 0 && releaseAway.boostRemaining == 0 &&
          !releaseAway.releaseEaseActive && releaseAway.drifting,
          "a nose aimed away can still hit the wall because the first easing step retains real outward inertia");
    const float retainedImpactSpeed = releaseAway.speed;
    releaseAway.update(kDt, held, wall);
    check(releaseAway.speed <= retainedImpactSpeed + releaseAway.topSpeedKmh/3.6f/releaseAway.accelerationTime*kDt+.001f &&
          releaseAway.boostSerial == 0 && !releaseAway.releaseEaseActive,
          "cancelled release easing cannot restore momentum lost at the wall on the next step");

    Car wallBoost = slide(track, 1, model);
    wallBoost.update(kDt, held, track);
    run(wallBoost,track,held,6);
    wallBoost.x = 7.9f;
    wallBoost.z = 0;
    wallBoost.yaw = wallBoost.travelYaw = 0.6f;
    wallBoost.slipAngle = wallBoost.steerAngle = 0;
    wallBoost.drifting = false;
    wallBoost.releaseEaseActive = false;
    wallBoost.releaseEaseSlip = wallBoost.releaseEaseElapsed = 0;
    wallBoost.ground = wall.query(wallBoost.x, 0, 0);
    Car ordinaryWall = wallBoost;
    ordinaryWall.speed -= ordinaryWall.boostSpeedBonus;
    ordinaryWall.boostSpeedBonus = ordinaryWall.boostRemaining = 0;
    ordinaryWall.boostLevel = 0;
    wallBoost.update(kDt, held, wall);
    ordinaryWall.update(kDt, held, wall);
    check(wallBoost.impact > ordinaryWall.impact && wallBoost.boostSpeedBonus == 0 &&
          wallBoost.boostRemaining == 0 && std::fabs(wallBoost.speed - ordinaryWall.speed) < 0.0001f,
          "boosted wall contact removes the pulse and keeps exactly the unboosted tangential speed");
    const float afterWall = wallBoost.speed;
    wallBoost.update(kDt, held, wall);
    ordinaryWall.update(kDt, held, wall);
    check(std::fabs(wallBoost.speed - ordinaryWall.speed) < 0.0001f &&
          wallBoost.speed <= afterWall + wallBoost.topSpeedKmh/3.6f/wallBoost.accelerationTime*kDt+.001f,
          "the next step cannot restore boost speed or momentum lost in a collision");

    Car resetBoost = slide(track, 1, model);
    resetBoost.update(kDt, held, track);
    check(resetBoost.boostRemaining > 0 && resetBoost.boostSerial == 1,
          "reset test starts with an active awarded boost");
    resetBoost.reset(track);
    check(resetBoost.driftCharge == 0 && resetBoost.driftChargeLevel == 0 &&
          resetBoost.boostRemaining == 0 && resetBoost.boostLevel == 0 && resetBoost.boostSerial == 0 &&
          resetBoost.boostSpeedBonus == 0 &&
          resetBoost.driftPhase == DriftPhase::None && resetBoost.driftAngleCommand == 0 &&
          !resetBoost.releaseEaseActive && resetBoost.releaseEaseSlip == 0 && resetBoost.releaseEaseElapsed == 0 &&
          !resetBoost.driftHeld && !resetBoost.driftHoldActive && !resetBoost.driftHoldBlocked &&
          resetBoost.driftEntrySpeed == 0 && resetBoost.driftAge == 0 && resetBoost.driftKick == 0,
          "reset clears an active boost and every held drift state");
    check(resetBoost.driftModel == model, "reset preserves the selected handling model");
}

void testPulseTopSpeedAndRehold(const Track& track, DriftModel model) {
    DriveInput throttle;
    throttle.throttle = 1;
    Car ordinaryTop = moving(track, 0, model);
    run(ordinaryTop, track, throttle, 3600);
    const float naturalTopSpeed = ordinaryTop.speed;
    check(near(naturalTopSpeed,200/3.6f,.0001f),
          "the ordinary drivetrain reaches the finite200km/h practice cap");
    run(ordinaryTop, track, throttle, 60);
    check(std::fabs(ordinaryTop.speed - naturalTopSpeed) < 0.01f,
          "normal top-speed reference has settled before testing the boost");
    DriveInput hold = throttle;
    hold.steer = hold.handbrake = 1;
    for (float topSpeedKmh : {60.0f,100.0f,160.0f}) {
        const float entrySpeed=topSpeedKmh/3.6f;
        Car car = moving(track, entrySpeed, model);
        car.topSpeedKmh=topSpeedKmh;
        run(car, track, hold, 180);
        check(car.driftChargeLevel == 3 && std::fabs(car.speed - heldSpeedAfter(entrySpeed, 3)) < 0.003f,
              "top-speed drift applies the same gentle slowdown while charging");
        car.update(kDt, throttle, track);
        check(car.boostSerial==1 && car.boostSpeedBonus==0,
              "a charged top-speed release emits an event before its speed attack has gained amplitude");
        run(car,track,throttle,6);
        check(car.speed > entrySpeed && car.speed <= entrySpeed+18 && car.boostSpeedBonus == 18,
              "charged pulse briefly exceeds every selected normal practice cap by its earned reward");
        const float peak = car.speed;
        while (car.boostRemaining > 0) {
            car.update(kDt, throttle, track);
            check(car.speed <= entrySpeed + car.boostSpeedBonus + 0.0001f,
                  "temporary top speed follows the shrinking pulse rather than a permanent cap increase");
        }
        check(car.speed <= entrySpeed && car.boostSpeedBonus == 0 && car.boostSerial == 1,
              "expiry returns within the ordinary cap without retriggering the reward");
        std::printf("top-speed pulse model%d: entry %.1f, peak %.1f, expired %.1fkm/h\n",
                    int(model), entrySpeed * 3.6f, peak * 3.6f, car.speed * 3.6f);
    }

    Car car = slide(track, 1, model);
    car.update(kDt, throttle, track);
    run(car,track,throttle,6);
    const float baseSpeed = car.speed - car.boostSpeedBonus;
    check(car.boostSpeedBonus == 18, "rehold check starts at the maximum real release pulse");
    Car straightRehold = car;
    DriveInput straightHold = throttle;
    straightHold.handbrake = 1;
    straightRehold.update(kDt, straightHold, track);
    check(straightRehold.boostSpeedBonus == 0 && straightRehold.boostRemaining == 0 && !straightRehold.releaseEaseActive &&
          straightRehold.speed <= baseSpeed + straightRehold.topSpeedKmh/3.6f/straightRehold.accelerationTime*kDt+.001f && !straightRehold.driftHoldActive,
          "even a neutral Space press cancels the pulse instead of banking its speed");
    car.update(kDt, hold, track);
    check(car.boostSpeedBonus == 0 && car.boostRemaining == 0 && !car.releaseEaseActive &&
          std::fabs(car.driftEntrySpeed - baseSpeed) < 0.0001f &&
          std::fabs(car.speed - heldSpeedAfter(baseSpeed, kDt)) < 0.003f && car.boostSerial == 1,
          "restarting drift captures only the ordinary base speed and cancels the existing pulse");
    run(car, track, hold, 180);
    check(car.driftChargeLevel == 3 && std::fabs(car.speed - heldSpeedAfter(baseSpeed, 181 * kDt)) < 0.003f,
          "new slide slows from its unboosted entry without banking old boosted excess");
    Car noCharge = car;
    noCharge.driftCharge = noCharge.driftChargeLevel = 0;
    car.update(kDt, throttle, track);
    noCharge.update(kDt, throttle, track);
    check(car.boostSerial == 2 && car.boostSpeedBonus == 0 && car.speed==noCharge.speed,
          "a newly charged release earns one fresh zero-start attack without stacking the previous reward");
    run(car,track,throttle,6);
    run(noCharge,track,throttle,6);
    check(near(car.boostSpeedBonus,18,.001f) && near(car.speed-noCharge.speed,18,.001f),
          "the freshly earned attack reaches its own peak without banking a previous boost");
}

void testWall() {
    Track track = straight(8);
    Car car = moving(track, 24);
    car.x = 7.9f;
    car.yaw = -0.4f;       // Body faces away from the wall.
    car.travelYaw = 0.6f;  // Momentum still hits the wall during a slide.
    car.slipAngle = -1.0f;
    car.driftBlend = 0.8f;
    car.driftDirection = -1;
    car.driftPhase = DriftPhase::Held;
    car.driftAngleCommand = 1.0f;
    car.drifting = true;
    car.driftHoldActive = true;
    car.handbrakeHeld = true;
    car.driftHeld = true;
    car.driftEntrySpeed = 24;
    car.driftCharge = 2.2f;
    car.driftChargeLevel = 3;
    car.boostRemaining = 0.50f;
    car.boostLevel = 2;
    car.boostSpeedBonus = 14;
    car.speed += car.boostSpeedBonus;
    DriveInput input;
    input.throttle = 0.5f;
    input.steer = -0.4f;
    input.handbrake = 1;
    car.update(kDt, input, track);
    check(car.impact > 10, "wall impact follows velocity even when body faces inward");
    check(std::fabs(car.ground.lateral) <= track.wallLateral() + 0.001f,
          "wall correction keeps the car on its allowed side");
    float outward = std::sin(car.travelYaw) * car.speed;
    check(outward < 0.001f && car.speed > 15, "wall absorbs outward velocity and preserves tangential speed");
    check(car.driftCharge == 0 && car.driftChargeLevel == 0 && car.boostRemaining == 0 && car.boostSpeedBonus == 0 &&
          !car.driftHoldActive && car.boostSerial == 0 && car.driftKick == 0,
          "wall impact cancels charge, boost and speed retention");
    const float impactSpeed = car.speed;
    car.update(kDt, input, track);
    check(car.speed < impactSpeed + 0.5f, "holding drift after impact does not restore pre-impact speed");

    car = moving(track, -8);
    car.x = 7.99f;
    car.yaw = car.travelYaw = -0.6f;
    car.update(kDt, {}, track);
    check(car.impact > 3 && car.speed < 0, "reverse wall collisions preserve the reverse speed sign");
    check(std::sin(car.travelYaw) * car.speed < 0.001f, "reverse wall collisions remove outward velocity");

    car.reset(track);
    DriveInput reverse;
    reverse.brake = 1;
    run(car, track, reverse, 120);
    check(car.speed < -8 && !car.drifting, "holding brake from rest sustains reverse without oscillating near zero");
    reverse.brake = 0;
    reverse.throttle = 1;
    run(car, track, reverse, 120);
    check(car.speed > 5, "throttle stops reverse motion then accelerates forward");
}

void testCorner() {
    Track track;
    track.makeTestCorner();
    const Track broad = straight();
    constexpr float pi = 3.14159265f;
    // The old full-corner script assumed a gradual exit and no longer describes
    // manual release. Seed a real charged slide on the legacy exit straight,
    // aim along the road, then check the quick eased exit against its geometry.
    for (DriftModel model : {DriftModel::Inertia, DriftModel::LineControl}) for (float side : {-1.0f, 1.0f}) {
        Car car = slide(broad, side, model);
        DriveInput hold;
        hold.throttle = hold.handbrake = 1;
        run(car, broad, hold, 120);
        car.x = 54;
        car.z = 50;
        car.yaw = pi / 2;
        car.travelYaw = wrapPi(car.yaw - car.slipAngle);
        car.ground = track.query(car.x, car.z, 0);
        car.onAsphalt = true;
        float maximumLateral = 0;
        int wallHits = 0;
        int offroad = 0;
        DriveInput input;
        input.throttle = 1;
        for (int frame = 0; frame < 60; ++frame) {
            car.update(kDt, input, track);
            maximumLateral = std::fmax(maximumLateral, std::fabs(car.ground.lateral));
            if (car.impact > 0.01f) ++wallHits;
            if (!car.onAsphalt) ++offroad;
        }
        check(car.ground.found && car.ground.s >= TrialCourse::finishS && offroad == 0 && wallHits == 0 &&
              maximumLateral > 0.25f && maximumLateral < 2.0f && std::fabs(car.z - 50) < 2.0f,
              "quick eased exit retains a small sideways displacement but crosses the legacy finish without wall assistance");
        check(car.slipAngle == 0 && car.speed * 3.6f >= 65 && car.boostSerial == 1,
              "legacy track geometry supports the new exit and exactly one charged speed pulse");
        std::printf("legacy exit model%d side%+.0f: %.3fm lateral, %d hits, %d offroad, %.1fkm/h\n",
                    int(model), side, maximumLateral, wallHits, offroad, car.speed * 3.6f);
    }
}

void testStress(const Track& track, DriftModel model = DriftModel::Inertia) {
    Car car = moving(track, 24, model);
    Car repeat = car;
    for (int i = 0; i < 60 * 180; ++i) {
        DriveInput input;
        input.throttle = (i / 180) % 3 == 0 ? 0.0f : 1.0f;
        input.brake = (i / 120) % 7 == 0 ? 0.7f : 0;
        input.handbrake = i % 360 < 180 ? 1.0f : 0;
        input.steer = std::sin(float(i) * 0.017f);
        car.update(kDt, input, track);
        repeat.update(kDt, input, track);
        if (!std::isfinite(car.x) || !std::isfinite(car.z) || !std::isfinite(car.speed) ||
            !std::isfinite(car.yaw) || !std::isfinite(car.slipAngle)) {
            check(false, "three-minute mixed-input simulation stays finite");
            return;
        }
        if (car.x != repeat.x || car.z != repeat.z || car.speed != repeat.speed ||
            car.yaw != repeat.yaw || car.slipAngle != repeat.slipAngle ||
            car.throttleLevel != repeat.throttleLevel || car.driftCharge != repeat.driftCharge ||
            car.driftAge != repeat.driftAge || car.driftKick != repeat.driftKick ||
            car.driftPhase != repeat.driftPhase || car.driftAngleCommand != repeat.driftAngleCommand ||
            car.releaseEaseActive != repeat.releaseEaseActive || car.releaseEaseSlip != repeat.releaseEaseSlip ||
            car.releaseEaseElapsed != repeat.releaseEaseElapsed ||
            car.boostRemaining != repeat.boostRemaining || car.boostSpeedBonus != repeat.boostSpeedBonus ||
            car.boostTotalDuration != repeat.boostTotalDuration || car.boostAttackDuration != repeat.boostAttackDuration ||
            car.boostHoldDuration != repeat.boostHoldDuration || car.boostReturnDuration != repeat.boostReturnDuration ||
            car.boostSerial != repeat.boostSerial) {
            check(false, "replaying the same fixed-step input sequence is deterministic");
            return;
        }
        check(car.driftIntensity >= 0 && car.driftIntensity <= 1,
              "drift feedback intensity stays normalized");
        check(std::fabs(car.driftIntensity - slipFeedback(car.speed, car.slipAngle, car.onAsphalt)) < 0.00001f,
              "physics uses the shared physical slip feedback function");
    }
    car.reset(track);
    check(car.speed == 0 && car.driftBlend == 0 && car.slipAngle == 0 && car.impact == 0 &&
          car.travelYaw == car.yaw && car.driftTravelRate == 0 && car.driftAge == 0 && car.driftKick == 0 &&
          car.driftCharge == 0 && car.driftChargeLevel == 0 &&
          car.boostRemaining == 0 && car.boostLevel == 0 && car.boostSerial == 0 && car.boostSpeedBonus == 0 &&
          !car.driftHeld && car.driftEntrySpeed == 0,
          "reset clears all motion, charge, boost and feedback state");
    check(!car.releaseEaseActive && car.releaseEaseSlip == 0 && car.releaseEaseElapsed == 0,
          "reset also clears every manual release easing state");
    check(car.driftPhase == DriftPhase::None && car.driftAngleCommand == 0,
          "reset clears the Entry/Held state and its remembered angle command");
}

} // namespace

void testRuntimeBoostParameters() {
    const Track track = straight();
    for (DriftModel model : {DriftModel::Inertia, DriftModel::LineControl}) {
        Car normal = moving(track,80/3.6f,model), quicker = normal;
        quicker.chargeRateScale = 2;
        DriveInput held;
        held.throttle = held.handbrake = 1;
        held.steer = .6f;
        run(normal,track,held,60);
        run(quicker,track,held,60);
        check(near(quicker.driftCharge,2*normal.driftCharge,.001f) &&
              quicker.driftChargeLevel>normal.driftChargeLevel,
              "runtime charge-rate tuning changes actual tier timing under the same input");
        Car tuned = slide(track,1,model);
        tuned.boostSpeedScale = 1.5f;
        tuned.boostDurationScale = 2;
        tuned.chargeRateScale = 2;
        DriveInput release;
        release.throttle = 1;
        tuned.update(kDt,release,track);
        check(tuned.boostLevel==3 && tuned.boostSpeedBonus==0 &&
              near(tuned.boostRemaining,2.2f,.001f) && near(tuned.boostTotalDuration,2.2f,.001f),
              "runtime boost strength and length scale the actual reward independently");
        run(tuned,track,release,21);
        check(near(tuned.boostSpeedBonus,27,.002f) && near(tuned.boostAttackDuration,.2f) && near(tuned.boostHoldDuration,.4f),
              "duration scaling stretches every captured stage while strength scales only its full-power gain");
        run(tuned,track,release,120);
        check(tuned.boostLevel==0 && tuned.boostRemaining==0 && tuned.boostSpeedBonus==0 && tuned.boostTotalDuration==0,
              "a tuned boost expires cleanly without retaining its scaled reward speed");
        tuned.reset(track);
        check(near(tuned.boostSpeedScale,1.5f) && near(tuned.boostDurationScale,2) &&
              near(tuned.chargeRateScale,2) && tuned.boostTotalDuration==0,
              "reset preserves selected physical settings while clearing active reward state");
        Car disabled = slide(track,1,model);
        disabled.boostSpeedScale = 0;
        disabled.update(kDt,release,track);
        check(disabled.boostSpeedBonus==0, "zero boost strength is a real zero instead of falling back to one");
        disabled.boostSpeedScale = std::numeric_limits<float>::quiet_NaN();
        disabled.boostDurationScale = 0;
        disabled.chargeRateScale = std::numeric_limits<float>::infinity();
        disabled.update(kDt,release,track);
        check(disabled.boostSpeedScale==1 && disabled.boostDurationScale==.25f && disabled.chargeRateScale==1 &&
              std::isfinite(disabled.speed), "invalid external physical settings fall back or clamp before simulation");
    }
}

void testLiveTuning() {
    const Track track = straight();
    Car car = moving(track, 29.0f);
    car.boostLevel = 3;
    car.boostSerial = 17;
    car.boostTotalDuration = 1.1f;
    car.boostRemaining = .4f;
    car.boostAttackDuration=.1f; car.boostHoldDuration=.2f; car.boostReturnDuration=.8f;
    car.boostSpeedBonus = 9;
    car.x = 3;
    car.z = 10;
    car.yaw = .4f;
    car.travelYaw = .2f;
    car.slipAngle = .2f;
    car.steerAngle = .1f;
    car.drifting = true;
    car.releaseEaseActive = true;
    car.releaseEaseSlip = .2f;
    car.releaseEaseElapsed = .05f;
    const Car before = car;
    FeelSettings settings;
    settings.boostDurationScale = 2;
    car.applyTuning(settings);
    check(near(car.boostTotalDuration,2.2f) && near(car.boostRemaining,.8f) &&
          near(car.speed,29) && near(car.boostSpeedBonus,9),
          "live duration changes preserve reward progress and instantaneous speed without a simulation tick");
    settings.boostSpeedScale = 2;
    car.applyTuning(settings);
    check(near(car.speed,38) && near(car.boostSpeedBonus,18),
          "live reward strength updates both speed and its removable component immediately");
    settings.boostSpeedScale = 0;
    car.applyTuning(settings);
    check(near(car.speed,20) && car.boostSpeedBonus==0 && car.boostRemaining>0,
          "zero live reward strength removes only the reward speed without ending the timer");
    settings.boostSpeedScale = 1;
    car.applyTuning(settings);
    check(near(car.speed,29) && near(car.boostSpeedBonus,9) && car.boostSerial==17,
          "restoring live strength from zero recovers the current reward without a fresh release event");
    check(car.x==before.x && car.z==before.z && car.yaw==before.yaw && car.travelYaw==before.travelYaw &&
          car.slipAngle==before.slipAngle && car.steerAngle==before.steerAngle &&
          car.driftPhase==before.driftPhase && car.driftCharge==before.driftCharge &&
          car.releaseEaseActive==before.releaseEaseActive && car.releaseEaseSlip==before.releaseEaseSlip &&
          car.releaseEaseElapsed==before.releaseEaseElapsed,
          "live reward tuning preserves the paused pose, drift phase, charge and release easing");
    Car ordinary = car;
    ordinary.speed -= ordinary.boostSpeedBonus;
    ordinary.boostSpeedBonus = ordinary.boostRemaining = ordinary.boostTotalDuration = 0;
    ordinary.boostLevel = 0;
    DriveInput input;
    car.update(kDt,input,track);
    ordinary.update(kDt,input,track);
    check(near(car.speed-car.boostSpeedBonus,ordinary.speed,.0002f) && car.boostSerial==17,
          "the next physics tick removes the synchronized bonus exactly once and preserves ordinary acceleration");

    Car held = slide(track,1);
    const Car heldBefore = held;
    settings.recoveryTime = 1.3f;
    settings.chargeRateScale = 2;
    held.applyTuning(settings);
    check(near(held.recoveryTime,1.3f) && near(held.chargeRateScale,2) &&
          held.driftCharge==heldBefore.driftCharge && held.driftChargeLevel==heldBefore.driftChargeLevel &&
          held.driftPhase==heldBefore.driftPhase && held.driftAngleCommand==heldBefore.driftAngleCommand &&
          held.speed==heldBefore.speed && held.boostRemaining==0 && held.boostTotalDuration==0 &&
          held.boostSpeedBonus==0 && held.boostSerial==heldBefore.boostSerial,
          "live charge and recovery settings affect future evolution without retroactive charge or revived boost");

    settings.recoveryTime = std::numeric_limits<float>::quiet_NaN();
    settings.boostSpeedScale = std::numeric_limits<float>::infinity();
    settings.boostDurationScale = 0;
    settings.chargeRateScale = std::numeric_limits<float>::quiet_NaN();
    car.applyTuning(settings);
    check(car.recoveryTime==.8f && car.boostSpeedScale==1 && car.boostDurationScale==.25f &&
          car.chargeRateScale==1 && std::isfinite(car.speed) && near(car.boostTotalDuration,.275f),
          "live tuning sanitizes non-finite values and bounds before rescaling an active reward");
    settings.recoveryTime = 100;
    settings.boostSpeedScale = -1;
    settings.boostDurationScale = 100;
    settings.chargeRateScale = -1;
    car.applyTuning(settings);
    check(car.recoveryTime==1.5f && car.boostSpeedScale==0 && car.boostDurationScale==3 &&
          car.chargeRateScale==.25f && near(car.boostTotalDuration,3.3f) &&
          car.boostSpeedBonus==0 && car.boostSerial==17,
          "live tuning bounds extreme settings without restarting reward progress or its event serial");
}

void testDriftLineSteering() {
    constexpr float pi=3.14159265359f;
    const Track track=straight();
    float responseTimes[2]={};
    for (DriftModel model : {DriftModel::Inertia,DriftModel::LineControl}) {
        float maxNeutralSweep=0, maxReversalTime=0;
        for (float side : {-1.0f,1.0f}) {
            Car base=slide(track,side,model);
            DriveInput hold;
            hold.throttle=hold.handbrake=1;
            run(base,track,hold,120);
            check(base.driftPhase==DriftPhase::Held && std::fabs(base.driftTravelRate)<.00001f,
                  "real held neutral setup has a sideways body and a settled straight travel line");
            for (float direction : {-1.0f,1.0f}) {
                Car turning=base, offThrottle=base;
                const float startHeading=base.travelYaw;
                float totalTurn=0, time90=0;
                for (int frame=0;frame<60;++frame) {
                    hold.steer=direction;
                    const Car previous=turning;
                    turning.update(kDt,hold,track);
                    DriveInput coasting=hold;
                    coasting.throttle=0;
                    offThrottle.update(kDt,coasting,track);
                    totalTurn+=wrapPi(turning.travelYaw-previous.travelYaw);
                    if (!time90 && direction*turning.driftTravelRate>=1.8f*.9f-.00001f) time90=(frame+1)*kDt;
                    const float dx=turning.x-previous.x,dz=turning.z-previous.z;
                    check(near(std::hypot(dx,dz),turning.speed*kDt,.0001f) &&
                          near(wrapPi(std::atan2(dx,dz)-turning.travelYaw),0,.0001f),
                          "large held line steering moves the real velocity and position along its commanded arc");
                    check(turning.x==offThrottle.x && turning.z==offThrottle.z && turning.yaw==offThrottle.yaw &&
                          turning.travelYaw==offThrottle.travelYaw && turning.speed==offThrottle.speed,
                          "throttle does not alter the drift steering response, sideways angle or mild held speed loss");
                    check(side*turning.slipAngle>=44.99f*pi/180 && side*turning.slipAngle<=60.01f*pi/180 &&
                          turning.driftDirection==side && turning.driftPhase==DriftPhase::Held && near(turning.speed,20,.003f),
                          "line steering in either direction preserves the chosen-side45..60degree drift and90percent speed floor");
                }
                const float across=(turning.x-base.x)*std::cos(startHeading)-(turning.z-base.z)*std::sin(startHeading);
                check(direction*totalTurn>1.5f && direction*across>8 && near(turning.driftTravelRate,direction*1.8f,.0001f),
                      "one second of full steering visibly bends the actual line by over85degrees and moves it over8metres sideways");
                const float expectedTime=model==DriftModel::Inertia?.192f:.12f;
                check(time90>=expectedTime-.0001f && time90<expectedTime+kDt+.0001f,
                      "the line reaches90percent of full steering at the selected model's time90");
                responseTimes[model==DriftModel::Inertia?0:1]=time90;

                Car reversing=turning;
                hold.steer=-direction;
                float reversalTime=0;
                for (int frame=0;frame<30;++frame) {
                    const float oldSlip=reversing.slipAngle;
                    reversing.update(kDt,hold,track);
                    if (!reversalTime && direction*reversing.driftTravelRate<0) reversalTime=(frame+1)*kDt;
                    check(side*reversing.slipAngle>=44.99f*pi/180 && side*reversing.slipAngle<=60.01f*pi/180 &&
                          std::fabs(wrapPi(reversing.slipAngle-oldSlip))<2*pi/180,
                          "fast reversed line steering cannot unwind, flip or jerk the held sideways pose");
                }
                check(reversalTime>0 && reversalTime<=.1f && -direction*reversing.driftTravelRate>1.79f,
                      "opposite steering reverses the actual curvature within100ms and reaches the opposite full turn");
                maxReversalTime=std::fmax(maxReversalTime,reversalTime);

                const float turningHeading=turning.travelYaw;
                hold.steer=0;
                float neutralSweep=0;
                for (int frame=0;frame<60;++frame) {
                    const float previousHeading=turning.travelYaw;
                    turning.update(kDt,hold,track);
                    neutralSweep+=wrapPi(turning.travelYaw-previousHeading);
                }
                maxNeutralSweep=std::fmax(maxNeutralSweep,std::fabs(neutralSweep));
                check(direction*neutralSweep>0 && std::fabs(neutralSweep)<.2f && std::fabs(turning.driftTravelRate)<.00002f &&
                      turning.driftHoldActive && std::fabs(turning.slipAngle)>44.99f*pi/180,
                      "neutral steering smoothly removes curvature with under12degrees of carry while preserving the manual drift");
                const float straightHeading=turning.travelYaw;
                run(turning,track,hold,60);
                check(std::fabs(wrapPi(turning.travelYaw-straightHeading))<.0001f &&
                      direction*wrapPi(straightHeading-turningHeading)>0,
                      "the settled neutral line stays straight instead of continuing to orbit or aligning to the nose");
            }
        }
        std::printf("drift line model%d: full %.1fdeg/s, radius at80km/h %.2fm / held72km/h %.2fm, time90 %.3fs, reverse sign %.3fs, neutral carry %.2fdeg\n",
                    int(model),1.8f*180/pi,(80/3.6f)/1.8f,20/1.8f,responseTimes[int(model)],maxReversalTime,maxNeutralSweep*180/pi);

        Car live=slide(track,1,model);
        DriveInput hold;
        hold.throttle=hold.handbrake=1;
        run(live,track,hold,120);
        FeelSettings mild;
        mild.driftSteerRate=.6f;
        mild.driftSteerResponse=.5f;
        const Car before=live;
        live.applyTuning(mild);
        check(live.x==before.x && live.z==before.z && live.yaw==before.yaw && live.travelYaw==before.travelYaw &&
              live.driftTravelRate==before.driftTravelRate && live.driftCharge==before.driftCharge && live.speed==before.speed,
              "live line tuning preserves the current car pose, line curvature, charge and speed without resetting its run");
        hold.steer=1;
        run(live,track,hold,180);
        check(near(live.driftTravelRate,.6f,.0002f), "a mild live steering gain changes the actual held curvature");
        FeelSettings strong=mild;
        strong.driftSteerRate=2.4f;
        strong.driftSteerResponse=.05f;
        const float previousRate=live.driftTravelRate;
        live.applyTuning(strong);
        check(live.driftTravelRate==previousRate && near(live.driftSteerRate,2.4f) && near(live.driftSteerResponse,.05f),
              "stronger live steering changes future response without snapping current curvature");
        live.update(kDt,hold,track);
        check(live.driftTravelRate>previousRate+.5f, "the stronger live gain and quicker response are felt on the very next physics frame");
        run(live,track,hold,5);
        check(live.driftTravelRate>2.2f && std::fabs(live.slipAngle)>=44.99f*pi/180,
              "the edited strong line turns rapidly without cancelling or unwinding the held drift");
        Car quick=live,slow=live;
        FeelSettings slower=strong;
        slower.driftSteerResponse=.5f;
        slow.applyTuning(slower);
        hold.steer=-1;
        run(quick,track,hold,3);
        run(slow,track,hold,3);
        check(quick.driftTravelRate<0 && slow.driftTravelRate>1 && slow.driftTravelRate-quick.driftTravelRate>2,
              "live response tuning independently changes reversal inertia at the same steering strength");
        live.reset(track);
        check(near(live.driftSteerRate,2.4f) && near(live.driftSteerResponse,.05f) && live.driftTravelRate==0,
              "reset keeps selected steering tuning while clearing the old line-motion state");

        for (float gain : {.3f,3.0f}) for (float response : {.05f,.5f}) {
            Car extreme=slide(track,1,model);
            FeelSettings custom;
            custom.driftSteerRate=gain;
            custom.driftSteerResponse=response;
            extreme.applyTuning(custom);
            for (int frame=0;frame<300;++frame) {
                hold.steer=(frame/6)%2?1:-1;
                extreme.update(kDt,hold,track);
                check(std::isfinite(extreme.yaw) && std::isfinite(extreme.travelYaw) &&
                      extreme.slipAngle>=44.99f*pi/180 && extreme.slipAngle<=60.01f*pi/180 &&
                      extreme.driftHoldActive && near(extreme.speed,20,.003f),
                      "rapid line reversals at every permitted steering extreme stay finite, bounded and manually held");
            }
        }
        FeelSettings invalid;
        invalid.driftSteerRate=std::numeric_limits<float>::quiet_NaN();
        invalid.driftSteerResponse=0;
        live.applyTuning(invalid);
        check(near(live.driftSteerRate,1.8f) && near(live.driftSteerResponse,.05f),
              "live drift steering rejects nonfinite strength and bounds zero response before simulation");
    }
    check(responseTimes[0]>responseTimes[1], "both models steer the line strongly while A retains a measurably slower response than B");
}

void testQuickPracticeSpeed() {
    const Track track=straight();
    DriveInput throttle;
    throttle.throttle=1;
    auto reachCap=[&](float cap,float seconds) {
        Car car=moving(track,0);
        FeelSettings settings;
        settings.topSpeedKmh=cap;
        settings.accelerationTime=seconds;
        car.applyTuning(settings);
        float reached=0;
        for (int frame=0;frame<int((seconds+5)*60);++frame) {
            const float previous=car.speed;
            car.update(kDt,throttle,track);
            if (!reached && car.speed*3.6f>=cap-.001f) reached=(frame+1)*kDt;
            check(car.speed>=previous-.0001f && car.speed*3.6f<=cap+.0001f,
                  "full throttle accelerates monotonically to the finite practice cap without ordinary overspeed");
            if (reached) check(near(car.speed*3.6f,cap,.001f),
                               "holding W maintains99..100percent of the selected cap for the following five seconds");
        }
        check(reached>=seconds-.0001f && reached<=seconds+.12f,
              "forward acceleration reaches its finite cap in the selected time plus only a short input-smoothing delay");
        return reached;
    };
    const float defaultTime=reachCap(200,1);
    check(defaultTime<=1.2f, "the default zero-to200km/h run takes at most1.2seconds");
    reachCap(60,1);
    reachCap(160,1);
    const float quickTime=reachCap(100,.4f),slowTime=reachCap(100,2);
    check(quickTime<.55f && slowTime>2 && slowTime>quickTime+1.4f,
          "the live acceleration-time parameter materially changes how quickly the same cap is reached");

    Car stationary=moving(track,0);
    DriveInput none;
    run(stationary,track,none,300);
    check(stationary.speed==0 && stationary.x==0 && stationary.z==0,
          "quick practice acceleration never drives the car without an accelerator input");
    Car coasting=moving(track,100/3.6f);
    coasting.topSpeedKmh=100; // Retain the explicit100km/h coast/brake regression.
    coasting.throttleLevel=1;
    run(coasting,track,none,60);
    check(coasting.speed<100/3.6f && coasting.speed>24,
          "releasing W immediately keeps the original gentle drag and rolling coast rather than seeking the cap");
    Car braking=coasting;
    DriveInput brake;
    brake.brake=1;
    run(braking,track,brake,60);
    check(braking.speed<1 && braking.speed>=-9,
          "brake force still stops a practice-speed car independently of the new acceleration setting");
    Car reversing=moving(track,0);
    run(reversing,track,brake,120);
    check(reversing.speed<-8 && reversing.speed>=-9,
          "holding brake still enters the existing bounded reverse instead of applying forward practice acceleration");

    Car live=moving(track,100/3.6f);
    live.x=3; live.z=20; live.yaw=.4f; live.travelYaw=.3f; live.slipAngle=.1f;
    const Car before=live;
    FeelSettings low;
    low.topSpeedKmh=60;
    live.applyTuning(low);
    check(near(live.speed,60/3.6f) && live.x==before.x && live.z==before.z && live.yaw==before.yaw &&
          live.travelYaw==before.travelYaw && live.slipAngle==before.slipAngle &&
          near(live.driftIntensity,slipFeedback(live.speed,live.slipAngle,live.onAsphalt)),
          "lowering the normal cap while paused clamps only speed and refreshes slip feedback without moving or rotating the car");
    FeelSettings high=low;
    high.topSpeedKmh=160;
    high.accelerationTime=.4f;
    const float heldSpeed=live.speed;
    live.applyTuning(high);
    check(live.speed==heldSpeed && near(live.topSpeedKmh,160) && near(live.accelerationTime,.4f),
          "raising the live cap and changing acceleration time cannot teleport current speed");
    live.update(kDt,throttle,track);
    check(live.speed>heldSpeed && live.speed<160/3.6f,
          "the raised cap and faster acceleration start acting on the next accelerator-driven frame");

    for (DriftModel model : {DriftModel::Inertia,DriftModel::LineControl}) {
        Car held=moving(track,100/3.6f,model);
        DriveInput hold=throttle;
        hold.handbrake=hold.steer=1;
        run(held,track,hold,300);
        check(near(held.speed,90/3.6f,.003f) && held.driftPhase==DriftPhase::Held,
              "drifting at the new top speed keeps the existing slight slowdown and90percent manual hold floor");
        const Car heldBefore=held;
        held.applyTuning(low);
        check(near(held.speed,60/3.6f) && held.driftHoldActive && held.driftPhase==heldBefore.driftPhase &&
              held.driftCharge==heldBefore.driftCharge && held.driftDirection==heldBefore.driftDirection &&
              held.driftAngleCommand==heldBefore.driftAngleCommand && held.yaw==heldBefore.yaw,
              "lowering the live cap during drift keeps charge, chosen-side pose and manual entry/exit state");
        Car boosted=heldBefore;
        boosted.update(kDt,throttle,track);
        run(boosted,track,throttle,6);
        const Car reward=boosted;
        check(reward.boostSpeedBonus==18 && reward.speed>100/3.6f,
              "the existing earned release boost still exceeds the lower normal top speed");
        boosted.applyTuning(low);
        check(near(boosted.speed,60/3.6f+reward.boostSpeedBonus) && boosted.boostSpeedBonus==reward.boostSpeedBonus &&
              boosted.boostRemaining==reward.boostRemaining && boosted.boostTotalDuration==reward.boostTotalDuration &&
              boosted.boostSerial==reward.boostSerial && boosted.x==reward.x && boosted.z==reward.z,
              "a paused cap reduction clamps only the normal component while preserving active boost strength, progress and position");
        const float boostedSpeed=boosted.speed;
        boosted.applyTuning(high);
        check(boosted.speed==boostedSpeed && boosted.boostRemaining==reward.boostRemaining && boosted.boostSerial==reward.boostSerial,
              "raising a cap during boost neither adds base speed nor restarts its reward");
        boosted.reset(track);
        check(boosted.speed==0 && boosted.topSpeedKmh==160 && near(boosted.accelerationTime,.4f),
              "reset preserves selected practice speed and acceleration while clearing motion");
    }
    FeelSettings invalid;
    invalid.topSpeedKmh=std::numeric_limits<float>::infinity();
    invalid.accelerationTime=0;
    live.applyTuning(invalid);
    check(live.topSpeedKmh==200 && near(live.accelerationTime,.2f),
          "invalid live practice settings fall back or clamp before they can affect a force calculation");
    std::printf("practice acceleration: zero-to200 %.3fs, selected.4 %.3fs / selected2 %.3fs; normal cap200km/h\n",
                defaultTime,quickTime,slowTime);
}

void testLiveBoostFlow() {
    const Track track=straight();
    DriveInput release;
    release.throttle=1;
    auto reward=[&]() {
        Car car=slide(track,1);
        car.update(kDt,release,track);
        return car;
    };
    auto samePausedRun=[&](const Car& car,const Car& before) {
        check(car.x==before.x && car.z==before.z && car.yaw==before.yaw && car.travelYaw==before.travelYaw &&
              car.boostSerial==before.boostSerial && near(car.speed, before.speed,.0003f) &&
              near(car.boostSpeedBonus,before.boostSpeedBonus,.0003f) && near(car.boostEnvelopeAmount(),before.boostEnvelopeAmount(),.00003f),
              "a paused stage-time edit preserves position, direction, current gain, ordinary speed and the original reward event");
    };
    FeelSettings longer;
    longer.boostSpeedAttack=.2f;
    longer.boostSpeedHold=.4f;
    longer.boostSpeedReturn=1.6f;
    Car attack=reward();
    run(attack,track,release,3);
    const Car halfAttack=attack;
    check(near(attack.boostSpeedBonus,9,.001f), "the default speed attack reaches half gain at50ms");
    attack.applyTuning(longer);
    samePausedRun(attack,halfAttack);
    check(near(attack.boostTotalDuration,2.2f) && near(attack.boostRemaining,2.1f,.0001f),
          "an attack edit preserves its half-stage progress rather than overall lifetime progress");
    run(attack,track,release,6);
    check(near(attack.boostSpeedBonus,18,.001f), "the edited attack reaches full gain after its remaining100ms");

    Car holding=reward();
    run(holding,track,release,12);
    const Car halfHold=holding;
    FeelSettings holdTime;
    holdTime.boostSpeedHold=.6f;
    holding.applyTuning(holdTime);
    samePausedRun(holding,halfHold);
    check(near(holding.boostTotalDuration,1.5f) && near(holding.boostRemaining,1.1f,.0001f),
          "a hold-time edit retains the halfway point of the plateau without restarting the attack");
    run(holding,track,release,18);
    check(near(holding.boostSpeedBonus,18,.001f), "the edited full-power plateau lasts for its remaining300ms");
    run(holding,track,release,24);
    check(near(holding.boostSpeedBonus,9,.001f), "the return after an edited hold retains its independent800ms smooth shape");

    Car returning=reward();
    run(returning,track,release,42);
    const Car halfReturn=returning;
    returning.applyTuning(longer);
    samePausedRun(returning,halfReturn);
    check(near(returning.boostRemaining,.8f,.0001f), "a return edit retains its half-return progress after the earlier phases change length");
    longer.boostDurationScale=.25f;
    const Car beforeScale=returning;
    returning.applyTuning(longer);
    samePausedRun(returning,beforeScale);
    check(near(returning.boostAttackDuration,.05f) && near(returning.boostHoldDuration,.1f) &&
          near(returning.boostReturnDuration,.4f) && near(returning.boostRemaining,.2f,.0001f),
          "the global duration scale multiplies all captured phases while keeping the current stage gain unchanged");
    const float base=returning.speed-returning.boostSpeedBonus;
    longer.boostSpeedScale=0;
    returning.applyTuning(longer);
    check(returning.boostSpeedBonus==0 && near(returning.speed,base) && returning.boostRemaining>0,
          "zero live boost strength removes only the bonus while its valid phase clock remains active");
    longer.boostSpeedScale=2;
    returning.applyTuning(longer);
    check(near(returning.boostSpeedBonus,18,.001f) && near(returning.speed,base+18,.001f) && returning.boostSerial==1,
          "restoring or increasing strength acts immediately at the same phase gain without creating a new event");
    const Car pause=returning;
    returning.update(0,release,track);
    returning.update(std::numeric_limits<float>::infinity(),release,track);
    check(returning.boostRemaining==pause.boostRemaining && returning.speed==pause.speed && returning.x==pause.x,
          "paused and invalid elapsed time do not advance the physical speed envelope");

    Car noHold=reward();
    run(noHold,track,release,12);
    const Car held=noHold;
    FeelSettings removedHold;
    removedHold.boostSpeedHold=0;
    noHold.applyTuning(removedHold);
    samePausedRun(noHold,held);
    check(noHold.boostHoldDuration==0 && near(noHold.boostRemaining,.8f,.0001f),
          "removing a live plateau enters the start of return at full gain without dividing by zero");
    run(noHold,track,release,24);
    check(near(noHold.boostSpeedBonus,9,.001f), "a zero-hold profile returns smoothly from its peak over the selected duration");
    const Car middle=noHold;
    noHold.applyTuning(holdTime);
    samePausedRun(noHold,middle);
    check(near(noHold.boostRemaining,.4f,.0001f),
          "adding a hold during return preserves the return stage instead of replaying the new plateau");

    Car smooth=reward();
    smooth.update(.099f,release,track);
    const float beforePeak=smooth.boostSpeedBonus;
    smooth.update(.001f,release,track);
    const float peak=smooth.boostSpeedBonus;
    smooth.update(.001f,release,track);
    const float peakAcceleration=std::fmax(std::fabs(peak-beforePeak),std::fabs(smooth.boostSpeedBonus-peak))/.001f;
    check(near(peak,18,.001f) && peakAcceleration<.5f,
          "speed gain arrives at its full-power plateau with nearly zero slope instead of an acceleration kink");
    smooth.update(.999f,release,track);
    check(smooth.boostRemaining==0 && smooth.boostSpeedBonus==0 && smooth.boostAttackDuration==0 &&
          smooth.boostHoldDuration==0 && smooth.boostReturnDuration==0,
          "natural expiry clears the captured speed-profile clock and rejoins the unboosted speed");
    for (float attackTime : {.03f,.3f}) for (float holdingTime : {0.0f,.6f})
        for (float returnTime : {.2f,2.0f}) for (float scale : {.25f,3.0f}) {
            Car extreme=reward();
            FeelSettings settings;
            settings.boostSpeedAttack=attackTime;
            settings.boostSpeedHold=holdingTime;
            settings.boostSpeedReturn=returnTime;
            settings.boostDurationScale=scale;
            extreme.applyTuning(settings);
            const float total=(attackTime+holdingTime+returnTime)*scale;
            check(near(extreme.boostTotalDuration,total,.0001f) && extreme.boostSpeedBonus==0,
                  "every permitted speed-profile setting preserves a zero-start release and finite captured phases");
            int frames=0;
            while (extreme.boostRemaining>0 && frames<600) {
                extreme.update(kDt,release,track);
                ++frames;
                check(std::isfinite(extreme.speed) && extreme.boostEnvelopeAmount()>=0 && extreme.boostEnvelopeAmount()<=1 &&
                      extreme.speed-extreme.boostSpeedBonus<=extreme.topSpeedKmh/3.6f+.0001f,
                      "speed-profile extremes remain finite and cannot bank bonus speed in the normal component");
            }
            check(extreme.boostRemaining==0 && frames*kDt>=total-.0001f && frames*kDt<=total+kDt+.0001f,
                  "all scaled speed-profile extremes end finitely after the selected total duration");
        }
    FeelSettings invalid;
    invalid.boostSpeedAttack=std::numeric_limits<float>::quiet_NaN();
    invalid.boostSpeedHold=-1;
    invalid.boostSpeedReturn=0;
    invalid.boostDurationScale=0;
    Car bounded=reward();
    bounded.applyTuning(invalid);
    check(near(bounded.boostSpeedAttack,.1f) && bounded.boostSpeedHold==0 && near(bounded.boostSpeedReturn,.2f) &&
          near(bounded.boostDurationScale,.25f) && near(bounded.boostTotalDuration,.075f),
          "invalid live phase durations and zero duration scale sanitize to finite positive attack/return stages");
    std::printf("boost flow: +32.4km/h at.05, +64.8 at.10..30, +32.4 at.70, zero at1.10; peak sampled acceleration %.3fm/s2\n",peakAcceleration);
}

void testCameraDirectedRelease() {
    constexpr float pi=3.14159265359f;
    const Track track=straight();
    auto charged=[&](DriftModel model,float side,float travel=0.0f) {
        Car car=slide(track,side,model);
        DriveInput neutral;
        neutral.throttle=neutral.handbrake=1;
        run(car,track,neutral,120);
        car.x=car.z=0;
        car.travelYaw=travel;
        car.yaw=wrapPi(travel+car.slipAngle);
        car.ground=track.query(0,0,0);
        return car;
    };
    float largestFirstStep=0,smallestLastStep=1;
    for(DriftModel model : {DriftModel::Inertia,DriftModel::LineControl})
        for(float side : {-1.0f,1.0f}) for(float travel : {0.0f,pi-.05f,-pi+.05f}) {
            Car car=charged(model,side,travel);
            const Car initial=car;
            const float target=wrapPi(travel+side*.32f);
            const float travelOffset=wrapPi(travel-target),bodyOffset=wrapPi(car.yaw-target);
            DriveInput release;
            release.throttle=1;release.hasBoostHeading=true;release.boostHeading=target;
            car.update(0,release,track);
            check(car.travelYaw==initial.travelYaw && car.handbrakeHeld && car.boostSerial==0,
                  "paused release does not consume its captured heading or advance the manual edge");
            std::vector<DriveInput> recorded;
            std::vector<Car> poses;
            for(int frame=1;frame<=15;++frame) {
                const Car previous=car;
                recorded.push_back(release);
                car.update(kDt,release,track);
                poses.push_back(car);
                const float remaining=std::fmax(1-frame*kDt/.25f,0.0f);
                const float eased=remaining*remaining*remaining;
                check(std::fabs(wrapPi(car.travelYaw-(target+travelOffset*eased)))<.0001f &&
                      std::fabs(wrapPi(car.yaw-(target+bodyOffset*eased)))<.0001f,
                      "both real velocity and body converge smoothly to the captured view direction across either angular seam");
                const float step=std::fabs(wrapPi(car.travelYaw-previous.travelYaw));
                if(frame==1) {
                    largestFirstStep=std::fmax(largestFirstStep,step);
                    check(step>0 && step<std::fabs(travelOffset)*.25f && car.releaseEaseActive &&
                          car.releaseUsesCameraHeading && car.boostSerial==1 && car.boostSpeedBonus==0,
                          "camera-facing release retains most incoming momentum on its first frame and awards one zero-start speed attack");
                }
                if(frame==15) smallestLastStep=std::fmin(smallestLastStep,step);
                const float dx=car.x-previous.x,dz=car.z-previous.z;
                check(std::fabs(wrapPi(std::atan2(dx,dz)-car.travelYaw))<.0001f &&
                      near(std::hypot(dx,dz),car.speed*kDt,.0001f),
                      "every smooth-release frame moves exactly once along its current blended velocity");
                check(near(car.yawRate,wrapPi(car.yaw-previous.yaw)/kDt,.0001f),
                      "reported body yaw rate includes the actual authored release rotation");
                // Further samples may carry a different camera pose, but only the
                // first valid release captures a direction for this transition.
                release.boostHeading=wrapPi(target+1);
            }
            check(!car.releaseEaseActive && !car.drifting && car.releaseUsesCameraHeading &&
                  car.slipAngle==0 && car.travelYaw==car.yaw && std::fabs(wrapPi(car.yaw-target))<.0001f,
                  "the250ms transition joins grip exactly while retaining its camera-return identity");
            Car replay=initial;
            for(size_t frame=0;frame<recorded.size();++frame) {
                replay.update(kDt,recorded[frame],track);
                check(replay.x==poses[frame].x && replay.z==poses[frame].z && replay.yaw==poses[frame].yaw &&
                      replay.travelYaw==poses[frame].travelYaw && replay.boostRemaining==poses[frame].boostRemaining,
                      "recorded world-heading inputs reproduce the same release path without consulting another camera");
            }
        }
    check(largestFirstStep<.065f && smallestLastStep<.0002f,
          "release starts with a bounded turn and its final step becomes very small rather than snapping at completion");

    DriveInput release;
    release.throttle=1;release.hasBoostHeading=true;release.boostHeading=.32f;
    for(DriftModel model : {DriftModel::Inertia,DriftModel::LineControl}) {
        Car seed=charged(model,1),left=seed,right=seed;
        release.steer=-1;run(left,track,release,15);
        release.steer=1;run(right,track,release,15);
        check(left.releaseHeading<.17f && right.releaseHeading>.47f &&
              std::fabs(wrapPi(left.travelYaw-left.releaseHeading))<.0001f &&
              std::fabs(wrapPi(right.travelYaw-right.releaseHeading))<.0001f,
              "left and right inputs still bend the captured target throughout the smooth release");
        release.steer=0;
        Car live=seed;
        run(live,track,release,3);
        const Car paused=live;
        FeelSettings slower;
        slower.releaseDirectionTime=.5f;
        live.applyTuning(slower);
        check(live.x==paused.x && live.yaw==paused.yaw && live.travelYaw==paused.travelYaw &&
              near(live.releaseEaseElapsed,.1f) && live.boostSerial==paused.boostSerial,
              "live direction timing preserves current pose and normalized transition progress without replaying its boost");
        run(live,track,release,24);
        check(!live.releaseEaseActive && near(live.travelYaw,.32f,.0001f),
              "the retimed direction transition finishes after only its remaining stage time");

        for(float invalid : {std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::infinity()}) {
            Car fallback=seed,legacy=seed;
            release.boostHeading=invalid;fallback.update(kDt,release,track);
            release.hasBoostHeading=false;legacy.update(kDt,release,track);
            check(!fallback.releaseUsesCameraHeading && fallback.x==legacy.x && fallback.z==legacy.z &&
                  fallback.travelYaw==legacy.travelYaw && fallback.yaw==legacy.yaw && fallback.boostSerial==1,
                  "nonfinite camera headings use the existing charged release instead of poisoning vehicle state");
            release.hasBoostHeading=true;
        }
        Car huge=seed;
        release.boostHeading=std::numeric_limits<float>::max();
        huge.update(kDt,release,track);
        check(huge.releaseUsesCameraHeading && huge.boostSerial==1 && std::isfinite(huge.x) &&
              std::isfinite(huge.travelYaw) && std::isfinite(huge.releaseHeading) &&
              std::fabs(huge.releaseHeading)<=pi,
              "a huge finite captured heading normalizes in bounded work and cannot hang or poison release motion");
        release.boostHeading=.32f;
        Car uncharged=moving(track,24,model);
        DriveInput hold;
        hold.throttle=hold.handbrake=hold.steer=1;
        run(uncharged,track,hold,15);
        Car legacy=uncharged;
        uncharged.update(kDt,release,track);
        DriveInput ordinary=release;ordinary.hasBoostHeading=false;
        legacy.update(kDt,ordinary,track);
        check(!uncharged.releaseUsesCameraHeading && uncharged.boostSerial==0 &&
              uncharged.yaw==legacy.yaw && uncharged.travelYaw==legacy.travelYaw,
              "a release with no earned charge keeps the existing150ms nose-directed response");
        Car zero=seed;zero.boostSpeedScale=0;zero.update(kDt,release,track);
        check(!zero.releaseUsesCameraHeading && zero.boostSpeedBonus==0,
              "a disabled speed reward cannot activate camera-directed launch state");

        for(int cause=0;cause<4;++cause) {
            Car interrupted=seed;
            interrupted.update(kDt,release,track);
            DriveInput next=release;
            Track surface=track;
            if(cause==0) next.brake=1;
            if(cause==1) {next.handbrake=next.steer=1;}
            if(cause>=2) {
                surface=straight(cause==2?6.5f:8.0f);
                if(cause==2) surface.runoff=8;
                interrupted.x=cause==2?7.0f:7.99f;
                interrupted.z=0;
                interrupted.ground=surface.query(interrupted.x,0,0);
            }
            interrupted.update(kDt,next,surface);
            check(!interrupted.releaseUsesCameraHeading && !interrupted.releaseEaseActive &&
                  interrupted.releaseHeading==0 && interrupted.releaseTravelOffset==0 && interrupted.boostRemaining==0,
                  "brake, rehold, grass and barrier interruptions all clear the captured heading and its easing state");
        }
        live.reset(track);
        check(!live.releaseUsesCameraHeading && live.releaseHeading==0 && live.releaseTravelOffset==0 &&
              !live.releaseEaseActive && near(live.releaseDirectionTime,.5f),
              "reset clears captured direction state while preserving the selected direction timing");
    }
    std::printf("camera-facing release: first travel step %.3fdeg, final step %.5fdeg, convergence .25sec; seam/replay/cancel green\n",
                largestFirstStep*180/pi,smallestLastStep*180/pi);
}

void testScaledCollision() {
    Track road=straight(6.5f);
    road.runoff=8;
    Track enlarged=road;
    enlarged.scaleBy(1.4f);
    Car small=moving(road,0),large=moving(enlarged,0);
    small.x=large.x=8;
    small.update(kDt,{},road);large.update(kDt,{},enlarged);
    check(!small.onAsphalt && large.onAsphalt && small.impact==0 && large.impact==0,
          "a wider scaled stage creates real asphalt for the unchanged vehicle physics");
    for(float side : {-1.0f,1.0f}) {
        small=moving(road,20);large=moving(enlarged,20);
        small.x=side*(road.wallLateral()+.3f);large.x=side*(enlarged.wallLateral()+.3f);
        small.yaw=small.travelYaw=large.yaw=large.travelYaw=side*1.57079632679f;
        small.update(kDt,{},road);large.update(kDt,{},enlarged);
        check(small.impact>19 && large.impact>19 && std::fabs(small.speed)<.001f && std::fabs(large.speed)<.001f &&
              near(std::fabs(small.x),road.wallLateral(),.0001f) &&
              near(std::fabs(large.x),enlarged.wallLateral(),.0001f),
              "the same car collides with both scaled barrier sides at the corresponding world boundary");
    }
}

Track verticalCourse(int shape) {
    Track track;
    track.closed=false;
    track.halfWidth=40;
    track.spawnZ=-2;
    for(int i=0;i<=760;++i) {
        const float z=-30+i*.25f;
        float y=0;
        if(shape==0 && z>0 && z<40) y=2*std::pow(std::sin(z*3.14159265359f/40),2);
        if(shape==1 && z>0 && z<20) y=-3*std::pow(std::sin(z*3.14159265359f/20),2);
        if(shape==2) y=z*.1f;
        track.samples.push_back({0,z,y,shape==2?.06f:0.0f,z+30});
    }
    track.length=190;
    track.spawnY=shape==2?-.2f:0;
    return track;
}

void testVerticalContact() {
    const Track flat=straight();
    DriveInput none;
    Car fall=moving(flat,0);
    fall.grounded=false;fall.y=2;fall.verticalSpeed=-1;
    for(int frame=0;frame<12;++frame) fall.update(kDt,none,flat);
    check(!fall.grounded && near(fall.y,2-.2f-.5f*9.81f*.2f*.2f,.0001f) &&
          near(fall.verticalSpeed,-1-9.81f*.2f,.0001f) && fall.groundHeight==0,
          "airborne height and velocity follow real ballistic gravity independently of the floor");
    float impact=0;
    for(int frame=0;frame<120;++frame) {
        fall.update(kDt,none,flat);
        impact=std::fmax(impact,fall.landingImpact);
    }
    check(fall.grounded && fall.y==0 && fall.verticalSpeed==0 && fall.airTime==0 &&
          fall.landingSerial==1 && impact>6,
          "a falling car hits the floor once, reports its absorbed vertical speed and settles without repeated landing events");

    float crestAir=0,dipClearance=0;
    for(int shape : {0,1}) {
        const Track course=verticalCourse(shape);
        Car car=moving(course,30);
        bool flew=false,landed=false;
        float maxGap=0,maxImpact=0;
        for(int frame=0;frame<240;++frame) {
            car.update(kDt,none,course);
            flew|=!car.grounded;landed|=car.landingSerial>0;
            if(!car.grounded) crestAir=std::fmax(crestAir,car.airTime);
            maxGap=std::fmax(maxGap,car.y-car.groundHeight);
            maxImpact=std::fmax(maxImpact,car.landingImpact);
            check(std::isfinite(car.y) && std::isfinite(car.verticalSpeed) && car.y>=car.groundHeight-.001f,
                  "real crest/dip traversal remains finite and never penetrates its selected support floor");
        }
        check(flew && landed && maxGap>.1f && maxImpact>1,
              "both a convex crest and a depression cause momentum-preserving flight and a real landing");
        if(shape==1) dipClearance=maxGap;
    }
    const Track slope=verticalCourse(2);
    Car uphill=moving(slope,20),reverse=moving(slope,-8);
    for(int frame=0;frame<60;++frame) {
        uphill.update(kDt,none,slope);reverse.update(kDt,none,slope);
    }
    check(uphill.grounded && reverse.grounded && uphill.landingSerial==0 && reverse.landingSerial==0 &&
          near(uphill.y,uphill.groundHeight,.0001f) && near(reverse.y,reverse.groundHeight,.0001f) &&
          near(uphill.terrainPitch,std::atan(.1f),.002f) && near(uphill.terrainRoll,std::atan(.06f),.002f) &&
          reverse.verticalSpeed<0,
          "constant grade/bank supports forward and reverse motion without fake jumps and reports the wheel-plane attitude");
    std::printf("grade contact: forward %d land%u pitch %.5f roll %.5f vy %.4f; reverse %d land%u vy %.4f\n",
                uphill.grounded,uphill.landingSerial,uphill.terrainPitch,uphill.terrainRoll,uphill.verticalSpeed,
                reverse.grounded,reverse.landingSerial,reverse.verticalSpeed);
    const auto wheelFloor=uphill.querySupport(slope,uphill.x+.82f,uphill.z+1.42f);
    check(wheelFloor.found && near(wheelFloor.centerY+wheelFloor.lateral*wheelFloor.cross,
                                 (uphill.z+1.42f)*.1f+.82f*.06f,.0001f) &&
          !uphill.querySupport(slope,std::numeric_limits<float>::quiet_NaN(),0).found,
          "render/contact callers share the exact sloped and banked support plane and reject invalid coordinates");

    Car coast=moving(flat,40);
    coast.grounded=false;coast.y=10;coast.verticalSpeed=3;coast.travelYaw=.7f;coast.yaw=1.2f;
    coast.airSteerControl=0; // Explicit zero-control mode preserves the pure ballistic regression.
    const Car airborne=coast;
    Car controlled=coast;
    DriveInput airInput;
    airInput.throttle=airInput.brake=airInput.handbrake=1;airInput.steer=-1;
    for(int frame=0;frame<30;++frame) {
        coast.update(kDt,none,flat);controlled.update(kDt,airInput,flat);
        check(coast.speed==airborne.speed && controlled.speed==airborne.speed && coast.x==controlled.x &&
              coast.z==controlled.z && coast.yaw==controlled.yaw && coast.travelYaw==controlled.travelYaw &&
              controlled.driftCharge==0 && controlled.boostSerial==0,
              "zero air-control mode preserves horizontal momentum despite throttle/brake/steering/Space input");
    }
    FeelSettings edit;
    edit.gravity=18;edit.engineGain=.3f;
    edit.topSpeedKmh=120; // Explicit historical airborne-overspeed fixture.
    const Car before=coast;
    coast.applyTuning(edit);
    check(coast.speed==before.speed && coast.y==before.y && coast.verticalSpeed==before.verticalSpeed && coast.gravity==18,
          "live gravity and unrelated gain edits affect future flight without clamping its current overspeed or pose");
    Track resized=flat;resized.scaleBy(1.7f);
    coast.rescaleWorld(1.7f,resized);
    check(!coast.grounded && near(coast.y,before.y*1.7f,.0001f) &&
          near(coast.verticalSpeed,before.verticalSpeed*1.7f,.0001f) && coast.landingSerial==before.landingSerial,
          "a midair stage resize transports height and vertical momentum without a false landing");
    coast.update(kDt,none,resized);
    check(!coast.grounded && coast.landingSerial==0,"flight continues after a stage edit without consuming a contact event");
    Track largerSlope=slope;largerSlope.scaleBy(1.4f);
    uphill.rescaleWorld(1.4f,largerSlope);
    uphill.update(kDt,none,largerSlope);
    check(uphill.grounded && uphill.landingSerial==0,"a grounded stage resize rebases support without a fake crest impulse");

    // Crossing roads with a ten-metre deck separation. The airborne car is
    // closer to the upper deck, but its last supporting route is the lower one.
    Track bridge;
    bridge.samples={{0,-100,0,0,0},{0,100,0,0,200},{100,100,10,0,300},
                    {100,0,10,0,400},{-100,0,10,0,600},{-100,-100,0,0,700}};
    bridge.length=800;bridge.spawnZ=-10;bridge.halfWidth=6.5f;
    Car under=moving(bridge,20);
    under.grounded=false;under.y=12;
    for(int frame=0;frame<120;++frame) {
        under.update(kDt,none,bridge);
        check(under.groundHeight==0 && under.ground.s<200 && std::isfinite(under.y),
              "airborne height never selects the upper bridge instead of the last supporting lower road");
    }
    check(under.grounded && under.landingSerial==1 && under.y==0,
          "the crossing jump lands on its original lower route rather than snapping onto the overpass");
    Car repeat=airborne,original=airborne;
    for(int frame=0;frame<180;++frame) {
        DriveInput input=frame%20<10?airInput:none;
        original.update(kDt,input,flat);repeat.update(kDt,input,flat);
        check(original.y==repeat.y && original.verticalSpeed==repeat.verticalSpeed && original.x==repeat.x &&
              original.grounded==repeat.grounded && original.landingSerial==repeat.landingSerial,
              "mixed airborne/landing fixed-step replay remains deterministic");
    }
    repeat.grounded=false;repeat.y=20;
    repeat.update(std::numeric_limits<float>::max(),none,flat);
    check(std::isfinite(repeat.y) && std::isfinite(repeat.speed) && std::isfinite(repeat.verticalSpeed),
          "an exceptional huge elapsed time is bounded and subdivided without overflow or an unbounded loop");
    repeat.reset(flat);
    check(repeat.grounded && repeat.y==0 && repeat.groundHeight==0 && repeat.verticalSpeed==0 &&
          repeat.landingSerial==0 && repeat.landingImpact==0 && repeat.airTime==0,
          "reset clears every flight/landing state and restores real floor contact");
    FeelSettings invalid;
    invalid.gravity=std::numeric_limits<float>::quiet_NaN();repeat.applyTuning(invalid);
    check(near(repeat.gravity,9.81f),"invalid gravity restores the finite physical default before simulation");
    invalid.gravity=100;repeat.applyTuning(invalid);repeat.reset(flat);
    check(repeat.gravity==22,"gravity is bounded and the selected value survives a physical reset");
    Track wall=straight(8);
    Car airborneWall=moving(wall,40);
    airborneWall.grounded=false;airborneWall.y=3;airborneWall.x=7.99f;
    airborneWall.yaw=airborneWall.travelYaw=1.57079632679f;
    airborneWall.boostLevel=3;airborneWall.boostRemaining=.5f;airborneWall.boostTotalDuration=1.1f;
    airborneWall.boostSpeedBonus=18;
    airborneWall.update(kDt,none,wall);
    check(!airborneWall.grounded && airborneWall.y>2.99f && airborneWall.verticalSpeed<0 &&
          airborneWall.impact>39 && airborneWall.speed<.001f && airborneWall.boostRemaining==0,
          "the existing side barrier removes outward air momentum and reward without canceling gravity");
    std::printf("vertical contact: crest air %.3fs, dip clearance %.3fm, flat landing %.2fm/s; gravity/reverse/bridge/replay/resize green\n",
                crestAir,dipClearance,impact);
}

void testSuzukaVertical() {
    Track track;
    bool loaded=false;
    for(const char* path : {"assets/track/suzuka.txt","../assets/track/suzuka.txt"})
        if(track.load(path)) {loaded=true;break;}
    check(loaded,"vertical course regression loads the repository's actual Suzuka elevation data");
    if(!loaded) return;
    for(int variant=0;variant<2;++variant) {
        Track course=track;
        const float scale=variant?2.0f:1.4f,top=variant?160.0f:120.0f;
        course.scaleBy(scale);
        Car car;car.reset(course);car.topSpeedKmh=top;
        DriveInput throttle;throttle.throttle=1;
        float maxGap=0,maxLanding=0,maxRise=0,takeoffY=0,longestFlight=0;
        int airFrames=0;
        for(int frame=0;frame<360;++frame) {
            const Car before=car;
            car.update(kDt,throttle,course);
            if(before.grounded&&!car.grounded) takeoffY=before.y;
            if(!car.grounded) maxRise=std::fmax(maxRise,car.y-takeoffY);
            longestFlight=std::fmax(longestFlight,car.airTime);
            maxGap=std::fmax(maxGap,car.y-car.groundHeight);
            maxLanding=std::fmax(maxLanding,car.landingImpact);
            airFrames+=!car.grounded;
            check(std::isfinite(car.y) && std::isfinite(car.verticalSpeed) && std::isfinite(car.ground.s) &&
                  car.y>=car.groundHeight-.001f,
                  "the original unsmoothed main-straight depression remains finite and cannot swallow the car");
        }
        check(airFrames>10 && car.landingSerial>0 && maxGap>1 && maxLanding>1 && maxRise<1.06f,
              "the unchanged Suzuka dip still jumps and lands while upward rise is bounded even at160kmh/stage2");
        std::printf("Suzuka original dip scale%.1f/%gkmh: airframes %d, longest %.3fs, landings %u, rise %.2fm, floor gap %.2fm, landing %.2fm/s\n",
                    scale,top,airFrames,longestFlight,car.landingSerial,maxRise,maxGap,maxLanding);
    }
}

void testAirborneReward() {
    const Track flat=straight();
    DriveInput release;release.throttle=1;
    Car boosted=slide(flat,1);
    boosted.topSpeedKmh=120; // Keep this regression above its selected normal cap.
    boosted.update(kDt,release,flat);run(boosted,flat,release,6);
    check(boosted.boostSpeedBonus>17.9f,"airborne reward fixture earns a real full boost before leaving ground");
    boosted.grounded=false;boosted.y=30;boosted.verticalSpeed=0;
    const float momentum=boosted.speed;
    for(int frame=0;frame<90;++frame) {
        boosted.update(kDt,release,flat);
        check(!boosted.grounded && boosted.speed==momentum && !boosted.driftHoldBlocked,
              "an earned boost may expire in flight without deleting horizontal momentum or falsely blocking its presentation");
    }
    check(boosted.boostRemaining==0 && boosted.boostSpeedBonus==0 && boosted.boostSerial==1,
          "flight advances and expires the existing boost clock without creating another reward");
    while(!boosted.grounded && boosted.airTime<5) boosted.update(kDt,release,flat);
    check(boosted.grounded && boosted.landingSerial==1 && boosted.speed==momentum,
          "landing retains the flight velocity before restoring ordinary ground limits");
    boosted.update(kDt,release,flat);
    check(boosted.speed<momentum && boosted.speed>momentum-1,
          "landing excess speed eases back to the normal cap instead of snapping off on the first grounded frame");
    run(boosted,flat,release,180);
    check(boosted.speed<=boosted.topSpeedKmh/3.6f+.01f,"landing cannot permanently bank expired reward speed");
    Car held=slide(flat,1);
    held.grounded=false;held.y=8;
    DriveInput hold;hold.throttle=hold.handbrake=1;hold.steer=-1;
    const float charge=held.driftCharge,slip=held.slipAngle,travel=held.travelYaw;
    run(held,flat,hold,15);
    check(held.driftCharge==charge && near(wrapPi(held.yaw-held.travelYaw),slip,.00001f) &&
          std::fabs(wrapPi(held.travelYaw-travel))>.05f,
          "an existing held slide keeps charge and drift angle while reduced air steering deliberately bends its line");
    held.update(kDt,release,flat);
    check(held.boostSerial==0 && held.driftCharge==0 && !held.driftHoldActive,
          "releasing Space in air consumes charge without granting a new ground-powered boost");
}

void testSuspensionAndAirControl() {
    const Track flat=straight();
    auto charged=[&](DriftModel model) {
        Car car=slide(flat,1,model);
        DriveInput hold;hold.throttle=hold.handbrake=1;
        run(car,flat,hold,120);
        car.x=car.z=0;car.travelYaw=0;car.yaw=car.slipAngle;
        car.ground=flat.query(0,0,0);
        return car;
    };
    int smallStepAirFrames=0;
    for(float height : {-.10f,-.05f,.05f,.10f}) for(DriftModel model : {DriftModel::Inertia,DriftModel::LineControl}) {
        Track steps;
        steps.closed=false;steps.halfWidth=1000;steps.length=100;
        for(int i=0;i<=1000;++i) {
            const float z=-20+i*.1f;
            steps.samples.push_back({0,z,z>=0&&z<3?height:0,0,z+20});
        }
        Car car=charged(model);
        car.z=-1;car.ground=steps.query(0,-1,0);
        Car replay=car;
        const float initialCharge=car.driftCharge;
        bool testedSupportedRelease=false;
        for(int frame=0;frame<75;++frame) {
            DriveInput hold;hold.throttle=hold.handbrake=1;hold.steer=frame<15?0:.5f;
            car.update(kDt,hold,steps);replay.update(kDt,hold,steps);
            smallStepAirFrames+=!car.grounded;
            check(car.grounded && car.driftHoldActive && !car.driftHoldBlocked && car.driftPhase==DriftPhase::Held &&
                  car.driftCharge==initialCharge && car.landingSerial==0 && car.y-car.groundHeight<=car.suspensionTravel+.001f,
                  "five/ten-centimetre up/down irregularities preserve contact, held drift and charge without landing spam");
            check(car.x==replay.x && car.y==replay.y && car.travelYaw==replay.travelYaw && car.grounded==replay.grounded,
                  "suspension-supported drift follows deterministic recorded controls through small steps");
            if(!testedSupportedRelease && car.y>car.groundHeight+.01f) {
                Car releasing=car;
                DriveInput release;release.throttle=1;release.hasBoostHeading=true;release.boostHeading=.2f;
                releasing.update(kDt,release,steps);
                check(releasing.grounded && releasing.boostSerial==1,
                      "releasing while the suspension still reaches a small depression awards the normal charged boost");
                testedSupportedRelease=true;
            }
        }
        check(car.driftTravelRate>.8f && car.travelYaw>.7f,
              "deliberate held steering continues to curve the line after a small step without Space being repressed");
    }
    const Track dip=verticalCourse(1);
    Car jumping=moving(dip,30);
    DriveInput none;
    float takeoffY=0,maxRise=0,maxLaunch=0;
    bool flying=false;
    for(int frame=0;frame<240;++frame) {
        const Car before=jumping;
        jumping.update(kDt,none,dip);
        if(before.grounded&&!jumping.grounded) {takeoffY=before.y;flying=true;maxLaunch=std::fmax(maxLaunch,jumping.verticalSpeed);}
        if(!jumping.grounded) maxRise=std::fmax(maxRise,jumping.y-takeoffY);
        check(jumping.verticalSpeed<=jumping.maxLaunchSpeed+.001f,
              "raw steep uphill support cannot inject more than the selected upward launch momentum");
    }
    check(flying && jumping.landingSerial>0 && maxRise<=4.5f*4.5f/(2*9.81f)+.03f,
          "large terrain still causes a real jump while its rise above takeoff stays within the bounded ballistic height");

    for(DriftModel model : {DriftModel::Inertia,DriftModel::LineControl}) {
        Car held=charged(model);
        held.grounded=false;held.y=.30f;held.verticalSpeed=0;
        const float charge=held.driftCharge,slip=held.slipAngle;
        DriveInput hold;hold.throttle=hold.handbrake=hold.steer=1;
        float airTurn=0;
        for(int frame=0;frame<60 && !held.grounded;++frame) {
            const float heading=held.travelYaw;
            held.update(kDt,hold,flat);
            airTurn+=wrapPi(held.travelYaw-heading);
            check(held.driftHoldActive && !held.driftHoldBlocked && held.driftCharge==charge &&
                  near(wrapPi(held.yaw-held.travelYaw),slip,.00001f) && held.boostSerial==0,
                  "a brief hop preserves held drift intent, charge and slip while reduced air control turns both headings together");
        }
        const float airRate=held.driftTravelRate;
        run(held,flat,hold,6);
        check(held.grounded && held.driftHoldActive && held.driftPhase==DriftPhase::Held &&
              held.driftTravelRate>airRate+.3f && airTurn>.03f && airRate<1.8f*.35f+.001f,
              "landing immediately restores the held ground steering response without another Space press");

        Car pressed=moving(flat,20);
        pressed.grounded=false;pressed.y=.15f;
        for(int frame=0;frame<30;++frame) pressed.update(kDt,hold,flat);
        check(pressed.grounded && pressed.driftHoldActive && !pressed.driftHoldBlocked && pressed.boostSerial==0,
              "a Space press made during a short hop is retained as landing intent without granting an airborne drift or boost");

        Car released=charged(model);
        released.grounded=false;released.y=.3f;
        DriveInput release;release.throttle=1;release.hasBoostHeading=true;release.boostHeading=.2f;
        run(released,flat,release,30);
        check(released.boostSerial==0 && released.driftCharge==0 && !released.driftHoldActive,
              "a genuine airborne release is consumed once and cannot become a delayed phantom boost after landing");

        Car easing=charged(model);
        easing.update(kDt,release,flat);
        easing.grounded=false;easing.y=.3f;
        const float oldElapsed=easing.releaseEaseElapsed;
        const float initialResidual=wrapPi(easing.travelYaw-easing.releaseHeading);
        release.steer=1;
        for(int frame=0;frame<60 && !easing.grounded;++frame) {
            easing.update(kDt,release,flat);
            check(near(easing.releaseEaseElapsed,oldElapsed) &&
                  near(wrapPi(easing.travelYaw-easing.releaseHeading),initialResidual,.00001f),
                  "air steering transports a pending camera release target with the actual line instead of leaving a stale world heading");
        }
        const float beforeResume=easing.travelYaw;
        easing.update(kDt,release,flat);
        check(easing.releaseEaseElapsed>oldElapsed && easing.boostSerial==1 &&
              std::fabs(wrapPi(easing.travelYaw-beforeResume))<.2f,
              "a pending direction ease resumes continuously after the hop without replaying the boost event");
    }
    Car live=charged(DriftModel::Inertia);
    live.grounded=false;live.y=4;
    const Car before=live;
    FeelSettings settings;
    settings.suspensionTravel=.4f;settings.maxLaunchSpeed=3;settings.airSteerControl=.7f;
    live.applyTuning(settings);
    check(live.y==before.y && live.verticalSpeed==before.verticalSpeed && live.yaw==before.yaw &&
          live.driftCharge==before.driftCharge && live.suspensionTravel==.4f && live.maxLaunchSpeed==3 && live.airSteerControl==.7f,
          "live suspension/launch/air-control edits preserve current pose, momentum and earned charge");
    settings.suspensionTravel=0;settings.maxLaunchSpeed=100;settings.airSteerControl=std::numeric_limits<float>::quiet_NaN();
    live.applyTuning(settings);live.reset(flat);
    check(live.suspensionTravel==.05f && live.maxLaunchSpeed==8 && near(live.airSteerControl,.35f) &&
          live.grounded && live.landingSerial==0,
          "new vertical parameters sanitize safely and reset preserves their selected values while clearing flight state");
    std::printf("controllable bumps: small-step airframes %d, big-dip maxlaunch %.3fm/s rise %.3fm (cap %.3fm), air turn restored\n",
                smallStepAirFrames,maxLaunch,maxRise,4.5f*4.5f/(2*9.81f));
}

void testGrassResistance() {
    Track road=straight(6.5f);
    road.curbWidth=.8f;road.runoff=8;
    auto onStrip=[&](float lateral,float speed) {
        Car car=moving(road,speed);
        car.x=lateral;
        car.ground=road.query(car.x,car.z,car.y);
        car.onAsphalt=std::fabs(lateral)<=road.halfWidth+.15f;
        return car;
    };
    DriveInput throttle;throttle.throttle=1;
    Car grass=onStrip(8,200/3.6f),replay=grass;
    float reached80=0,firstLoss=0;
    for(int frame=0;frame<300;++frame) {
        const float before=grass.speed;
        grass.update(kDt,throttle,road);replay.update(kDt,throttle,road);
        if(frame==0) firstLoss=(before-grass.speed)*3.6f;
        if(!reached80 && grass.speed<=80/3.6f+.0001f) reached80=(frame+1)*kDt;
        check(grass.speed<=before+.0001f && before-grass.speed<=13*kDt+.0001f && grass.speed>=80/3.6f-.001f,
              "full throttle on true green decelerates smoothly toward80kmh with no instantaneous cap or undershoot");
        check(grass.speed==replay.speed && grass.x==replay.x && grass.z==replay.z,
              "green resistance remains deterministic under the same recorded controls");
    }
    check(reached80>2.4f && reached80<2.8f && near(grass.speed,80/3.6f,.001f) && firstLoss<1,
          "entering green at200 reaches80 in about two-and-a-half seconds and then holds a low escape speed");
    Car entering=onStrip(6.4f,200/3.6f);
    entering.yaw=entering.travelYaw=1.57079632679f;
    entering.update(kDt,throttle,road);
    check(entering.x>road.halfWidth+road.curbWidth && near(entering.speed,200/3.6f,.001f),
          "crossing the visible curb edge does not teleport the entry speed to the grass limit");
    entering.update(kDt,throttle,road);
    check(near(entering.speed,200/3.6f-13*kDt,.001f),"grass resistance begins smoothly on the next contacted step");

    Car asphalt=onStrip(0,0),curb=onStrip(7,0),edge=onStrip(7.3f,0),escape=onStrip(8,0);
    for(int frame=0;frame<120;++frame) {
        asphalt.update(kDt,throttle,road);curb.update(kDt,throttle,road);
        edge.update(kDt,throttle,road);escape.update(kDt,throttle,road);
    }
    check(near(asphalt.speed,200/3.6f,.001f) && curb.speed==asphalt.speed && edge.speed==asphalt.speed &&
          escape.speed*3.6f>50 && escape.speed*3.6f<70,
          "painted curb keeps ordinary driving while actual green still allows a stopped car to accelerate out");
    run(escape,road,throttle,120);
    check(near(escape.speed,80/3.6f,.001f),"low-speed green escape settles at the reduced cruising speed despite W being held");
    Car resting=onStrip(8,0);
    run(resting,road,{},120);
    check(resting.speed==0,"green resistance cannot drive a stationary car backwards without input");
    Car lowCap=onStrip(8,0);lowCap.topSpeedKmh=60;
    run(lowCap,road,throttle,240);
    check(near(lowCap.speed,60/3.6f,.001f),"grass escape never exceeds a deliberately lower normal speed setting");
    Car braking=onStrip(8,80/3.6f);
    DriveInput brake;brake.brake=1;
    run(braking,road,brake,30);
    check(braking.speed>=0 && braking.speed<10,"braking still supplies stronger stopping force than green resistance");
    run(braking,road,brake,150);
    check(braking.speed<-8 && braking.speed>=-9,"holding S still permits bounded reverse while on green");
    run(braking,road,throttle,120);
    check(braking.speed>5,"W can stop reverse and move the car out of green instead of trapping it");

    Car flying=onStrip(8,200/3.6f);
    flying.grounded=false;flying.y=2;flying.verticalSpeed=0;
    const float momentum=flying.speed;
    for(int frame=0;frame<20;++frame) {
        flying.update(kDt,throttle,road);
        check(!flying.grounded && flying.speed==momentum,"flying over green preserves horizontal momentum without applying tyre resistance");
    }
    while(!flying.grounded && flying.airTime<2) flying.update(kDt,throttle,road);
    check(flying.grounded && flying.landingSerial==1 && flying.speed==momentum,
          "the flight lands on green before any grounded braking force is applied");
    flying.update(kDt,throttle,road);
    check(near(flying.speed,momentum-13*kDt,.001f),"the contacted green begins its smooth slowdown immediately after landing");

    const Track broad=straight();
    Car reward=slide(broad,1);
    reward.update(kDt,throttle,broad);run(reward,broad,throttle,6);
    reward.x=8;reward.z=0;reward.y=.3f;reward.grounded=false;
    reward.yaw=reward.travelYaw=0;reward.slipAngle=0;reward.releaseEaseActive=false;
    reward.ground=road.query(8,0,0);
    check(reward.boostSerial==1 && reward.boostRemaining>0,"green landing fixture begins with a genuine earned boost");
    for(int frame=0;frame<60&&!reward.grounded;++frame) reward.update(kDt,throttle,road);
    check(reward.grounded && !reward.onAsphalt && reward.boostRemaining==0 && reward.boostSpeedBonus==0 &&
          reward.boostSerial==1 && reward.driftCharge==0,
          "landing on green cancels the existing charge/boost once while preserving the airborne approach");
    const float landedSpeed=reward.speed;
    reward.update(kDt,throttle,road);
    check(reward.speed<landedSpeed && landedSpeed-reward.speed<1,
          "an earned-boost green landing continues into smooth physical slowdown rather than an80kmh clamp");
    FeelSettings invalid;invalid.topSpeedKmh=1000;
    reward.applyTuning(invalid);
    check(reward.topSpeedKmh==200,"live speed tuning cannot exceed the new200kmh ordinary limit");
    std::printf("green resistance: 200->80 %.3fs fullW, first loss %.3fkmh, curb unchanged, escape80, flight/landing/reverse green\n",
                reached80,firstLoss);
}

int main() {
    Track track = straight();
    testGrip(track);
    testSustainAndRecover(track);
    testArcadePhases(track);
    testLineControl(track);
    testFourTraceHold(track, DriftModel::Inertia);
    testFourTraceHold(track, DriftModel::LineControl);
    testStatefulHold(track, DriftModel::Inertia);
    testStatefulHold(track, DriftModel::LineControl);
    testGentleHoldSlowdown(track, DriftModel::Inertia);
    testGentleHoldSlowdown(track, DriftModel::LineControl);
    testEasedRelease(track, DriftModel::Inertia);
    testEasedRelease(track, DriftModel::LineControl);
    testFallbackRecovery(track, DriftModel::Inertia);
    testFallbackRecovery(track, DriftModel::LineControl);
    testChargeAndBoost(track);
    testChargeAndBoost(track, DriftModel::LineControl);
    testCancellation(track);
    testCancellation(track, DriftModel::LineControl);
    testPulseTopSpeedAndRehold(track, DriftModel::Inertia);
    testPulseTopSpeedAndRehold(track, DriftModel::LineControl);
    testWall();
    testCorner();
    testStress(track);
    testStress(track, DriftModel::LineControl);
    testRuntimeBoostParameters();
    testLiveTuning();
    testDriftLineSteering();
    testQuickPracticeSpeed();
    testLiveBoostFlow();
    testCameraDirectedRelease();
    testScaledCollision();
    testVerticalContact();
    testAirborneReward();
    testSuzukaVertical();
    testSuspensionAndAirControl();
    testGrassResistance();
    if (failures) return EXIT_FAILURE;
    std::puts("drift tests passed: manual hold/quick eased exit, four tire paths, charge/pulse, cancel/no farming, steering, fallback recovery, legacy track, walls, reverse, pause/replay/stress");
    return EXIT_SUCCESS;
}
