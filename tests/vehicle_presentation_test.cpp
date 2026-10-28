#include "vehicle_presentation.hpp"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

namespace {
constexpr float kPi = 3.14159265358979323846f;
constexpr float kDt = 1.0f / 60.0f;
int failures = 0;

void check(bool condition, const char* message) {
    if (!condition) { std::fprintf(stderr, "FAIL: %s\n", message); ++failures; }
}
bool near(float left, float right, float tolerance = 0.0001f) {
    return std::fabs(left - right) < tolerance;
}
float radians(float degrees) { return degrees * kPi / 180; }

Car moving(float speed = 24, float heading = 0) {
    Car car;
    car.speed = speed;
    car.yaw = car.travelYaw = heading;
    return car;
}

VehiclePresentation circle(int fps, float direction, float slip = 0, float start = 0) {
    VehiclePresentation view;
    FeelSettings settings;
    Car car = moving(24, start);
    car.yaw = wrapPi(car.travelYaw + slip);
    view.update(1.0f / fps, car, settings);
    for (int frame = 1; frame <= fps * 2; ++frame) {
        car.travelYaw = wrapPi(start + direction * 0.4f * float(frame) / fps);
        car.yaw = wrapPi(car.travelYaw + slip);
        view.update(1.0f / fps, car, settings);
    }
    return view;
}

void testActualLineAndSymmetry() {
    const auto right = circle(60, 1);
    const auto left = circle(60, -1);
    check(right.roll > radians(2) && right.roll < radians(4),
          "a right curved line visibly leans the sprung body towards the outside");
    check(near(right.roll, -left.roll) && near(right.pitch, left.pitch),
          "left/right trajectories produce mirrored roll and matching pitch");
    check(std::fabs(right.pitch) < radians(0.01f),
          "constant-speed grip cornering does not invent acceleration pitch");

    const auto sideways = circle(60, 1, radians(60));
    check(sideways.roll > 0 && sideways.roll < right.roll * 0.6f,
          "sideways motion projects lateral load into the body's actual right axis");
    const auto seam = circle(60, 1, 0, kPi - 0.3f);
    check(near(seam.roll, right.roll, 0.00002f) && std::fabs(seam.pitch) < radians(0.01f),
          "crossing the180degree seam causes no false body impulse");

    VehiclePresentation view;
    FeelSettings settings;
    Car car = moving();
    view.update(kDt, car, settings);
    for (int frame = 0; frame < 60; ++frame) {
        car.yaw = wrapPi(car.yaw + 0.08f);
        car.yawRate = 4.8f;
        car.slipAngle = wrapPi(car.yaw - car.travelYaw);
        car.driftHeld = car.drifting = true;
        view.update(kDt, car, settings);
    }
    check(view.roll == 0 && view.pitch == 0,
          "turning only the car body during a straight slide cannot create a cornering load");
}

void testAccelerationAndReturn() {
    VehiclePresentation accelerating, braking;
    FeelSettings settings;
    Car fast = moving(10), slow = moving(30);
    accelerating.update(kDt, fast, settings);
    braking.update(kDt, slow, settings);
    for (int frame = 0; frame < 30; ++frame) {
        fast.speed += 12 * kDt;
        slow.speed -= 12 * kDt;
        accelerating.update(kDt, fast, settings);
        braking.update(kDt, slow, settings);
    }
    check(accelerating.pitch > radians(1) && accelerating.pitch < radians(2),
          "forward acceleration lifts the nose with a bounded visual pitch");
    check(near(accelerating.pitch, -braking.pitch, 0.00002f) && accelerating.roll == 0,
          "equal braking pitches down without creating roll");
    const float pitchBefore = accelerating.pitch;
    for (int frame = 0; frame < 15; ++frame) accelerating.update(kDt, fast, settings);
    check(near(accelerating.pitch, pitchBefore * 0.1f, 0.00002f),
          "constant speed removes90percent of pitch over the configured250ms recovery");

    const Car frozenCar = fast;
    for (int frame = 0; frame < 180; ++frame) accelerating.update(kDt, frozenCar, settings);
    check(accelerating.pitch == 0 && accelerating.roll == 0,
          "a settled constant-speed car does not keep rocking");
}

void testTimeSteps() {
    const auto reference = circle(60, 1);
    for (int fps : {30, 120}) {
        const auto other = circle(fps, 1);
        check(near(other.roll, reference.roll, 0.00004f) && near(other.pitch, reference.pitch, 0.00004f),
              "30/60/120Hz trajectory samples give comparable visible body lean");
    }
    FeelSettings settings;
    VehiclePresentation fixed, variable;
    Car regular = moving(10), irregular = regular;
    fixed.update(kDt, regular, settings);
    variable.update(kDt, irregular, settings);
    for (int frame = 0; frame < 60; ++frame) {
        regular.speed += 10 * kDt;
        fixed.update(kDt, regular, settings);
    }
    const float steps[] = {0.005f, 0.027f, 0.011f, 0.023f, 0.014f, 0.020f};
    for (int cycle = 0; cycle < 10; ++cycle) for (float dt : steps) {
        irregular.speed += 10 * dt;
        variable.update(dt, irregular, settings);
    }
    check(near(fixed.pitch, variable.pitch, 0.00002f),
          "irregular timesteps preserve the response to the same sustained acceleration");
}

void testPauseResetAndLiveLimits() {
    FeelSettings settings;
    VehiclePresentation view = circle(60, 1);
    const float rollBefore = view.roll;
    Car car = moving(24, 0.8f);
    view.update(5, car, settings, false);
    check(view.roll == rollBefore, "pausing freezes the current visible lean");
    view.update(0, car, settings, false);
    const float pausedRoll = view.roll;
    view.update(kDt, car, settings);
    check(std::fabs(view.roll) < std::fabs(pausedRoll) && view.pitch == 0,
          "resuming an unchanged paused car relaxes its lean without an acceleration impulse");

    VehiclePresentation uninterrupted, renderObserved;
    car = moving(10);
    uninterrupted.update(kDt, car, settings);
    renderObserved.update(kDt, car, settings);
    for (int frame = 0; frame < 120; ++frame) {
        car.speed += 8 * kDt;
        car.yaw = car.travelYaw = wrapPi(float(frame) * 0.005f);
        // This zero-dt observation deliberately sees the new car sample before
        // the next simulation update. It must not consume that velocity delta.
        renderObserved.update(0, car, settings, false);
        uninterrupted.update(kDt, car, settings);
        renderObserved.update(kDt, car, settings);
        renderObserved.update(0, car, settings, false);
        check(uninterrupted.roll == renderObserved.roll && uninterrupted.pitch == renderObserved.pitch,
              "extra live-limit render observations never reset or consume fixed-step velocity history");
    }
    settings.bodyRollMax = 0;
    settings.bodyPitchMax = 0;
    view.pitch = radians(1);
    view.update(0, car, settings, false);
    check(view.roll == 0 && view.pitch == 0, "turning both live limits off is immediate even while paused");

    settings = FeelSettings{};
    view = circle(60, 1);
    settings.bodyRollMax = 0.5f;
    view.update(0, car, settings, false);
    check(std::fabs(view.roll) <= radians(0.5f), "reducing a live limit cannot leave the body beyond its new range");
    view.reset();
    check(view.roll == 0 && view.pitch == 0, "reset removes all visible tilt");
    car = moving(100, kPi);
    view.update(kDt, car, settings);
    check(view.roll == 0 && view.pitch == 0, "reset primes a new velocity without carrying an old impulse");
}

void testStopAndMalformedSamples() {
    FeelSettings settings;
    VehiclePresentation view = circle(60, 1);
    Car car = moving();
    car.speed = 0;
    view.update(kDt, car, settings);
    for (int frame = 0; frame < 180; ++frame) {
        car.yaw = wrapPi(car.yaw + 0.2f);
        car.travelYaw = wrapPi(car.travelYaw - 0.2f);
        view.update(kDt, car, settings);
    }
    check(view.roll == 0 && view.pitch == 0, "a stopped car settles without low-speed steering chatter");
    for (float dt : {0.0f, -1.0f, std::numeric_limits<float>::infinity(), std::numeric_limits<float>::quiet_NaN()}) {
        view.update(dt, car, settings);
        check(std::isfinite(view.roll) && std::isfinite(view.pitch), "invalid timestep cannot poison body angles");
    }
    car.speed = std::numeric_limits<float>::quiet_NaN();
    view.update(kDt, car, settings);
    car = moving(30, 1);
    view.update(kDt, car, settings);
    check(view.roll == 0 && view.pitch == 0, "invalid velocity is discarded and the next valid sample is freshly primed");
    car.speed = 1000;
    car.travelYaw = -1;
    view.update(0.00001f, car, settings);
    check(std::isfinite(view.roll) && std::isfinite(view.pitch) &&
          std::fabs(view.roll) <= radians(settings.bodyRollMax) && std::fabs(view.pitch) <= radians(settings.bodyPitchMax),
          "an abrupt velocity change stays finite and within the configured body limits");
}

void testPausedSpeedEditRebase() {
    FeelSettings settings;
    Car car = moving(30, 0.8f);
    car.yaw = wrapPi(car.travelYaw + radians(60));
    VehiclePresentation view;
    view.update(kDt, car, settings);
    // An existing suspension pose must survive the setting edit itself.
    view.roll = radians(1);
    view.pitch = radians(0.5f);
    const float beforeRoll = view.roll;
    const float beforePitch = view.pitch;
    settings.topSpeedKmh = 60;
    const float previousSpeed = car.speed;
    car.applyTuning(settings);
    check(car.speed < previousSpeed && near(car.speed, 60 / 3.6f),
          "paused test performs a real live top-speed edit that immediately changes Car speed");
    view.rebaseVelocity(car);
    view.update(0, car, settings, false);
    check(view.roll == beforeRoll && view.pitch == beforePitch,
          "rebasing an edited paused velocity preserves the currently visible body pose");
    view.update(kDt, car, settings);
    const float naturalReturn = std::exp(-2.302585093f * kDt / settings.bodyRecoveryTime);
    check(near(view.roll, beforeRoll * naturalReturn, 0.000001f) &&
          near(view.pitch, beforePitch * naturalReturn, 0.000001f),
          "resuming after a paused speed edit only relaxes the old pose and creates no false roll or pitch load");

    const float validRoll = view.roll;
    const float validPitch = view.pitch;
    car.travelYaw = std::numeric_limits<float>::quiet_NaN();
    view.rebaseVelocity(car);
    check(view.roll == validRoll && view.pitch == validPitch,
          "an invalid rebase request cannot poison or clear the visible suspension pose");
    car = moving(40, -2);
    view.update(kDt, car, settings);
    check(view.roll == validRoll && view.pitch == validPitch,
          "a valid sample after an invalid rebase is freshly primed without a false acceleration");
}

void releaseBoost(Car& car) {
    ++car.boostSerial;
    car.boostLevel = 3;
    car.boostRemaining = 1.3f;
}

void testBoostHeaveEvents() {
    FeelSettings settings;
    Car car = moving();
    car.boostSerial = 42;
    car.boostLevel = 3;
    car.boostRemaining = 1.3f;
    VehiclePresentation view;
    view.update(kDt, car, settings);
    for (int frame = 0; frame < 10; ++frame) view.update(kDt, car, settings);
    check(view.heave == 0, "joining an existing boost serial cannot replay a past body bounce");
    releaseBoost(car);
    view.update(kDt, car, settings);
    check(view.heave > 0 && view.heave < settings.bodyBounceHeight,
          "a new valid boost starts a small upward sprung-body response");
    float highest = view.heave, lowest = view.heave;
    for (int frame = 0; frame < 90; ++frame) {
        view.update(kDt, car, settings);
        highest = std::fmax(highest, view.heave);
        lowest = std::fmin(lowest, view.heave);
    }
    check(highest > settings.bodyBounceHeight * 0.9f && highest <= settings.bodyBounceHeight,
          "an isolated boost bounce reaches approximately the configured small height");
    check(lowest < 0 && std::fabs(lowest) < highest * 0.1f && view.heave == 0,
          "boost heave has one small opposite recoil and settles despite a sustained boost state");
    ++car.boostSerial;
    car.boostLevel = 0;
    car.boostRemaining = 0;
    view.update(kDt, car, settings);
    check(view.heave == 0, "an incremented event number without a valid boost cannot make the body hop");
}

void testZeroBoostStrengthAndBrakePriority() {
    FeelSettings settings;
    settings.boostSpeedScale = 0;
    Car car = moving();
    VehiclePresentation view;
    view.update(kDt, car, settings);
    releaseBoost(car);
    view.update(kDt, car, settings);
    check(view.heave == 0, "a reward serial with zero acceleration strength cannot trigger boost heave");
    settings.boostSpeedScale = 1;
    view.update(kDt, car, settings);
    check(view.heave == 0, "enabling boost strength does not replay a previously suppressed reward serial");
    releaseBoost(car);
    view.update(kDt, car, settings);
    check(view.heave > 0, "a later valid nonzero-strength reward still triggers boost heave");

    settings.boostSpeedScale = 0;
    car = moving();
    view.reset();
    view.update(kDt, car, settings);
    releaseBoost(car);
    car.speed -= 0.5f;
    view.update(kDt, car, settings, true, 1);
    check(view.heave < 0,
          "an invalid new boost serial does not suppress a simultaneous real braking compression");
}

void testBrakeHeaveEvents() {
    FeelSettings settings;
    Car car = moving();
    VehiclePresentation held, released;
    held.update(kDt, car, settings);
    released.update(kDt, car, settings);
    car.speed -= 0.5f;
    held.update(kDt, car, settings, true, 1);
    released.update(kDt, car, settings, true, 1);
    check(held.heave < 0, "a real brake onset with falling speed starts a small downward compression");
    float lowest = held.heave;
    for (int frame = 0; frame < 90; ++frame) {
        car.speed = std::fmax(5, car.speed - 0.1f);
        held.update(kDt, car, settings, true, 1);
        released.update(kDt, car, settings, true, 0);
        lowest = std::fmin(lowest, held.heave);
        check(near(held.heave, released.heave, 0.000001f),
              "holding a brake while decelerating does not repeatedly trigger compression");
    }
    check(std::fabs(lowest) > settings.bodyBounceHeight * 0.5f &&
          std::fabs(lowest) <= settings.bodyBounceHeight * 0.61f && held.heave == 0,
          "brake compression is about60percent of boost height and returns to rest");

    VehiclePresentation still, noBrake, changingDirection;
    car = moving(0);
    still.update(kDt, car, settings);
    for (int frame = 0; frame < 60; ++frame) still.update(kDt, car, settings, true, float(frame % 2));
    check(still.heave == 0, "pressing or tapping the brake while stationary cannot make the body bounce");
    car = moving();
    noBrake.update(kDt, car, settings);
    car.speed -= 1;
    noBrake.update(kDt, car, settings, true, 0);
    check(noBrake.heave == 0, "deceleration without a real brake onset cannot create a brake bounce");
    changingDirection.update(kDt, car, settings);
    car.travelYaw += 0.2f;
    car.yaw = 1;
    changingDirection.update(kDt, car, settings, true, 1);
    check(changingDirection.heave == 0,
          "a direction change at unchanged scalar speed is not a braking impulse even when the brake is pressed");
}

void testRepeatedEventsStayBounded() {
    FeelSettings settings;
    Car car = moving();
    VehiclePresentation tapping, single;
    tapping.update(kDt, car, settings);
    single.update(kDt, car, settings);
    car.speed -= 0.5f;
    tapping.update(kDt, car, settings, true, 1);
    single.update(kDt, car, settings, true, 1);
    for (int frame = 0; frame < 9; ++frame) {
        car.speed -= 0.1f;
        tapping.update(kDt, car, settings, true, float(frame % 2));
        single.update(kDt, car, settings, true, 0);
        check(near(tapping.heave, single.heave, 0.000001f),
              "rapid brake edges within the short cooldown cannot stack body kicks");
    }
    VehiclePresentation repeated;
    repeated.update(kDt, car, settings);
    constexpr float dt = 0.001f;
    const float speedBound = 24 * settings.bodyBounceHeight / settings.bodyBounceReturn;
    for (int frame = 0; frame < 600; ++frame) {
        if (frame % 4 == 0) releaseBoost(car);
        const float before = repeated.heave;
        repeated.update(dt, car, settings, true, float(frame % 2));
        check(std::isfinite(repeated.heave) && std::fabs(repeated.heave) <= settings.bodyBounceHeight,
              "even an artificial flood of real boost events cannot accumulate unbounded height");
        check(std::fabs(repeated.heave - before) <= speedBound * dt + 0.000001f,
              "repeated event responses keep the visible vertical movement speed bounded");
    }
}

void testHeavePauseZeroResetAndRebase() {
    FeelSettings settings;
    Car car = moving();
    VehiclePresentation view;
    view.update(kDt, car, settings);
    releaseBoost(car);
    view.update(kDt, car, settings);
    const float frozen = view.heave;
    view.update(10, car, settings, false, 1);
    view.update(0, car, settings, false, 1);
    check(view.heave == frozen, "pause and zero-dt observations freeze the suspension spring and its events");
    VehiclePresentation reference = view;
    const Car oldCar = car;
    car.speed = 50;
    view.rebaseVelocity(car);
    check(view.heave == frozen, "a live speed rebase preserves the existing heave pose");
    view.update(kDt, car, settings);
    reference.update(kDt, oldCar, settings);
    check(near(view.heave, reference.heave, 0.000001f),
          "resuming a rebased speed edit advances only the existing spring, without a false new event");
    settings.bodyBounceHeight = 0;
    view.update(0, car, settings, false);
    check(view.heave == 0, "setting bounce height to zero takes effect immediately while paused");
    settings.bodyBounceHeight = 0.035f;
    view.update(kDt, car, settings);
    check(view.heave == 0, "re-enabling height cannot restore an old hidden spring velocity or replay the active boost");
    releaseBoost(car);
    view.update(kDt, car, settings);
    check(view.heave > 0, "a later new boost can still trigger after re-enabling the bounce");
    view.reset();
    view.update(kDt, car, settings);
    check(view.heave == 0, "reset clears heave and primes an existing serial without replaying it");
}

std::array<float, 3> heaveAtLandmarks(int fps) {
    FeelSettings settings;
    Car car = moving();
    VehiclePresentation view;
    view.update(1.0f / fps, car, settings);
    releaseBoost(car);
    std::array<float, 3> result{};
    int next = 0;
    for (int frame = 1; frame <= fps * 3 / 10; ++frame) {
        view.update(1.0f / fps, car, settings);
        if (frame == fps * (next + 1) / 10) result[next++] = view.heave;
    }
    return result;
}
void testHeaveTimeSteps() {
    const auto reference = heaveAtLandmarks(60);
    for (int fps : {30, 120}) {
        const auto sample = heaveAtLandmarks(fps);
        for (int point = 0; point < 3; ++point)
            check(near(reference[point], sample[point], 0.000002f),
                  "the analytically damped spring follows the same100/200/300ms pose at30/60/120Hz");
    }
    FeelSettings settings;
    Car car = moving();
    VehiclePresentation variable;
    variable.update(kDt, car, settings);
    releaseBoost(car);
    const float steps[] = {0.005f, 0.027f, 0.011f, 0.023f, 0.014f, 0.020f};
    for (int point = 0; point < 3; ++point) {
        for (float dt : steps) variable.update(dt, car, settings);
        check(near(reference[point], variable.heave, 0.000002f),
              "irregular timestep splits preserve the analytic suspension response");
    }
    for (int frame = 0; frame < 120; ++frame) variable.update(kDt, car, settings);
    check(variable.heave == 0, "heave reaches exact rest after the configured finite return interval");
}

void testHeaveLiveDurationAndLimits() {
    FeelSettings beforeSettings;
    Car car = moving();
    VehiclePresentation before;
    before.update(kDt, car, beforeSettings);
    releaseBoost(car);
    before.update(0.03f, car, beforeSettings);
    for (float duration : {0.2f, 1.0f}) {
        FeelSettings afterSettings = beforeSettings;
        afterSettings.bodyBounceReturn = duration;
        VehiclePresentation edited = before;
        VehiclePresentation reference = before;
        const float poseBeforeEdit = edited.heave;
        edited.update(0, car, afterSettings, false);
        check(edited.heave == poseBeforeEdit,
              "a paused live spring duration edit preserves the visible height exactly");
        constexpr float dt = 0.02f;
        edited.update(dt, car, afterSettings);
        reference.update(dt * beforeSettings.bodyBounceReturn / duration, car, beforeSettings);
        check(near(edited.heave, reference.heave, 0.000002f),
              "changing spring duration preserves phase and re-times its existing velocity continuously");
        afterSettings.bodyBounceHeight = 0.08f;
        for (int frame = 0; frame < 120; ++frame) {
            edited.update(kDt, car, afterSettings);
            check(std::isfinite(edited.heave) && std::fabs(edited.heave) <= afterSettings.bodyBounceHeight,
                  "minimum and maximum spring durations remain finite within the8cm height limit");
        }
        check(edited.heave == 0, "both extreme duration settings finish at exact rest");
    }
}

void land(Car& car, float impact) {
    car.grounded = true;
    car.verticalSpeed = 0;
    car.landingImpact = impact;
    ++car.landingSerial;
}

void testLandingOnceAndRecovery() {
    FeelSettings settings;
    Car car = moving();
    VehiclePresentation heldImpact, clearedImpact;
    heldImpact.update(kDt, car, settings);
    clearedImpact.update(kDt, car, settings);
    car.grounded = false;
    car.verticalSpeed = -6;
    for (int frame = 0; frame < 15; ++frame) {
        heldImpact.update(kDt, car, settings);
        clearedImpact.update(kDt, car, settings);
    }
    check(heldImpact.landingCompression == 0, "airborne vertical velocity alone cannot create a landing compression");
    land(car, 6);
    heldImpact.update(kDt, car, settings);
    clearedImpact.update(kDt, car, settings);
    check(heldImpact.landingCompression < 0 && heldImpact.heave == 0,
          "a new grounded landing event compresses only its separate body suspension layer");
    float lowest = heldImpact.landingCompression;
    for (int frame = 0; frame < 90; ++frame) {
        Car clear = car;
        clear.landingImpact = 0;
        heldImpact.update(kDt, car, settings);
        clearedImpact.update(kDt, clear, settings);
        lowest = std::fmin(lowest, heldImpact.landingCompression);
        check(near(heldImpact.landingCompression, clearedImpact.landingCompression, 0.000001f),
              "holding the same landing serial and impact cannot retrigger or extend compression");
        check(heldImpact.landingCompression >= -0.100001f && heldImpact.landingCompression <= 0,
              "a hard landing stays within ten centimetres and never invents an upward physical jump");
    }
    check(lowest < -0.09f && heldImpact.landingCompression == 0,
          "a strong landing reaches a small visible compression then returns to exact rest");
}

float landingDepth(float impact, float scale) {
    FeelSettings settings;
    settings.landingImpactScale = scale;
    Car car = moving();
    VehiclePresentation view;
    view.update(kDt, car, settings);
    land(car, impact);
    float lowest = 0;
    for (int frame = 0; frame < 90; ++frame) {
        view.update(kDt, car, settings);
        lowest = std::fmin(lowest, view.landingCompression);
    }
    return -lowest;
}

void testLandingStrengthAndBounds() {
    const float weak = landingDepth(2, 1);
    const float strong = landingDepth(6, 1);
    check(weak > 0.02f && weak < 0.04f && strong > weak * 3 && strong <= .100001f,
          "landing impact controls compression strength rather than treating every touchdown as a heavy hit");
    check(near(landingDepth(6, .5f), strong * .5f, .00001f) &&
          near(landingDepth(6, 2), strong * 2, .00001f),
          "the authored landing scale changes depth predictably while retaining the same return timing");
    check(landingDepth(.5f, 1) == 0 && landingDepth(-5, 1) == 0 && landingDepth(6, 0) == 0,
          "tiny contact noise, invalid negative impact and a disabled layer cannot create a landing event");
    check(near(landingDepth(1000, 1), strong, .00001f), "extreme impact remains capped at the authored compression depth");
    check(landingDepth(std::numeric_limits<float>::quiet_NaN(), 1) == 0 &&
          landingDepth(std::numeric_limits<float>::infinity(), 1) == 0,
          "nonfinite impact cannot poison the landing response");

    FeelSettings settings;
    Car car = moving();
    VehiclePresentation repeated;
    repeated.update(kDt, car, settings);
    for (int frame = 0; frame < 2000; ++frame) {
        land(car, 100);
        repeated.update(.001f, car, settings);
        check(std::isfinite(repeated.landingCompression) && repeated.landingCompression >= -.100001f &&
              repeated.landingCompression <= 0,
              "an artificial flood of landing serials cannot accumulate an unbounded suspension offset");
    }
}

void testLandingPauseZeroResetAndRebase() {
    FeelSettings settings;
    Car car = moving();
    VehiclePresentation view;
    view.update(kDt, car, settings);
    land(car, 6);
    view.update(kDt, car, settings);
    const float frozen = view.landingCompression;
    VehiclePresentation reference = view;
    view.update(10, car, settings, false);
    view.update(0, car, settings, false);
    check(view.landingCompression == frozen, "pause and zero-duration observations freeze landing compression exactly");
    view.update(kDt, car, settings);
    reference.update(kDt, car, settings);
    check(view.landingCompression == reference.landingCompression,
          "resuming continues the same landing spring rather than restarting the hit");

    const float beforeRebase = view.landingCompression;
    reference = view;
    Car oldCar = car;
    car.speed = 50;
    land(car, 6);
    view.rebaseVelocity(car);
    check(view.landingCompression == beforeRebase, "rebasing a stage or tuning edit keeps the existing visible compression");
    view.update(kDt, car, settings);
    reference.update(kDt, oldCar, settings);
    check(near(view.landingCompression, reference.landingCompression, .000001f),
          "rebase consumes the current event number and cannot create a false touchdown from an edited pose");

    settings.landingImpactScale = 0;
    land(car, 6);
    view.update(0, car, settings, false);
    check(view.landingCompression == 0, "landing scale zero clears the visible compression immediately even while paused");
    settings.landingImpactScale = 1;
    view.update(kDt, car, settings);
    check(view.landingCompression == 0, "re-enabling a disabled layer cannot replay a landing hidden during pause");
    land(car, 6);
    view.update(kDt, car, settings);
    check(view.landingCompression < 0, "a genuinely new landing still works after re-enabling the layer");
    view.reset();
    view.update(kDt, car, settings);
    check(view.landingCompression == 0 && view.heave == 0,
          "reset clears both vertical presentation layers and primes an existing landing serial without replaying it");
    land(car, 6);
    view.update(3, car, settings, false);
    check(view.landingCompression == 0, "pause defers an unobserved new landing without advancing its envelope");
    view.update(kDt, car, settings);
    check(view.landingCompression < 0, "resume consumes the deferred landing once");
}

void testAirborneGroundEventsAreSuppressed() {
    FeelSettings settings;
    Car car = moving();
    VehiclePresentation view;
    view.update(kDt, car, settings);
    car.grounded = false;
    releaseBoost(car);
    view.update(kDt, car, settings);
    check(view.heave == 0, "an airborne boost event cannot replay a ground suspension kick");
    car.grounded = true;
    view.update(kDt, car, settings);
    check(view.heave == 0, "landing cannot replay a boost serial already consumed while airborne");
    car.grounded = false;
    car.speed -= 1;
    view.update(kDt, car, settings, true, 1);
    check(view.heave == 0, "airborne braking cannot invent contact compression");
    car.grounded = true;
    car.speed -= 1;
    view.update(kDt, car, settings, true, 1);
    check(view.heave == 0, "a held brake does not replay its airborne edge on touchdown");
    car.grounded = false;
    land(car, 6);
    car.grounded = false;
    view.update(kDt, car, settings);
    car.grounded = true;
    view.update(kDt, car, settings);
    check(view.landingCompression == 0, "a malformed airborne landing serial is consumed without replaying it later");
}

void testLandingTimingAndLiveEdits() {
    FeelSettings settings;
    Car car = moving();
    std::array<std::array<float, 3>, 3> traces{};
    const int rates[] = {30, 60, 120};
    for (int index = 0; index < 3; ++index) {
        VehiclePresentation view;
        view.update(1.0f / rates[index], car, settings);
        Car event = car;
        land(event, 6);
        int next = 0;
        for (int frame = 1; frame <= rates[index] * 3 / 10; ++frame) {
            view.update(1.0f / rates[index], event, settings);
            if (frame == rates[index] * (next + 1) / 10) traces[index][next++] = view.landingCompression;
        }
    }
    for (int index : {0, 2}) for (int point = 0; point < 3; ++point)
        check(near(traces[index][point], traces[1][point], .000002f),
              "landing compression has comparable100/200/300ms poses at30/60/120Hz");

    VehiclePresentation original;
    original.update(kDt, car, settings);
    land(car, 6);
    original.update(.03f, car, settings);
    for (float duration : {.2f, 1.0f}) {
        VehiclePresentation edited = original, reference = original;
        FeelSettings changed = settings;
        changed.bodyBounceReturn = duration;
        const float before = edited.landingCompression;
        edited.update(0, car, changed, false);
        check(edited.landingCompression == before, "a paused landing duration edit preserves the visible pose");
        edited.update(.02f, car, changed);
        reference.update(.02f * settings.bodyBounceReturn / duration, car, settings);
        check(near(edited.landingCompression, reference.landingCompression, .000002f),
              "a landing duration edit re-times the existing response continuously instead of restarting the impact");
        for (int frame = 0; frame < 180; ++frame) edited.update(kDt, car, changed);
        check(edited.landingCompression == 0, "minimum and maximum return times both end at exact rest");
    }
}

std::array<float, 28> physicalSnapshot(const Car& car) {
    return {car.x, car.y, car.z, car.speed, car.yaw, car.travelYaw, car.yawRate, car.slipAngle,
            car.driftTravelRate, car.driftCharge, float(car.driftChargeLevel), car.boostRemaining,
            car.boostSpeedBonus, float(car.boostSerial), car.steerAngle, car.wheelSpin,
            float(car.drifting), float(car.driftHeld), float(car.onAsphalt), car.impact,
            float(car.grounded), car.verticalSpeed, car.groundHeight, car.airTime, car.terrainPitch,
            car.terrainRoll, car.landingImpact, float(car.landingSerial)};
}
void testPhysicalIndependence() {
    Track track;
    track.makePracticeArea();
    Car withPresentation, withoutPresentation;
    withPresentation.reset(track);
    withoutPresentation = withPresentation;
    VehiclePresentation view;
    FeelSettings settings;
    DriveInput input;
    input.throttle = 1;
    for (int frame = 0; frame < 600; ++frame) {
        input.steer = frame < 100 ? 0 : frame < 300 ? 0.65f : -0.35f;
        input.handbrake = frame >= 120 && frame < 300 ? 1 : 0;
        withPresentation.update(kDt, input, track);
        withoutPresentation.update(kDt, input, track);
        view.update(kDt, withPresentation, settings, true, input.brake);
        check(physicalSnapshot(withPresentation) == physicalSnapshot(withoutPresentation),
              "observing a real throttle/drift/release run leaves the car's trajectory and reward unchanged");
    }
}
} // namespace

int main() {
    testActualLineAndSymmetry();
    testAccelerationAndReturn();
    testTimeSteps();
    testPauseResetAndLiveLimits();
    testStopAndMalformedSamples();
    testPausedSpeedEditRebase();
    testBoostHeaveEvents();
    testZeroBoostStrengthAndBrakePriority();
    testBrakeHeaveEvents();
    testRepeatedEventsStayBounded();
    testHeavePauseZeroResetAndRebase();
    testHeaveTimeSteps();
    testHeaveLiveDurationAndLimits();
    testLandingOnceAndRecovery();
    testLandingStrengthAndBounds();
    testLandingPauseZeroResetAndRebase();
    testAirborneGroundEventsAreSuppressed();
    testLandingTimingAndLiveEdits();
    testPhysicalIndependence();
    if (failures) return EXIT_FAILURE;
    std::puts("vehicle presentation tests passed: velocity loads, event heave, landing compression, airborne suppression, damped timing, pause/reset/live limits and physical independence");
    return EXIT_SUCCESS;
}
