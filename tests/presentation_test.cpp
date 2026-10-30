#include <algorithm>
#include "presentation.hpp"

#include <cmath>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <vector>

namespace {
constexpr float kDt = 1.0f / 60.0f;
constexpr float kPi = 3.14159265359f;
int failures = 0;
// Explicit no-hold fixtures retain coverage of the earlier lens/dolly curves.
// New default holds and normal steering preview are exercised separately below.
FeelSettings baselineSettings() {
    FeelSettings result;
    result.boostFovReturn = 1;
    result.boostFovHold = result.boostSetbackHold = 0;
    result.cameraSteerLookAhead = 0;
    return result;
}

void check(bool condition, const char* message) {
    if (!condition) { std::fprintf(stderr, "FAIL: %s\n", message); ++failures; }
}
bool near(float a, float b, float tolerance = 0.0001f) { return std::fabs(a - b) < tolerance; }
void boost(Car& car, unsigned serial = 1, int level = 3) {
    car.boostSerial = serial;
    car.boostLevel = level;
    car.boostRemaining = 0.65f;
    car.boostSpeedBonus = level == 3 ? 18 : (level == 2 ? 14 : 10);
    car.speed = 22 + car.boostSpeedBonus;
}
void run(CameraPresentation& view, float seconds, float dt, const Car& car, const FeelSettings& settings) {
    const int frames = int(std::lround(seconds / dt));
    for (int frame = 0; frame < frames; ++frame) view.update(dt, car, settings);
}

struct ScreenBox { float left = 1, right = 0, top = 1, bottom = 0; };
struct ScreenPoint { float x, y, depth; };
ScreenPoint projectedPoint(const CameraPresentation& view, Vec3 point) {
    const Mat4 clip = mul(perspective(view.fovDegrees * kPi / 180, 16.0f / 9.0f, 0.15f, 4000),
                          lookAt(view.eye, view.look, {0, 1, 0}));
    const float depth = clip.m[3] * point.x + clip.m[7] * point.y + clip.m[11] * point.z + clip.m[15];
    return {0.5f + 0.5f * (clip.m[0] * point.x + clip.m[4] * point.y + clip.m[8] * point.z + clip.m[12]) / depth,
            0.5f - 0.5f * (clip.m[1] * point.x + clip.m[5] * point.y + clip.m[9] * point.z + clip.m[13]) / depth,
            depth};
}
ScreenBox projectedCar(const CameraPresentation& view, const Car& car) {
    // Geometry fixture from the rendered block car, excluding shadow and glow.
    // This checks visible composition rather than claiming the reference vehicle's dimensions.
    std::vector<Vec3> points;
    auto box = [&](Vec3 lo, Vec3 hi) {
        for (float x : {lo.x, hi.x}) for (float y : {lo.y, hi.y}) for (float z : {lo.z, hi.z})
            points.push_back({x, y, z});
    };
    box({-0.88f, 0.32f, -2.05f}, {0.88f, 0.62f, 2.05f});
    box({-0.72f, 0.62f, -0.55f}, {0.72f, 1.05f, 1.05f});
    box({-0.62f, 0.78f, -0.35f}, {0.62f, 1.02f, 0.9f});
    box({-0.7f, 0.32f, 1.7f}, {0.7f, 0.5f, 2.25f});
    box({-0.95f, 0.26f, 1.95f}, {0.95f, 0.36f, 2.35f});
    box({-0.86f, 0.85f, -2.2f}, {0.86f, 1.0f, -1.85f});
    box({-0.15f, 0.95f, -2.05f}, {0.15f, 1.15f, -1.9f});
    for (float x : {-0.82f, 0.82f}) for (float z : {-1.38f, 1.42f})
        for (float side : {-1.0f, 1.0f}) for (int i = 0; i < 14; ++i) {
            const float angle = i * 2 * kPi / 14;
            points.push_back({x + side * 0.15f, 0.34f + 0.34f * std::cos(angle), z + 0.34f * std::sin(angle)});
        }
    const Mat4 model = mul(translation(car.x, car.y, car.z), rotationY(car.yaw));
    const Mat4 camera = mul(perspective(view.fovDegrees * kPi / 180, 16.0f / 9.0f, 0.15f, 4000),
                            lookAt(view.eye, view.look, {0, 1, 0}));
    const Mat4 clip = mul(camera, model);
    ScreenBox result;
    for (Vec3 p : points) {
        const float w = clip.m[3] * p.x + clip.m[7] * p.y + clip.m[11] * p.z + clip.m[15];
        const float x = 0.5f + 0.5f * (clip.m[0] * p.x + clip.m[4] * p.y + clip.m[8] * p.z + clip.m[12]) / w;
        const float y = 0.5f - 0.5f * (clip.m[1] * p.x + clip.m[5] * p.y + clip.m[9] * p.z + clip.m[13]) / w;
        result.left = std::fmin(result.left, x); result.right = std::fmax(result.right, x);
        result.top = std::fmin(result.top, y); result.bottom = std::fmax(result.bottom, y);
    }
    return result;
}

void testReferenceComposition() {
    FeelSettings settings = baselineSettings();
    Car car;
    car.yaw = 30 * kPi / 180;
    car.slipAngle = car.yaw / settings.cameraTravelWeight;
    car.travelYaw = car.yaw - car.slipAngle;
    car.speed = 22;
    CameraPresentation view;
    view.update(0, car, settings);
    const ScreenBox framed = projectedCar(view, car);
    const float width = framed.right - framed.left, height = framed.bottom - framed.top;
    check(near(width, 0.386f, 0.02f) && near(height, 0.359f, 0.02f) &&
          near((framed.top + framed.bottom) * 0.5f, 0.617f, 0.02f) && near(framed.bottom, 0.796f, 0.02f),
          "the current car projects near the reference09 size and vertical composition at a comparable rear three-quarter pose");
    car.speed = 82;
    for (int frame = 0; frame < 180; ++frame) {
        car.x += std::sin(car.travelYaw) * car.speed * kDt;
        car.z += std::cos(car.travelYaw) * car.speed * kDt;
        view.update(kDt, car, settings);
    }
    const Vec3 forward{std::sin(view.heading), 0, std::cos(view.heading)};
    const Vec3 right{std::cos(view.heading), 0, -std::sin(view.heading)};
    const Vec3 offset = Vec3{car.x, car.y, car.z} - view.anchor;
    const ScreenBox fast = projectedCar(view, car);
    std::printf("reference framing: width %.3f height %.3f centreY %.3f bottom %.3f; fast slide width %.3f height %.3f lateral %.3fm\n",
                width, height, (framed.top + framed.bottom) * 0.5f, framed.bottom,
                fast.right - fast.left, fast.bottom - fast.top, dot(offset, right));
    check(std::fabs(dot(offset, forward)) < 0.0001f && std::fabs(dot(offset, right)) <= 0.7501f &&
          near(dot(Vec3{car.x, car.y, car.z} - view.eye, forward), 5, 0.0001f),
          "steady fast sliding cannot add chase distance and its sideways composition stays bounded");
    check(near(fast.bottom - fast.top, height, 0.005f) &&
          fast.right - fast.left >= width * 0.95f && fast.right - fast.left <= width * 1.20f &&
          fast.left > 0 && fast.right < 1,
          "high-speed camera damping keeps the fitted car size instead of shrinking it through longitudinal lag");
    Car straight;
    straight.speed = 82;
    CameraPresentation straightView;
    straightView.update(0, straight, settings);
    const ScreenBox stopped = projectedCar(straightView, straight);
    for (int frame = 0; frame < 180; ++frame) {
        straight.z += straight.speed * kDt;
        straightView.update(kDt, straight, settings);
    }
    const ScreenBox moving = projectedCar(straightView, straight);
    check(near(moving.right - moving.left, stopped.right - stopped.left) &&
          near(moving.bottom - moving.top, stopped.bottom - stopped.top),
          "straight running preserves exactly the static reference framing at speed");
}

void testFraming() {
    FeelSettings settings = baselineSettings();
    Car car;
    car.speed = 22;
    car.yaw = car.slipAngle = kPi / 3;
    CameraPresentation view;
    view.update(0, car, settings, false);
    check(view.ready && near(view.heading, 0.28f * car.slipAngle),
          "inactive preview initializes the existing travel/body camera mix");
    check(near(wrapPi(car.yaw - view.heading), 43.2f * kPi / 180),
          "a sustained sideways car remains visibly angled across the camera");
    check(near(std::hypot(view.eye.x - view.anchor.x, view.eye.z - view.anchor.z), 5.0f) &&
          near(view.eye.y - view.anchor.y, 2.05f) && near(view.look.y - view.anchor.y, 0.17f) &&
          near(view.fovDegrees, 60) && view.boostAmount == 0,
          "default framing uses the fitted close view without an invented startup lens cue");
    settings.cameraTravelWeight = 0.9f;
    view.ready = false;
    view.update(0, car, settings, false);
    check(near(view.heading, 0.1f * car.slipAngle), "camera comparison weight is still honored");
}

void testSeparateResponses() {
    FeelSettings settings = baselineSettings();
    Car car;
    CameraPresentation ordinary, oneStep;
    ordinary.update(0, car, settings);
    oneStep = ordinary;
    car.yaw = 1;
    run(ordinary, 0.15f, kDt, car, settings);
    oneStep.update(0.15f, car, settings);
    check(near(1 - ordinary.heading, std::pow(10.0f, -0.15f / 0.33f)) &&
          near(ordinary.heading, oneStep.heading),
          "ordinary fixed-target damping retains its baseline response at different frame rates");
    Car transverse;
    CameraPresentation damping;
    damping.update(0, transverse, settings);
    transverse.x = 0.5f;
    run(damping, 0.15f, kDt, transverse, settings);
    check(near(0.5f - damping.anchor.x, 0.5f * std::pow(10.0f, -0.15f / 0.20f)),
          "small transverse movement retains its independent position damping");
    transverse.x = 10;
    transverse.z = 20;
    damping.update(kDt, transverse, settings);
    check(near(transverse.z - damping.anchor.z, 0) &&
          near(transverse.x - damping.anchor.x, 0.75f),
          "longitudinal composition follows directly and transverse lag stays bounded");
    Car crossing;
    crossing.yaw = 179 * kPi / 180;
    CameraPresentation wrapped;
    wrapped.update(0, crossing, settings);
    crossing.yaw = -179 * kPi / 180;
    wrapped.update(kDt, crossing, settings);
    check(std::fabs(wrapped.heading) > 178 * kPi / 180,
          "camera heading crosses the world-angle seam along the short arc");
}

void setHeld(Car& car, float slip = kPi / 3) {
    car.driftHoldActive = car.driftHeld = car.drifting = true;
    car.driftPhase = DriftPhase::Held;
    car.yaw = car.slipAngle = slip;
    car.travelYaw = 0;
    car.driftAge = 1;
    car.speed = 22;
    car.releaseEaseActive = false;
}

void testBoostDirectionPreviewProjection() {
    FeelSettings settings = baselineSettings();
    for (float degrees : {45.0f, 60.0f}) for (float side : {-1.0f, 1.0f}) {
        Car car;
        setHeld(car, side * degrees * kPi / 180);
        CameraPresentation preview;
        preview.update(0, car, settings);
        CameraPresentation unpreviewed = preview;
        unpreviewed.boostPreview = 0;
        // Preserve a sampled lagged orbit for one tiny substep. The look target is
        // compared at the same eye pose, rather than conflating aim and chase position.
        for (float lag : {0.0f, 12.0f}) {
            CameraPresentation shown = preview, previous = unpreviewed;
            shown.heading -= side * lag * kPi / 180;
            previous.heading = shown.heading;
            shown.update(0.000001f, car, settings);
            previous.update(0.000001f, car, settings);
            check(length(shown.eye - previous.eye) < 0.0001f && near(shown.fovDegrees, previous.fovDegrees),
                  "boost anticipation changes the looked-at corridor without moving the eye or lens");
            const float originalGap = std::fabs(wrapPi(car.yaw - shown.heading));
            const float visibleGap = std::fabs(wrapPi(car.yaw - shown.lookHeading));
            check(visibleGap > originalGap * 0.65f && visibleGap < originalGap * 0.98f,
                  "the view includes a little nose direction while remaining primarily across the slide");
            for (float distance : {15.0f, 25.0f}) {
                const Vec3 nose{std::sin(car.yaw), 0, std::cos(car.yaw)};
                const Vec3 right{std::cos(car.yaw), 0, -std::sin(car.yaw)};
                const Vec3 ahead = nose * distance;
                const ScreenPoint now = projectedPoint(shown, ahead);
                const ScreenPoint before = projectedPoint(previous, ahead);
                check(std::fabs(now.x - 0.5f) < std::fabs(before.x - 0.5f) - 0.025f,
                      "the near and far boost directions move visibly inward from the edge");
                for (float width : {-1.5f, 0.0f, 1.5f}) {
                    const ScreenPoint corridor = projectedPoint(shown, ahead + right * width);
                    check(corridor.depth > 0 && corridor.x > 0.015f && corridor.x < 0.985f &&
                          corridor.y > 0 && corridor.y < 1,
                          "a three-metre boost corridor stays visible at15m and25m, including a lagged curved line");
                }
                const ScreenPoint travel = projectedPoint(shown, {0, 0, distance});
                check(travel.depth > 0 && travel.x > 0.15f && travel.x < 0.85f && travel.y > 0 && travel.y < 1,
                      "the current sliding line remains visible beside the anticipated boost line");
                if (side > 0 && degrees == 60 && distance == 25)
                    std::printf("boost preview: lag %.0fdeg nose25m x %.3f -> %.3f; current line %.3f; optical advance %.2fdeg\n",
                                lag, before.x, now.x, travel.x,
                                wrapPi(shown.lookHeading - shown.heading) * 180 / kPi);
            }
            const ScreenBox body = projectedCar(shown, car);
            check(body.left > 0 && body.right < 1 && body.top > 0 && body.bottom < 1,
                  "anticipating the boost direction keeps the whole sideways car in view");
        }
    }
    Car rightCar, leftCar;
    setHeld(rightCar);
    setHeld(leftCar, -kPi / 3);
    CameraPresentation right, left;
    right.update(0, rightCar, settings);
    left.update(0, leftCar, settings);
    const ScreenPoint rightNose = projectedPoint(right, {25 * std::sin(rightCar.yaw), 0, 25 * std::cos(rightCar.yaw)});
    const ScreenPoint leftNose = projectedPoint(left, {25 * std::sin(leftCar.yaw), 0, 25 * std::cos(leftCar.yaw)});
    check(near(rightNose.x + leftNose.x, 1) && near(rightNose.y, leftNose.y),
          "the anticipated corridor mirrors exactly for left and right drifts");
    rightCar.yaw += 179 * kPi / 180;
    rightCar.travelYaw += 179 * kPi / 180;
    right.ready = false;
    right.update(0, rightCar, settings);
    check(std::fabs(wrapPi(right.lookHeading - right.heading)) < 12 * kPi / 180,
          "boost anticipation takes a bounded short arc across the world-angle seam");
}

void testBoostDirectionPreviewTransitions() {
    FeelSettings settings = baselineSettings();
    Car car;
    CameraPresentation view;
    view.update(0, car, settings);
    check(view.boostPreview == 0 && near(view.lookHeading, view.heading),
          "ordinary running has no residual boost anticipation");
    setHeld(car);
    car.driftPhase = DriftPhase::Entry;
    car.driftAge = 0.01f;
    view.update(kDt, car, settings);
    check(view.boostPreview > 0 && view.boostPreview < 0.25f && view.lookHeading > view.heading,
          "nose anticipation starts gently while the entry orbit still lets the body turn first");
    view.update(0.18f, car, settings);
    check(view.boostPreview > 0.85f && view.boostPreview < 1,
          "the anticipated boost corridor becomes readable while the drift is held");
    const CameraPresentation frozen = view;
    car.driftHoldActive = car.driftHeld = false;
    car.driftPhase = DriftPhase::None;
    car.releaseEaseActive = true;
    view.update(3, car, settings, false);
    check(view.boostPreview == frozen.boostPreview && view.lookHeading == frozen.lookHeading,
          "pause freezes both the anticipated aim and its blend clock");
    view.update(kDt, car, settings);
    check(view.boostPreview >= frozen.boostPreview && view.boostPreview <= 1,
          "manual release keeps the anticipated exit visible while its slower orbit catches up");
    car.releaseEaseActive = car.drifting = false;
    car.slipAngle = 0;
    car.travelYaw = car.yaw;
    run(view, 1.3f, kDt, car, settings);
    check(view.boostPreview < 0.005f && near(view.lookHeading, view.heading),
          "preview finishes naturally as the release camera settles behind the nose");
    setHeld(car);
    view.ready = false;
    view.update(0, car, settings, false);
    check(near(view.boostPreview, 1) && view.lookHeading > view.heading,
          "priming a held pose starts with the current intended view, without an old blend tail");
    car.driftHoldActive = car.driftHeld = false;
    car.driftPhase = DriftPhase::None;
    view.update(kDt, car, settings);
    check(view.boostPreview > 0 && view.boostPreview < 1 && view.motionPhase == CameraMotionPhase::Recovery,
          "a cancelled drift fades anticipation through its separate recovery");
    car = Car{};
    view.ready = false;
    view.update(0, car, settings, false);
    check(view.boostPreview == 0 && near(view.lookHeading, view.heading),
          "reset cannot carry a drift's anticipated direction into straight running");
}

void testChargeLensAndTranslation() {
    FeelSettings settings = baselineSettings();
    Car car;
    setHeld(car);
    Car quietCar = car;
    CameraPresentation charged, quiet;
    charged.update(0, car, settings);
    quiet.update(0, quietCar, settings);
    car.driftCharge = 2.20f;
    float maxHorizontal = 0, maxVertical = 0;
    for (int frame = 0; frame < 120; ++frame) {
        charged.update(kDt, car, settings);
        quiet.update(kDt, quietCar, settings);
        check(charged.fovDegrees >= 59 && charged.fovDegrees <= 60 && charged.boostSetback == 0,
              "charge narrows the lens by at most one degree without any charge dolly");
        const Vec3 right{std::cos(charged.heading), 0, -std::sin(charged.heading)};
        const float horizontal = std::fabs(dot(charged.chargeShake, right));
        maxHorizontal = std::fmax(maxHorizontal, horizontal);
        maxVertical = std::fmax(maxVertical, std::fabs(charged.chargeShake.y));
        check(horizontal <= 0.00601f && std::fabs(charged.chargeShake.y) <= 0.00401f &&
              length((charged.look - charged.eye) - (quiet.look - quiet.eye)) < 0.00001f,
              "charge shake is a bounded common translation with no pitch, yaw or roll shake");
        check(length((charged.eye - charged.chargeShake) - quiet.eye) < 0.00001f &&
              length((charged.look - charged.chargeShake) - quiet.look) < 0.00001f,
              "both the eye and target receive exactly the same small charge movement");
        const ScreenBox body = projectedCar(charged, car);
        const ScreenPoint future = projectedPoint(charged, {25 * std::sin(car.yaw), 0, 25 * std::cos(car.yaw)});
        check(body.left > 0 && body.right < 1 && body.top > 0 && body.bottom < 1 &&
              future.x > 0.05f && future.x < 0.95f && future.y > 0 && future.y < 1,
              "the charged view retains the sideways car and its future boost direction");
    }
    check(near(charged.fovDegrees, 59, 0.001f) && maxHorizontal > 0.005f && maxVertical > 0.003f,
          "a capped charge holds a quiet steady zoom rather than repeating a zoom pulse");
    const CameraPresentation frozen = charged;
    charged.update(5, car, settings, false);
    check(charged.chargeAmount == frozen.chargeAmount && charged.fovDegrees == frozen.fovDegrees &&
          length(charged.eye - frozen.eye) == 0 && length(charged.look - frozen.look) == 0,
          "pause freezes the lens, charge amplitude and tiny shake at the exact current pose");
    car.driftHoldActive = false;
    const float beforeRelease = charged.chargeAmount;
    charged.update(kDt, car, settings);
    check(charged.chargeAmount > 0 && charged.chargeAmount < beforeRelease && charged.boostViewAmount == 0,
          "cancelling a charge fades its buildup without creating a release kick");
    car.speed = 0;
    charged.update(kDt, car, settings);
    check(charged.chargeAmount == 0 && length(charged.chargeShake) == 0 && near(charged.fovDegrees, 60),
          "a stopped car is neutral even if a stale charge number is still present");
    for (bool held : {false, true}) {
        car = Car{};
        if (held) setHeld(car);
        car.speed = held ? 0 : 22;
        car.driftCharge = 2.20f;
        charged.ready = false;
        charged.update(0, car, settings);
        run(charged, 0.5f, kDt, car, settings);
        check(charged.chargeAmount == 0 && near(charged.fovDegrees, 60) && charged.boostSetback == 0,
              "charge metadata alone cannot create camera effects without a moving held drift");
    }
    car = Car{};
    charged.ready = false;
    charged.update(0, car, settings, false);
    check(charged.chargeAmount == 0 && length(charged.chargeShake) == 0 && charged.boostViewAmount == 0,
          "reset removes both charge shake and any old release envelope");
}

void testChargeToBoostAndFinish() {
    FeelSettings settings = baselineSettings();
    Car car;
    setHeld(car);
    car.driftCharge = 2.20f;
    CameraPresentation view;
    view.update(0, car, settings);
    run(view, 1, kDt, car, settings);
    const float chargedFov = view.fovDegrees;
    car.driftHoldActive = car.driftHeld = false;
    car.driftPhase = DriftPhase::None;
    car.releaseEaseActive = true;
    car.driftCharge = 0;
    boost(car, 1, 3);
    view.update(kDt, car, settings);
    check(view.fovDegrees > chargedFov && view.fovDegrees < 63 && view.chargeAmount > 0 && view.boostTriggered,
          "release opens from the charged lens through a short attack instead of jumping instantly from59to65");
    run(view, 2 * kDt, kDt, car, settings);
    check(near(view.boostViewAmount,.5f) && view.boostFovOffset > 2.4f && view.boostFovOffset < 2.6f && near(view.boostSetback,.125f),
          "the camera dolly and lens are halfway open at50ms while the separate jet has reached its peak");
    run(view, 0.05f, kDt, car, settings);
    check(view.boostFovAmount > 0.99f && view.fovDegrees > 64.8f && view.fovDegrees <= 65 && near(view.boostSetback,.25f),
          "the lens and independent dolly reach their bounded peaks at one hundred milliseconds");
    const float peak = view.boostFovOffset;
    run(view, 0.20f, kDt, car, settings);
    check(view.boostFovOffset > peak * 0.94f && view.boostFovOffset < peak,
          "the lens remains visibly wide at three hundred milliseconds with a gentle initial return");
    run(view, 0.05f, kDt, car, settings);
    check(view.boostAmount == 0 && view.boostViewAmount>.8f && view.boostSetback>.2f && view.fovDegrees > 64.4f,
          "the jet ends by350ms while the independent camera dolly and wide lens continue returning");
    run(view, 0.75f, kDt, car, settings);
    check(view.boostFovAmount == 0 && near(view.fovDegrees, 60, 0.01f),
          "the lens completes its smooth one-second return after the hundred-millisecond attack");
    boost(car, 2, 3);
    run(view, 0.10f, kDt, car, settings);
    const Vec3 kickedEye = view.eye;
    view.clearBoostCue();
    const Vec3 forward{std::sin(view.heading), 0, std::cos(view.heading)};
    check(view.boostViewAmount == 0 && view.boostSetback == 0 && view.chargeAmount == 0 &&
          length(view.chargeShake) == 0 && near(view.fovDegrees, 60) &&
          near(dot(view.anchor - view.eye, forward), 5, 0.001f) && length(view.eye - kickedEye) > 0.20f,
          "finishing restores the undecorated eye and lens immediately before inactive rendering freezes the pose");
    const Vec3 finishedEye = view.eye;
    view.update(1, car, settings, false);
    check(length(view.eye - finishedEye) == 0 && view.boostViewAmount == 0,
          "the final frozen frame cannot retain the release setback or charge jitter");
    view.update(kDt, car, settings);
    check(!view.boostTriggered && view.boostViewAmount == 0,
          "clearing the presentation cannot replay an already observed boost serial");
    boost(car, 3, 3);
    run(view, 0.05f, kDt, car, settings);
    const float beforeCancel = view.boostFovOffset;
    car.boostRemaining = car.boostSpeedBonus = 0;
    car.boostLevel = 0;
    view.update(kDt, car, settings);
    check(!view.boostTriggered && view.boostFovOffset > 0 && view.boostFovOffset < beforeCancel,
          "early boost cancellation eases down from the current lens rather than causing a second attack");
    for (unsigned serial : {4u, 5u}) {
        boost(car, serial, 3);
        view.update(kDt, car, settings);
        check(view.boostTriggered && view.boostViewAmount >= 0 && view.boostViewAmount <= 1 &&
              view.fovDegrees <= 65 && view.boostSetback <= 0.25001f,
              "a newer real release replaces the active view cue without stacking lens or camera travel");
    }
    car.boostSerial = 6;
    car.boostLevel = 4;
    view.update(kDt, car, settings);
    check(!view.boostTriggered, "invalid boost tiers never arm a release camera kick");
    car.boostRemaining = car.boostSpeedBonus = 0;
    car.boostLevel = 0;
    run(view, 0.8f, kDt, car, settings);
    check(view.boostViewAmount == 0 && near(view.fovDegrees, 60),
          "cancelled and invalid events cannot leave an indefinite lens or setback tail");
}

void testEntryAndManualExit() {
    FeelSettings settings = baselineSettings();
    Car car;
    CameraPresentation view;
    view.update(0, car, settings);
    setHeld(car);
    car.driftPhase = DriftPhase::Entry;
    car.driftAge = 0.01f;
    view.update(0.06f, car, settings);
    check(view.motionPhase == CameraMotionPhase::Entry && near(view.heading, 0),
          "entry leaves the initial view in place while the car visibly rotates first");
    car.driftPhase = DriftPhase::Held;
    view.update(0.18f, car, settings);
    const float heldTarget = car.yaw - car.slipAngle * settings.cameraTravelWeight;
    check(view.motionPhase == CameraMotionPhase::Entry && near(view.heading, heldTarget * 0.5f),
          "entry uses a finite five-order eased catch-up instead of immediate ordinary damping");
    view.update(0.18f, car, settings);
    check(view.motionPhase == CameraMotionPhase::Held && near(view.heading, heldTarget),
          "entry finishes at the live mixed direction without an additional damping layer");
    car.driftHoldActive = car.driftHeld = false;
    car.driftPhase = DriftPhase::None;
    car.releaseEaseActive = true;
    car.releaseEaseElapsed = 0.1f;
    car.slipAngle = kPi / 180;
    const float from = view.heading;
    view.update(0.1f, car, settings);
    check(view.motionPhase == CameraMotionPhase::Exit && view.boostAmount == 0 &&
          view.heading > from + (car.yaw - from) * 0.40f && view.heading < car.yaw,
          "uncharged release catches up promptly in its first tenth without completing the whole turn");
    const float firstMove = view.heading - from;
    car.releaseEaseActive = car.drifting = false;
    car.slipAngle = 0;
    view.update(0.3f, car, settings);
    const float secondMove = view.heading - from - firstMove;
    check(view.motionPhase == CameraMotionPhase::Exit && secondMove > 0 &&
          near(view.heading, from + (car.yaw - from) * (1 - std::pow(1 - 0.4f / settings.cameraExitTime, settings.cameraExitPower))),
          "the return follows the configured front-loaded curve and leaves a small soft final approach");
    view.update(settings.cameraExitTime - 0.4f, car, settings);
    check(view.motionPhase == CameraMotionPhase::Grip && near(view.heading, car.yaw),
          "the finite exit settles exactly behind the live nose at its duration");
    const float settled = view.heading;
    run(view, 0.5f, kDt, car, settings);
    check(near(view.heading, settled), "settled release has no lingering tail or repeated restart");
}

void testInterruptionsAndPhasePause() {
    FeelSettings settings = baselineSettings();
    Car car;
    setHeld(car);
    CameraPresentation view;
    view.update(0, car, settings);
    car.driftHoldActive = car.driftHeld = false;
    car.driftPhase = DriftPhase::None;
    car.slipAngle *= 0.8f;
    view.update(kDt, car, settings);
    check(view.motionPhase == CameraMotionPhase::Recovery && !view.boostTriggered,
          "brake or collision cancellation is a separate eased recovery, never manual release");
    const float cancelledPose = view.heading;
    setHeld(car);
    car.driftPhase = DriftPhase::Entry;
    car.driftAge = 0.01f;
    view.update(kDt, car, settings);
    check(view.motionPhase == CameraMotionPhase::Entry && near(view.heading, cancelledPose),
          "redrift interrupts recovery continuously from the camera's current pose");
    view.update(0.2f, car, settings);
    const float pausedHeading = view.heading, pausedElapsed = view.motionElapsed;
    car.driftHoldActive = car.driftHeld = false;
    car.driftPhase = DriftPhase::None;
    car.releaseEaseActive = true;
    view.update(3, car, settings, false);
    check(view.motionPhase == CameraMotionPhase::Entry && near(view.motionElapsed, pausedElapsed) &&
          near(view.heading, pausedHeading), "pause freezes the motion clock and defers phase changes");
    view.update(kDt, car, settings);
    check(view.motionPhase == CameraMotionPhase::Exit && view.heading > pausedHeading,
          "resume consumes the pending uncharged release exactly once");
    view.update(0.08f, car, settings);
    const float exitPose = view.heading;
    setHeld(car, 50 * kPi / 180);
    car.driftPhase = DriftPhase::Entry;
    car.driftAge = 0.01f;
    view.update(kDt, car, settings);
    check(view.motionPhase == CameraMotionPhase::Entry && near(view.heading, exitPose),
          "immediate redrift never jumps back to the old entry or exit origin");
    view.ready = false;
    view.update(0, car, settings, false);
    check(view.motionPhase == CameraMotionPhase::Held && view.motionElapsed == 0 &&
          near(view.heading, car.yaw - car.slipAngle * settings.cameraTravelWeight),
          "reset primes a live held pose without replaying an old entry animation");
    Car crossing;
    crossing.yaw = 179 * kPi / 180;
    CameraPresentation wrapped;
    wrapped.update(0, crossing, settings);
    crossing.releaseEaseActive = true;
    crossing.yaw = -179 * kPi / 180;
    wrapped.update(0.15f, crossing, settings);
    check(std::fabs(wrapped.heading) > 178 * kPi / 180,
          "manual exit also follows the short arc across the angle seam");
}

Track temporalTrack() {
    Track track;
    track.halfWidth = 1000;
    track.curbWidth = track.runoff = 0;
    track.length = 30000;
    track.samples = {{0, -5000, 0, 0, 0}, {0, 5000, 0, 0, 10000},
                     {5000, 5000, 0, 0, 15000}, {5000, -5000, 0, 0, 25000}};
    return track;
}

void testSlowLensTimingAndReboost() {
    FeelSettings settings = baselineSettings();
    Car car;
    car.speed = 22;
    CameraPresentation view;
    view.update(0, car, settings);
    boost(car);
    run(view, 0.1f, kDt, car, settings);
    check(near(view.boostFovOffset, 5), "lens opens fully over the requested tenth of a second");
    const float peak = view.boostFovOffset;
    view.update(0.01f, car, settings);
    check(view.boostFovOffset <= peak && peak - view.boostFovOffset < 0.001f,
          "return starts with a gentle slope instead of the previous immediate fast reversal");
    view.update(0.19f, car, settings);
    check(view.boostFovOffset > 4.7f && view.boostFovOffset < 4.8f,
          "wide view remains present while the road flows after the short launch kick");
    view.update(0.30f, car, settings);
    check(near(view.boostFovOffset, 2.5f, 0.001f) && view.boostSetback>0 && view.boostSetback<.1f && view.boostAmount == 0,
          "half of the FOV emphasis remains at600ms while the camera dolly is returning and the separate jet is over");
    view.update(0.30f, car, settings);
    check(view.boostFovOffset > 0.28f && view.boostFovOffset < 0.30f,
          "the final part of the one-second return is small and smooth");
    view.update(0.19f, car, settings);
    const float almostSettled = view.boostFovOffset;
    view.update(0.01f, car, settings);
    check(view.boostFovOffset == 0 && almostSettled < 0.001f,
          "the lens reaches base without a visible final step");

    boost(car, 2, 3);
    run(view, 0.1f, kDt, car, settings);
    const float oldFov = view.boostFovOffset;
    boost(car, 3, 1);
    view.update(kDt, car, settings);
    check(view.boostFovOffset < oldFov && view.boostFovOffset > oldFov - 0.25f,
          "a weaker reboost starts at the current absolute lens value instead of snapping from5to3degrees");
    run(view, 5 * kDt, kDt, car, settings);
    check(near(view.boostFovOffset, 3), "weaker reboost reaches its own bounded target without stacking");
    boost(car, 4, 3);
    view.update(kDt, car, settings);
    check(view.boostFovOffset > 3 && view.boostFovOffset < 3.25f,
          "a stronger reboost opens continuously from the previous lens value");
    run(view, 5 * kDt, kDt, car, settings);
    check(near(view.boostFovOffset, 5), "stronger reboost remains capped at five degrees");
}

void testNaturalBoostExpiryAndSlowLook() {
    const Track track = temporalTrack();
    const FeelSettings settings;
    for (int level = 1; level <= 3; ++level) {
        Car car;
        car.reset(track);
        car.speed = 80 / 3.6f;
        car.boostDurationScale = .5f; // A short real reward must not clip either default camera tail.
        CameraPresentation view;
        view.update(0, car, settings);
        DriveInput input;
        input.throttle = input.handbrake = 1;
        input.steer = 0.6f;
        for (int frame = 0; frame < 300 && car.driftChargeLevel < level; ++frame) {
            car.update(kDt, input, track);
            view.update(kDt, car, settings);
        }
        check(car.driftChargeLevel == level, "real test reaches the intended charge tier");
        input.handbrake = input.steer = 0;
        bool sawNaturalTail = false, sawPositionTail = false;
        for (int frame = 1; frame <= 126; ++frame) {
            car.update(kDt, input, track);
            view.update(kDt, car, settings);
            if (car.boostLevel == 0 && frame < 126 && view.boostFovOffset > 0) sawNaturalTail = true;
            if (car.boostLevel == 0 && frame<66 && view.boostSetback>0) sawPositionTail=true;
            if (frame == 6) check(near(view.boostFovOffset, 2.0f + level, 0.001f),
                                  "real boost lens reaches its tier peak at100ms");
            if (frame == 36) check(view.boostFovOffset > (2.0f + level) * 0.49f,
                                   "real physical expiry cannot truncate the midpoint of the lens return");
            if (frame == 18) check(near(view.boostSetback,.25f*(2+level)/5,.001f) && near(view.boostFovOffset,2.0f+level,.001f),
                                   "the real-car lens and position hold their tier peak through300ms");
            if (frame == 42) check(near(view.boostSetback,.25f*(2+level)/5*.5f,.001f),
                                   "all real physical boost tiers retain the independent dolly midpoint at700ms");
            if (frame == 66) check(view.boostSetback==0,
                                   "the independent real-car position return ends at1100ms for every tier");
        }
        check(sawNaturalTail && sawPositionTail && view.boostFovAmount == 0 && car.boostLevel == 0,
              "all short physical boost tiers leave the requested finite lens tail after natural expiry");
    }

    Car car;
    setHeld(car);
    CameraPresentation look;
    look.update(0, car, settings);
    const float from = look.heading;
    float previousLook = look.lookHeading;
    car.driftHeld = car.driftHoldActive = false;
    car.driftPhase = DriftPhase::None;
    car.releaseEaseActive = true;
    for (int frame = 1; frame <= 48; ++frame) {
        if (frame >= 9) { car.releaseEaseActive = car.drifting = false; car.slipAngle = 0; }
        look.update(kDt, car, settings);
        check(wrapPi(look.lookHeading - previousLook) >= -0.00001f,
              "the exit optical aim never first moves away from the anticipated boost direction");
        previousLook = look.lookHeading;
        if (frame == 18) check(look.motionPhase == CameraMotionPhase::Exit && look.heading > from + (car.yaw - from) * 0.85f,
                               "the new front-loaded return is mostly aligned at300ms but still finishing smoothly");
    }
    check(look.motionPhase == CameraMotionPhase::Grip && near(look.heading, car.yaw),
          "the configured optical return finishes behind the nose without a backward aim twitch");
}

void testActualChargeBuildup() {
    const Track track = temporalTrack();
    FeelSettings settings = baselineSettings();
    for (DriftModel model : {DriftModel::Inertia, DriftModel::LineControl}) {
        Car car;
        car.driftModel = model;
        car.reset(track);
        car.speed = 80 / 3.6f;
        CameraPresentation view;
        view.update(0, car, settings);
        float previousFov = view.fovDegrees;
        for (int frame = 1; frame <= 210; ++frame) {
            DriveInput input;
            input.throttle = input.handbrake = 1;
            input.steer = 0.6f;
            car.update(kDt, input, track);
            view.update(kDt, car, settings);
            check(view.fovDegrees <= previousFov + 0.0001f && previousFov - view.fovDegrees < 0.025f,
                  "actual continuous charge builds lens emphasis smoothly across discrete charge tiers");
            check(view.fovDegrees >= 59 && view.boostSetback == 0 && !view.boostTriggered,
                  "actual held charging never creates release travel or a false boost event");
            previousFov = view.fovDegrees;
            if (frame >= 60) {
                const Vec3 position{car.x, car.y, car.z};
                const Vec3 nose{std::sin(car.yaw), 0, std::cos(car.yaw)};
                const ScreenPoint future = projectedPoint(view, position + nose * 25);
                check(future.depth > 0 && future.x > 0.02f && future.x < 0.98f && future.y > 0 && future.y < 1,
                      "a fully charging curved line still shows the future boost direction");
            }
        }
        check(car.driftChargeLevel == 3 && near(car.driftCharge, 2.20f) && near(view.fovDegrees, 59, 0.002f),
              "actual capped charge holds the one-degree lens limit without cycling or accumulating zoom");
    }
}

struct MotionTrace {
    std::array<float, 11> headings{}, body{}, looks{}, previews{}, fovs{}, setbacks{};
    float maxStep = 0, maxLookStep = 0;
    float highestFov = 0;
    bool sawEntry = false, sawHeld = false, sawExit = false, sawBoost = false;
};
MotionTrace actualTrace(int fps, DriftModel model, float side, bool charged = true) {
    const Track track = temporalTrack();
    Car car;
    car.driftModel = model;
    car.reset(track);
    car.speed = 80 / 3.6f;
    FeelSettings settings = baselineSettings();
    CameraPresentation view;
    view.update(0, car, settings);
    MotionTrace trace;
    const int presentationTicks = 120 / fps;
    const int releaseTick = charged ? 144 : 36;
    const std::array<int, 11> landmarks{12, 24, 48, 60, 144, 156, 168, 180, 216, 252, 276};
    for (int tick = 1; tick <= landmarks.back(); ++tick) {
        // The game simulates the car at60Hz; the presentation may render twice,
        // once or every other physics step without changing the physical path.
        if (tick % 2 == 0) {
            DriveInput input;
            input.throttle = 1;
            if (tick <= releaseTick) { input.steer = side * 0.6f; input.handbrake = 1; }
            car.update(1.0f / 60, input, track);
        }
        if (tick % presentationTicks == 0) {
            const float before = view.heading;
            const float beforeLook = view.lookHeading;
            view.update(float(presentationTicks) / 120, car, settings);
            trace.maxStep = std::fmax(trace.maxStep, std::fabs(wrapPi(view.heading - before)));
            trace.maxLookStep = std::fmax(trace.maxLookStep, std::fabs(wrapPi(view.lookHeading - beforeLook)));
            trace.sawEntry |= view.motionPhase == CameraMotionPhase::Entry;
            trace.sawHeld |= view.motionPhase == CameraMotionPhase::Held;
            trace.sawExit |= view.motionPhase == CameraMotionPhase::Exit;
            trace.sawBoost |= car.boostSerial != 0;
            trace.highestFov = std::fmax(trace.highestFov, view.fovDegrees);
            const ScreenBox screen = projectedCar(view, car);
            check(screen.left > -0.01f && screen.right < 1.01f && screen.top > 0 && screen.bottom < 1,
                  "the real entry/exit path keeps the full car in the reference framing");
        }
        for (size_t i = 0; i < landmarks.size(); ++i) if (tick == landmarks[i]) {
            trace.headings[i] = view.heading;
            trace.body[i] = car.yaw;
            trace.looks[i] = view.lookHeading;
            trace.previews[i] = view.boostPreview;
            trace.fovs[i] = view.fovDegrees;
            trace.setbacks[i] = view.boostSetback;
        }
    }
    check(trace.sawEntry && trace.sawExit && (!charged || trace.sawHeld),
          "real car transitions exercise entry, held and both charged/uncharged manual exit");
    check(trace.sawBoost == charged, "charged and uncharged actual releases remain distinct from the camera motion event");
    check(trace.maxStep < 15 * kPi / 180, "no displayed transition frame produces a hard heading jump");
    check(trace.maxLookStep < 15 * kPi / 180, "the anticipatory aim remains continuous through actual drift and release");
    return trace;
}

void testActualCarFrameRates() {
    for (DriftModel model : {DriftModel::Inertia, DriftModel::LineControl}) {
        const MotionTrace baseline = actualTrace(120, model, 1);
        const MotionTrace mirrored = actualTrace(120, model, -1);
        for (size_t i = 0; i < baseline.headings.size(); ++i)
            check(std::fabs(wrapPi(baseline.headings[i] + mirrored.headings[i])) < 0.001f &&
                  std::fabs(wrapPi(baseline.looks[i] + mirrored.looks[i])) < 0.001f,
                  "real left and right entry/exit motion is symmetric");
        for (int fps : {30, 60}) {
            const MotionTrace trace = actualTrace(fps, model, 1);
            for (size_t i = 0; i < baseline.headings.size(); ++i)
                check(std::fabs(wrapPi(trace.headings[i] - baseline.headings[i])) < 2 * kPi / 180 &&
                      std::fabs(wrapPi(trace.looks[i] - baseline.looks[i])) < 2 * kPi / 180 &&
                      std::fabs(trace.previews[i] - baseline.previews[i]) < 0.12f &&
                      std::fabs(trace.fovs[i] - baseline.fovs[i]) < 0.50f &&
                      std::fabs(trace.setbacks[i] - baseline.setbacks[i]) < 0.03f,
                      "real60Hz physics produces comparable camera landmarks at30/60/120Hz presentation");
        }
        const MotionTrace uncharged = actualTrace(60, model, 1, false);
        check(uncharged.sawExit && uncharged.highestFov <= 60.0001f,
              "short actual drift gets the eased exit without a fake boost lens kick");
        std::printf("motion %s: entry camera/body %.1f/%.1fdeg at.1s, exit camera/body %.1f/%.1fdeg at+.1s; max30Hz step %.1fdeg\n",
                    model == DriftModel::Inertia ? "A" : "B", baseline.headings[0] * 180 / kPi,
                    baseline.body[0] * 180 / kPi, baseline.headings[5] * 180 / kPi,
                    baseline.body[5] * 180 / kPi, actualTrace(30, model, 1).maxStep * 180 / kPi);
    }
    const Track track = temporalTrack();
    Car cancelling;
    cancelling.reset(track);
    cancelling.speed = 80 / 3.6f;
    CameraPresentation view;
    FeelSettings settings = baselineSettings();
    view.update(0, cancelling, settings);
    DriveInput input;
    input.throttle = input.handbrake = input.steer = 1;
    for (int frame = 0; frame < 60; ++frame) {
        cancelling.update(kDt, input, track);
        view.update(kDt, cancelling, settings);
    }
    input.brake = 1;
    cancelling.update(kDt, input, track);
    view.update(kDt, cancelling, settings);
    check(!cancelling.driftHoldActive && !cancelling.releaseEaseActive &&
          view.motionPhase == CameraMotionPhase::Recovery && !view.boostTriggered,
          "an actual brake cancellation selects recovery while the drift button remains held");
}

void testEventAndLookahead() {
    FeelSettings settings = baselineSettings();
    Car car;
    car.speed = 22;
    CameraPresentation view;
    view.update(0, car, settings);
    const float initialAhead = view.look.z - view.anchor.z;
    car.driftChargeLevel = 3;
    run(view, 0.1f, kDt, car, settings);
    check(view.fovDegrees == 60 && view.boostAmount == 0,
          "charge or ordinary speed alone does not create a boost cue");
    CameraPresentation fast = view;
    boost(car);
    view.update(kDt, car, settings);
    check(view.boostTriggered && view.boostEventSerial == 1 && view.boostAmount > 0 &&
          view.fovDegrees > 60 && view.fovDegrees <= 65 && view.boostSetback > 0,
          "one real release begins one bounded lens/setback kick together with its jet cue");
    check(near(view.look.z - view.anchor.z, initialAhead),
          "a boost cannot change the fitted lookahead");
    view.update(kDt, car, settings);
    check(!view.boostTriggered, "repeated active-state updates cannot retrigger the same release");
    view.update(kDt, car, settings);
    run(fast, 0.05f, 1.0f / 240, car, settings);
    check(near(view.fovDegrees, 62.5f) && near(view.boostSetback, 0.125f) &&
          near(view.boostAmount, 1) && near(view.boostAmount, fast.boostAmount),
          "the dolly is halfway out at fifty milliseconds while the independent jet is at its original peak");
    run(view, 0.05f, kDt, car, settings);
    run(fast, 0.05f, 1.0f / 240, car, settings);
    check(near(view.fovDegrees, 65) && near(view.boostFovAmount, fast.boostFovAmount) &&
          near(view.boostAmount, fast.boostAmount) && view.boostAmount > 0 && view.boostAmount < 1,
          "the lens peaks at100ms at60Hz and240Hz while the short jet has already begun relaxing");
    run(view, 0.25f, kDt, car, settings);
    check(view.boostAmount == 0 && view.fovDegrees > 64.4f,
          "the short pulse finishes without truncating the slower lens tail");
    run(view, 0.75f, kDt, car, settings);
    check(view.boostFovAmount == 0 && near(view.fovDegrees, 60),
          "the slow lens is finite even if an unchanged synthetic physics boost remains active");
    for (int level = 1; level <= 3; ++level) {
        boost(car, unsigned(level + 1), level);
        run(view, 0.10f, kDt, car, settings);
        check(near(view.fovDegrees, 62 + float(level)) && view.boostAmount > 0 && view.boostAmount < 1,
              "real charge tiers get increasing bounded release emphasis");
        run(view, 1.0f, kDt, car, settings);
    }
    settings.fovGain = 6;
    car.speed = 100;
    boost(car, 5);
    car.speed = 100;
    run(view, 0.10f, kDt, car, settings);
    check(near(view.fovDegrees, 71), "the existing absolute-speed FOV comparison remains separate from the capped release increment");
}

void testPauseCancelAndReset() {
    FeelSettings settings = baselineSettings();
    Car car;
    car.speed = 22;
    CameraPresentation view;
    view.update(0, car, settings, false);
    boost(car);
    run(view, 0.05f, kDt, car, settings);
    const CameraPresentation frozen = view;
    boost(car, 2, 1);
    car.x = 100;
    view.update(3, car, settings, false);
    view.update(0, car, settings);
    view.update(std::numeric_limits<float>::infinity(), car, settings);
    check(view.fovDegrees == frozen.fovDegrees && view.boostAmount == frozen.boostAmount &&
          view.anchor.x == frozen.anchor.x && view.boostEventSerial == 1,
          "pause and invalid elapsed time freeze the view without consuming a pending release");
    view.update(kDt, car, settings);
    check(view.boostTriggered && view.boostEventSerial == 2,
          "resume consumes a release deferred during pause exactly once");
    view.update(kDt, car, settings);
    check(!view.boostTriggered, "resume does not repeatedly restart its pending cue");
    car.boostRemaining = car.boostSpeedBonus = 0;
    car.boostLevel = 0;
    const float beforeCancel = view.boostAmount;
    view.update(kDt, car, settings);
    check(view.boostAmount > 0 && view.boostAmount < beforeCancel && !view.boostTriggered,
          "early cancellation returns from its current visual strength without a second attack or hard cut");
    run(view, 0.30f, kDt, car, settings);
    check(view.boostAmount == 0 && view.fovDegrees == 60, "cancelled cue has a finite tail");
    boost(car, 3);
    run(view, 0.05f, kDt, car, settings);
    view.clearBoostCue();
    view.update(kDt, car, settings);
    check(view.boostAmount == 0 && view.fovDegrees == 60 && !view.boostTriggered,
          "ending a run clears the cue without rearming its existing serial");
    view.ready = false;
    view.update(kDt, car, settings, false);
    check(view.boostAmount == 0 && view.fovDegrees == 60 && view.anchor.x == car.x,
          "reset re-primes the current pose without replaying an old release");
    car.boostSerial = 0;
    car.boostLevel = 0;
    car.boostRemaining = car.boostSpeedBonus = 0;
    view.ready = false;
    view.update(0, car, settings, false);
    boost(car, 1);
    view.update(kDt, car, settings);
    check(view.boostTriggered && view.boostEventSerial == 1,
          "a replay whose serial restarts at zero can issue one fresh release cue");
}
} // namespace

void testRuntimeParameters() {
    const FeelSettings defaults;
    check(near(defaults.cameraExitTime, 0.55f) && near(defaults.cameraExitPower, 3) &&
          near(defaults.boostFovReturn, 1.8f), "new default tuning gives a prompt soft-ending orbit and a longer lens tail");
    check(near(defaults.driftSteerRate, 1.8f) && near(defaults.driftSteerResponse, .12f),
          "default drift line controls expose the stronger turn rate and quick response");
    check(near(defaults.topSpeedKmh, 200) && near(defaults.accelerationTime, 1) &&
          near(defaults.stageScale, 1.4f) && near(defaults.releaseDirectionTime, .25f),
          "default practice reaches200kmh with a larger stage and a250ms launch direction transition");
    check(near(defaults.boostSpeedAttack,.1f) && near(defaults.boostSpeedHold,.2f) && near(defaults.boostSpeedReturn,.8f) &&
          near(defaults.boostSetbackHold,.2f) && near(defaults.boostFovHold,.2f) &&
          near(defaults.cameraSteerLookAhead,.2f) && near(defaults.cameraSteerLookMax,8),
          "default speed and camera holds align and normal steering preview remains small");
    struct Bound { float FeelSettings::*field; float low, high; };
    const Bound bounds[] = {
        {&FeelSettings::recoveryTime,.3f,1.5f},{&FeelSettings::cameraTravelWeight,.35f,.95f},
        {&FeelSettings::cameraResponseTime,.05f,1},{&FeelSettings::fovGain,0,6},
        {&FeelSettings::smokeScale,0,2},{&FeelSettings::audioScale,0,1.5f},
        {&FeelSettings::engineGain,0,1.5f},{&FeelSettings::tireGain,0,1.5f},
        {&FeelSettings::cameraDistance,3,10},{&FeelSettings::cameraHeight,1.2f,5},
        {&FeelSettings::cameraLookAhead,0,12},{&FeelSettings::cameraEntryTime,.1f,1.2f},
        {&FeelSettings::cameraExitTime,.15f,2},{&FeelSettings::cameraExitPower,1,5},
        {&FeelSettings::cameraPreviewWeight,0,.65f},{&FeelSettings::baseFov,45,80},
        {&FeelSettings::boostFovPeak,0,12},{&FeelSettings::boostFovAttack,.03f,.5f},
        {&FeelSettings::boostFovReturn,.3f,4},{&FeelSettings::chargeFovPeak,0,3},
        {&FeelSettings::chargeShakeScale,0,2},{&FeelSettings::boostSetbackPeak,0,.6f},
        {&FeelSettings::boostSetbackAttack,.03f,.5f},{&FeelSettings::boostSetbackReturn,.2f,2},
        {&FeelSettings::boostKickReturn,.1f,1.2f},{&FeelSettings::boostSpeedScale,0,2},
        {&FeelSettings::boostDurationScale,.25f,3},{&FeelSettings::chargeRateScale,.25f,3},
        {&FeelSettings::driftSteerRate,.3f,3},{&FeelSettings::driftSteerResponse,.05f,.5f},
        {&FeelSettings::topSpeedKmh,60,200},{&FeelSettings::accelerationTime,.2f,3},
        {&FeelSettings::stageScale,1,2.5f},{&FeelSettings::releaseDirectionTime,.1f,.6f},
        {&FeelSettings::boostSpeedAttack,.03f,.3f},{&FeelSettings::boostSpeedHold,0,.6f},{&FeelSettings::boostSpeedReturn,.2f,2},
        {&FeelSettings::boostSetbackHold,0,.6f},{&FeelSettings::boostFovHold,0,.6f},
        {&FeelSettings::cameraSteerLookAhead,0,.5f},{&FeelSettings::cameraSteerLookMax,0,18},
        {&FeelSettings::bodyRollMax,0,10},{&FeelSettings::bodyPitchMax,0,15},
        {&FeelSettings::bodyThrottlePitch,0,10},{&FeelSettings::bodyBrakePitch,0,10},
        {&FeelSettings::bodyBoostPitch,0,12},{&FeelSettings::bodyPitchHz,.8f,4},
        {&FeelSettings::bodyPitchDamping,.2f,.9f},
        {&FeelSettings::bodyLeanHz,.8f,4},{&FeelSettings::bodyLeanDamping,.2f,.9f},
        {&FeelSettings::bodyFlickScale,0,2},
        {&FeelSettings::bodyBounceHeight,0,.08f},{&FeelSettings::bodyBounceReturn,.2f,1},
        {&FeelSettings::boostBlurStrength,0,2},
        {&FeelSettings::gravity,6,22},{&FeelSettings::landingImpactScale,0,2},
        {&FeelSettings::speedCueScale,0,2},{&FeelSettings::windGain,0,1.5f},
        {&FeelSettings::suspensionTravel,.05f,.45f},{&FeelSettings::maxLaunchSpeed,2,8},
        {&FeelSettings::airSteerControl,0,.8f},{&FeelSettings::engineRevScale,0,2},
        {&FeelSettings::itemKickKmh,0,60},{&FeelSettings::itemChargeSeconds,0,1},{&FeelSettings::moveSpeedScale,.5f,2},{&FeelSettings::turnRateScale,.5f,2},
        {&FeelSettings::chargeSemis1,-12,12},
        {&FeelSettings::chargeSemis2,-12,12},
        {&FeelSettings::chargeSemis3,-12,12},
        {&FeelSettings::chargeDb1,-12,12},
        {&FeelSettings::chargeDb2,-12,12},
        {&FeelSettings::chargeDb3,-12,12},
        {&FeelSettings::chargeBlipSemis,0,12},
        {&FeelSettings::chargeBlipDb,0,12},
        {&FeelSettings::chargeBlipTime,0.05f,1},
        {&FeelSettings::chargeCreepSemis,0,6},
        {&FeelSettings::boostSemis1,0,12},
        {&FeelSettings::boostSemis2,0,12},
        {&FeelSettings::boostSemis3,0,12},
        {&FeelSettings::boostDb,-12,12},
        {&FeelSettings::boostTime,0.4f,2.5f},
        {&FeelSettings::itemRevSemis,0,12},
        {&FeelSettings::itemRevDb,-12,12},
        {&FeelSettings::itemRevTime,0.05f,1},
        {&FeelSettings::boostWhooshDb,-30,12},
        {&FeelSettings::itemWhooshDb,-30,12},
        {&FeelSettings::itemPickupDb,-30,12},
        {&FeelSettings::afterfireDb,-30,12},
        {&FeelSettings::engineRevReturn,.2f,1.5f},
    };
    for (const Bound& bound : bounds) {
        FeelSettings candidate = defaults;
        candidate.*(bound.field) = std::numeric_limits<float>::quiet_NaN();
        check(near(sanitizeFeelSettings(candidate).*(bound.field), defaults.*(bound.field)),
              "nonfinite saved parameters fall back to the individual default");
        candidate.*(bound.field) = -100;
        check(near(sanitizeFeelSettings(candidate).*(bound.field), bound.low), "each parameter enforces its lower bound");
        candidate.*(bound.field) = 1e6f;
        check(near(sanitizeFeelSettings(candidate).*(bound.field), bound.high), "each parameter enforces its upper bound");
        candidate.*(bound.field) = 0;
        check(near(sanitizeFeelSettings(candidate).*(bound.field), std::clamp(0.0f, bound.low, bound.high)),
              "zero-valued controls are preserved when zero is in range");
    }

    FeelSettings custom;
    custom.cameraDistance = 7;
    custom.cameraHeight = 3;
    custom.cameraLookAhead = 6;
    custom.baseFov = 68;
    custom.cameraPreviewWeight = 0;
    custom.chargeFovPeak = 2;
    custom.chargeShakeScale = 0;
    custom.cameraExitTime = .9f;
    custom.cameraExitPower = 2;
    Car held;
    setHeld(held);
    CameraPresentation view;
    view.update(0, held, custom);
    const Vec3 forward{std::sin(view.heading),0,std::cos(view.heading)};
    check(near(dot(view.anchor-view.eye,forward),7) && near(view.eye.y-view.anchor.y,3) &&
          near(dot(view.look-view.anchor,forward),6) && near(view.fovDegrees,68) && near(view.lookHeading,view.heading),
          "custom distance, height, lookahead, base lens and disabled preview all reach the actual pose");
    held.driftCharge = 2.2f;
    run(view,1,kDt,held,custom);
    check(near(view.fovDegrees,66,.002f) && length(view.chargeShake)==0,
          "charge lens strength and a true zero shake setting act independently");
    const float from = view.heading;
    held.driftHeld = held.driftHoldActive = false;
    held.driftPhase = DriftPhase::None;
    held.releaseEaseActive = true;
    view.update(.1f,held,custom);
    check(near(view.heading,from+wrapPi(held.yaw-from)*(1-std::pow(1-.1f/.9f,2))),
          "the live exit duration and front-loaded power replace the compiled default curve");

    FeelSettings lens;
    lens.boostFovHold = lens.boostSetbackHold = 0;
    lens.boostFovPeak = 8;
    lens.boostFovAttack = .2f;
    lens.boostFovReturn = 2.4f;
    lens.boostSetbackPeak = .4f;
    lens.boostKickReturn = .7f;
    Car car;
    car.speed = 22;
    CameraPresentation reward;
    reward.update(0,car,lens);
    boost(car,1,2);
    run(reward,.2f,kDt,car,lens);
    check(near(reward.boostFovOffset,6.4f,.002f) && reward.boostSetback>0,
          "custom boost strength and attack use the same tier ratios while the independently lengthened kick remains active");
    run(reward,1.2f,kDt,car,lens);
    check(near(reward.boostFovOffset,3.2f,.003f) && reward.boostSetback==0,
          "custom long FOV reaches its midpoint after the shorter positional cue has ended");
    run(reward,1.2f,kDt,car,lens);
    check(reward.boostFovAmount==0 && near(reward.fovDegrees,60), "custom lens tails finish at their own configured duration");
    lens.boostFovPeak = lens.boostSetbackPeak = 0;
    boost(car,2,3);
    run(reward,.2f,kDt,car,lens);
    check(reward.boostFovOffset==0 && reward.boostSetback==0 && std::isfinite(reward.fovDegrees),
          "zero lens and setback settings disable their channels without division by zero");

    CameraPresentation defaultLens;
    car = {};
    car.speed = 22;
    defaultLens.update(0,car,defaults);
    boost(car);
    run(defaultLens,.1f,kDt,car,defaults);
    check(near(defaultLens.boostFovOffset,5), "new default lens keeps the one-tenth opening");
    run(defaultLens,.2f,kDt,car,defaults);
    check(near(defaultLens.boostFovOffset,5), "new default lens holds its tier peak through three hundred milliseconds");
    run(defaultLens,.9f,kDt,car,defaults);
    check(near(defaultLens.boostFovOffset,2.5f,.002f), "new default wide-view emphasis reaches half return after its opening and hold");
    run(defaultLens,.9f,kDt,car,defaults);
    check(defaultLens.boostFovAmount==0 && near(defaultLens.fovDegrees,60), "new default lens settles after the extended one-point-eight-second return");
}

void testMovingPhaseReferences() {
    float worstEntryStep = 0, worstExitStep = 0, worstHeldError = 0;
    for (int fps : {30, 60, 120}) for (float side : {-1.0f, 1.0f}) {
        const float dt = 1.0f / fps;
        FeelSettings settings;
        settings.cameraEntryTime = 1.2f;
        settings.cameraExitTime = 2;
        const float slip = side * kPi / 3;
        const float speed = side * 240 * kPi / 180;
        Car car;
        car.yaw = car.travelYaw = side * 179 * kPi / 180;
        CameraPresentation view;
        view.update(0, car, settings);
        const float start = car.travelYaw;
        setHeld(car, slip);
        car.travelYaw = start;
        car.yaw = wrapPi(start + slip);
        car.driftAge = 0.01f;
        view.update(dt, car, settings);
        check(near(wrapPi(view.heading - start), 0), "entry rebases its orbit from the actual pose without moving to a different carrier");
        const float localTarget = slip * (1 - settings.cameraTravelWeight);
        float previous = view.heading;
        for (int frame = 1; frame <= fps * 2; ++frame) {
            car.travelYaw = wrapPi(start + speed * dt * frame);
            car.yaw = wrapPi(car.travelYaw + slip);
            car.driftAge += dt;
            view.update(dt, car, settings);
            const float local = wrapPi(view.heading - car.travelYaw);
            worstEntryStep = std::fmax(worstEntryStep, std::fabs(wrapPi(view.heading - previous)));
            check(local * side >= -0.0001f && local * side <= std::fabs(localTarget) + 0.0001f,
                  "a turning entry transports its carrier throughout the local delay and easing without accumulating world lag");
            if (view.motionPhase == CameraMotionPhase::Held) {
                const float error = std::fabs(wrapPi(local - localTarget));
                worstHeldError = std::fmax(worstHeldError, error);
                check(error < 0.0001f, "held tracking transports a constant local orbit across repeated world-angle seams");
            }
            previous = view.heading;
        }
        check(worstEntryStep < 12 * kPi / 180,
              "moving the entry endpoint through its original antipode cannot flip the camera's interpolation branch");

        car.driftHoldActive = car.driftHeld = false;
        car.driftPhase = DriftPhase::None;
        car.releaseEaseActive = true;
        const float exitStart = car.yaw;
        const float residual = wrapPi(view.heading - car.yaw);
        view.update(dt, car, settings);
        previous = view.heading;
        for (int frame = 1; frame < fps * 2; ++frame) {
            car.yaw = car.travelYaw = wrapPi(exitStart + speed * dt * frame);
            car.slipAngle = 0;
            view.update(dt, car, settings);
            const float elapsed = dt * (frame + 1);
            const float remaining = std::fmax(1 - elapsed / settings.cameraExitTime, 0.0f);
            check(near(wrapPi(view.heading - car.yaw), residual * std::pow(remaining, settings.cameraExitPower), 0.0002f),
                  "manual exit decays only the local residual while steering continues to transport the live nose direction");
            worstExitStep = std::fmax(worstExitStep, std::fabs(wrapPi(view.heading - previous)));
            previous = view.heading;
        }
        check(worstExitStep < 12 * kPi / 180,
              "a moving exit reference may pass its original antipode without a branch jump or reversed sweep");
        check(view.motionPhase == CameraMotionPhase::Grip && near(wrapPi(view.heading - car.yaw), 0, 0.0002f),
              "the turning exit still finishes at its configured finite duration");
    }
    std::printf("moving references: max entry/exit step %.2f/%.2fdeg, held local error %.4fdeg\n",
                worstEntryStep * 180 / kPi, worstExitStep * 180 / kPi, worstHeldError * 180 / kPi);
}

struct RapidTrace {
    float maxBodyGap = 0, maxTravelGap = 0, maxStep = 0, maxLookStep = 0, finalGap = 0;
    float earlyTravelGap = 0, lateTravelGap = 0, tailStep = 0;
    int entries = 0, exits = 0;
    bool crossedSeam = false;
    std::array<float, 6> heading{}, travel{};
};

RapidTrace rapidTrace(int fps, DriftModel model, float side, int halfFrames, bool alternate, const FeelSettings& settings) {
    Track track = temporalTrack();
    track.spawnYaw = side * 179 * kPi / 180;
    Car car;
    car.driftModel = model;
    car.reset(track);
    car.speed = 80 / 3.6f;
    CameraPresentation view;
    view.update(0, car, settings);
    const int presentationTicks = 120 / fps;
    RapidTrace trace;
    float previousTravel = car.travelYaw;
    CameraMotionPhase previousPhase = view.motionPhase;
    for (int tick = 1; tick <= 960; ++tick) {
        if (tick % 2 == 0) {
            const int frame = tick / 2 - 1;
            DriveInput input;
            input.throttle = 1;
            if (frame < 360) {
                const int half = frame / halfFrames;
                input.handbrake = half % 2 == 0 ? 1 : 0;
                input.steer = side * (alternate && (half / 2) % 2 ? -1 : 1);
            }
            car.update(kDt, input, track);
            trace.crossedSeam |= std::fabs(car.travelYaw - previousTravel) > kPi;
            previousTravel = car.travelYaw;
        }
        if (tick % presentationTicks == 0) {
            const float previous = view.heading, previousLook = view.lookHeading;
            view.update(float(presentationTicks) / 120, car, settings);
            const float step = std::fabs(wrapPi(view.heading - previous));
            trace.maxStep = std::fmax(trace.maxStep, step);
            trace.maxLookStep = std::fmax(trace.maxLookStep, std::fabs(wrapPi(view.lookHeading - previousLook)));
            trace.entries += view.motionPhase == CameraMotionPhase::Entry && previousPhase != view.motionPhase;
            trace.exits += view.motionPhase == CameraMotionPhase::Exit && previousPhase != view.motionPhase;
            previousPhase = view.motionPhase;
            const float travelGap = std::fabs(wrapPi(view.heading - car.travelYaw));
            trace.maxTravelGap = std::fmax(trace.maxTravelGap, travelGap);
            trace.maxBodyGap = std::fmax(trace.maxBodyGap, std::fabs(wrapPi(view.heading - car.yaw)));
            if (tick <= 240) trace.earlyTravelGap = std::fmax(trace.earlyTravelGap, travelGap);
            if (tick >= 480 && tick <= 720) trace.lateTravelGap = std::fmax(trace.lateTravelGap, travelGap);
            if (tick > 840) trace.tailStep = std::fmax(trace.tailStep, step);
            check(std::isfinite(view.heading) && std::isfinite(view.lookHeading) && std::isfinite(view.eye.x) &&
                  std::isfinite(view.eye.z), "rapid steering and phase interruptions keep every camera pose finite");
        }
        if (tick <= 720 && tick % 120 == 0) {
            trace.heading[tick / 120 - 1] = view.heading;
            trace.travel[tick / 120 - 1] = car.travelYaw;
        }
    }
    trace.finalGap = std::fabs(wrapPi(view.heading - car.yaw));
    check(trace.entries >= 19 && trace.exits >= 19 && trace.crossedSeam,
          "the rapid-input fixture actually crosses world seams and repeatedly interrupts entry and exit for six seconds");
    check(trace.maxTravelGap < kPi / 2 && trace.maxBodyGap < kPi / 2,
          "rapid Space inputs never accumulate a view from the front hemisphere of the live car or travel line");
    check(trace.lateTravelGap <= trace.earlyTravelGap + 12 * kPi / 180,
          "repeating the same Space cadence does not keep accumulating a larger tracking error");
    check(trace.maxStep < 15 * kPi / 180 && trace.maxLookStep < 15 * kPi / 180,
          "rapid manual entry, release and opposite-side reentry remain continuous at displayed frame rates");
    check(trace.finalGap < 0.05f * kPi / 180 && trace.tailStep < 0.02f * kPi / 180,
          "releasing steering and Space settles the camera without a lingering sweep or spontaneous orbit");
    return trace;
}

void testRapidManualInterruptions() {
    for (DriftModel model : {DriftModel::Inertia, DriftModel::LineControl}) {
        float maxGap = 0, maxStep = 0, maxFpsGap = 0;
        for (int halfFrames : {6, 9}) for (bool alternate : {false, true}) {
            FeelSettings settings;
            const RapidTrace reference = rapidTrace(120, model, 1, halfFrames, alternate, settings);
            const RapidTrace mirrored = rapidTrace(120, model, -1, halfFrames, alternate, settings);
            for (size_t i = 0; i < reference.heading.size(); ++i)
                check(std::fabs(wrapPi(reference.heading[i] + mirrored.heading[i])) < 0.001f,
                      "rapid drift interruptions remain symmetric on mirrored sides of the world-angle seam");
            for (int fps : {30, 60, 120}) {
                const RapidTrace trace = rapidTrace(fps, model, 1, halfFrames, alternate, settings);
                maxGap = std::fmax(maxGap, trace.maxTravelGap);
                maxStep = std::fmax(maxStep, trace.maxStep);
                for (size_t i = 0; i < reference.heading.size(); ++i) {
                    const float difference = std::fabs(wrapPi(trace.heading[i] - reference.heading[i]));
                    maxFpsGap = std::fmax(maxFpsGap, difference);
                    check(difference < 8 * kPi / 180,
                          "rapid-input local camera tracking has comparable landmarks at30/60/120Hz");
                }
            }
        }
        std::printf("rapid %s: max travel gap %.2fdeg, max displayed orbit step %.2fdeg, max FPS landmark difference %.2fdeg\n",
                    model == DriftModel::Inertia ? "A" : "B", maxGap * 180 / kPi, maxStep * 180 / kPi, maxFpsGap * 180 / kPi);
    }
}

void testLiveCameraSettings() {
    FeelSettings original;
    original.boostFovHold = original.boostSetbackHold = 0;
    original.cameraSteerLookAhead = 0;
    Car car;
    car.speed = 22;
    car.x = 3;
    car.yaw = car.travelYaw = .4f;
    CameraPresentation view;
    view.update(0, car, original);
    const CameraPresentation frozen = view;
    FeelSettings framing = original;
    framing.cameraDistance = 8;
    framing.cameraHeight = 3;
    framing.cameraLookAhead = 7;
    framing.baseFov = 72;
    view.applySettings(car, original, framing);
    const Vec3 forward{std::sin(view.heading),0,std::cos(view.heading)};
    check(near(dot(view.anchor-view.eye,forward),8) && near(view.eye.y-view.anchor.y,3) &&
          near(dot(view.look-view.anchor,forward),7) && near(view.fovDegrees,72),
          "an explicit paused settings revision immediately changes distance, height, lookahead and base FOV");
    check(view.heading == frozen.heading && view.motionPhase == frozen.motionPhase &&
          view.motionElapsed == frozen.motionElapsed && length(view.anchor-frozen.anchor)==0 &&
          car.x==3 && car.speed==22 && !view.boostTriggered && view.boostEventSerial==0,
          "live composition preserves the car, current heading, anchor, motion phase and event state");
    const CameraPresentation refreshed = view;
    view.update(3,car,framing,false);
    check(length(view.eye-refreshed.eye)==0 && view.fovDegrees==refreshed.fovDegrees,
          "ordinary paused updates remain frozen after an explicit composition refresh");
    boost(car);
    view.applySettings(car,framing,original);
    check(!view.boostTriggered && view.boostEventSerial==0 && view.boostAmount==0,
          "refreshing a pending physical reward cannot consume its serial or create a visual attack");
    view.update(kDt,car,original);
    check(view.boostTriggered && view.boostEventSerial==1,
          "the first active frame still receives a boost that was pending during a settings refresh");

    Car charging;
    setHeld(charging);
    charging.driftCharge = 2.2f;
    CameraPresentation chargeView;
    chargeView.update(0,charging,original);
    run(chargeView,.5f,kDt,charging,original);
    const float charge = chargeView.chargeAmount;
    const Vec3 shake = chargeView.chargeShake;
    FeelSettings chargeSettings = original;
    chargeSettings.chargeFovPeak = 3;
    chargeSettings.chargeShakeScale = 0;
    chargeView.applySettings(charging,original,chargeSettings);
    check(near(chargeView.fovDegrees,60-3*charge) && length(chargeView.chargeShake)==0 && chargeView.chargeAmount==charge,
          "paused charge strength and zero shake controls update without advancing the buildup");
    chargeView.applySettings(charging,chargeSettings,original);
    check(length(chargeView.chargeShake-shake)<0.000001f,
          "live tuning does not advance or restart the charge shake clock");

    Car drifting;
    CameraPresentation entry;
    entry.update(0,drifting,original);
    setHeld(drifting);
    drifting.driftAge = .01f;
    entry.update(.24f,drifting,original);
    CameraPresentation unrelated = entry, unedited = entry;
    unrelated.applySettings(drifting,original,framing);
    Car changingSlip = drifting;
    changingSlip.yaw = changingSlip.slipAngle = 50 * kPi / 180;
    unrelated.update(kDt,changingSlip,framing);
    unedited.update(kDt,changingSlip,original);
    check(unrelated.heading==unedited.heading && unrelated.motionElapsed==unedited.motionElapsed,
          "editing unrelated framing controls cannot rebase or change a moving entry's heading trajectory");
    const float entryHeading = entry.heading;
    FeelSettings longer = original;
    longer.cameraEntryTime = .78f;
    longer.cameraTravelWeight = .5f;
    entry.applySettings(drifting,original,longer);
    check(near(entry.motionElapsed,.42f) && entry.heading==entryHeading && entry.motionPhase==CameraMotionPhase::Entry,
          "a live entry duration edit retains the same progress after its fixed initial delay and preserves the shown heading");
    entry.update(.000001f,drifting,longer);
    check(std::fabs(wrapPi(entry.heading-entryHeading))<0.0001f,
          "a simultaneous travel-weight edit resumes from the current entry orbit rather than jumping to a new curve point");
    entry.update(.36f,drifting,longer);
    check(entry.motionPhase==CameraMotionPhase::Held && near(entry.heading,drifting.slipAngle*.5f),
          "the edited entry completes after its remaining new duration at the new mixed direction");
    CameraPresentation delayed;
    Car delayedCar;
    delayed.update(0,delayedCar,original);
    setHeld(delayedCar);
    delayedCar.driftAge=.01f;
    delayed.update(.03f,delayedCar,original);
    delayed.applySettings(delayedCar,original,longer);
    check(near(delayed.motionElapsed,.03f), "changing entry duration does not stretch its separate sixty-millisecond delay");
    delayed.update(.03f,delayedCar,longer);
    check(near(delayed.heading,0), "the edited entry still holds only its local orbit during the unchanged initial delay");

    CameraPresentation exit;
    exit.update(0,drifting,original);
    drifting.driftHoldActive=drifting.driftHeld=false;
    drifting.driftPhase=DriftPhase::None;
    drifting.releaseEaseActive=true;
    exit.update(.11f,drifting,original);
    const float exitHeading=exit.heading, exitResidual=wrapPi(exit.heading-drifting.yaw);
    FeelSettings exitSettings=original;
    exitSettings.cameraExitTime=1.1f;
    exitSettings.cameraExitPower=5;
    exit.applySettings(drifting,original,exitSettings);
    check(near(exit.motionElapsed,.22f) && exit.heading==exitHeading && exit.motionPhase==CameraMotionPhase::Exit,
          "live exit duration and curve edits preserve its current pose, phase and normalized progress");
    exit.update(.44f,drifting,exitSettings);
    check(near(wrapPi(exit.heading-drifting.yaw),exitResidual/32,.0002f),
          "a changed exit curve decays the current residual over the remaining motion instead of replaying its beginning");
    exit.update(.44f,drifting,exitSettings);
    check(exit.motionPhase==CameraMotionPhase::Grip && near(exit.heading,drifting.yaw),
          "the live-edited exit still finishes at its new finite endpoint");

    FeelSettings lens=original;
    lens.boostFovAttack=.4f;
    lens.boostFovReturn=3.6f;
    lens.boostKickReturn=.6f;
    Car rewardCar;
    rewardCar.speed=22;
    CameraPresentation attack;
    attack.update(0,rewardCar,original);
    boost(rewardCar);
    attack.update(.05f,rewardCar,original);
    const float attackAmount=attack.boostFovAmount;
    attack.applySettings(rewardCar,original,lens);
    check(near(attackAmount,.5f) && attack.boostFovAmount==attackAmount && !attack.boostTriggered,
          "a live FOV attack edit leaves its current normalized emphasis untouched without rearming the reward");
    attack.update(.2f,rewardCar,lens);
    check(near(attack.boostFovAmount,1), "a half-open lens remains half-open in the newly lengthened attack and reaches its peak after the remaining half");
    CameraPresentation tail;
    rewardCar={}; rewardCar.speed=22;
    tail.update(0,rewardCar,original);
    boost(rewardCar);
    tail.update(.55f,rewardCar,original);
    const float tailAmount=tail.boostFovAmount;
    tail.applySettings(rewardCar,original,lens);
    check(tail.boostFovAmount==tailAmount, "a live return edit retains the already reached FOV amount");
    tail.update(.9f,rewardCar,lens);
    check(near(tail.boostFovAmount,.5f,.0002f), "the lens proceeds from one-quarter to one-half return using the edited remaining duration");

    CameraPresentation kick;
    rewardCar={}; rewardCar.speed=22;
    kick.update(0,rewardCar,original);
    boost(rewardCar);
    kick.update(.125f,rewardCar,original);
    const float kickAmount=kick.boostAmount;
    kick.applySettings(rewardCar,original,lens);
    check(kick.boostAmount==kickAmount, "editing the short positional return does not rewind its normalized kick amount");
    kick.update(.15f,rewardCar,lens);
    check(near(kick.boostAmount,.125f,.0002f), "the short return independently resumes at its preserved progress");

    CameraPresentation cancelled;
    rewardCar={}; rewardCar.speed=22;
    cancelled.update(0,rewardCar,original);
    boost(rewardCar);
    cancelled.update(.05f,rewardCar,original);
    rewardCar.boostRemaining=rewardCar.boostSpeedBonus=0;
    rewardCar.boostLevel=0;
    rewardCar.speed=22;
    cancelled.update(.075f,rewardCar,original);
    const float cancelledAmount=cancelled.boostFovAmount;
    cancelled.applySettings(rewardCar,original,lens);
    check(cancelled.boostFovAmount==cancelledAmount && !cancelled.boostTriggered,
          "live settings preserve an in-progress interruption fade without restarting either envelope");
    cancelled.update(.15f,rewardCar,lens);
    check(near(cancelled.boostAmount,.5f,.0002f) && near(cancelled.boostFovAmount,.25f,.0002f),
          "both interrupted envelopes keep their cancellation progress when the shared fade duration changes");

    CameraPresentation expires;
    rewardCar={}; rewardCar.speed=22;
    expires.update(0,rewardCar,original);
    boost(rewardCar);
    expires.update(.2f,rewardCar,original);
    rewardCar.boostRemaining=.01f;
    rewardCar.boostTotalDuration=.1f;
    expires.applySettings(rewardCar,original,original);
    rewardCar.boostRemaining=rewardCar.boostSpeedBonus=0;
    rewardCar.boostLevel=0;
    rewardCar.speed=22;
    run(expires,.15f,kDt,rewardCar,original);
    check(expires.boostFovAmount>.9f && !expires.boostTriggered && expires.boostEventSerial==1,
          "shortening a physical burst during pause does not misclassify its next-frame natural expiry as an interrupted FOV tail");

    FeelSettings invalid=original;
    invalid.cameraEntryTime=std::numeric_limits<float>::quiet_NaN();
    invalid.cameraExitTime=0;
    invalid.boostFovAttack=invalid.boostFovReturn=invalid.boostKickReturn=0;
    tail.applySettings(rewardCar,lens,invalid);
    check(std::isfinite(tail.heading) && std::isfinite(tail.fovDegrees) && std::isfinite(tail.motionElapsed),
          "invalid live duration values are sanitized before progress clocks are remapped");
}

void testIndependentBoostDolly() {
    const FeelSettings settings;
    float peakVelocity=0,tailVelocity=0;
    for (int fps : {30,60,120}) for (int level=1;level<=3;++level) {
        Car car;
        car.speed=22;
        car.yaw=car.travelYaw=.7f;
        CameraPresentation view,withoutDolly;
        FeelSettings disabled=settings;
        disabled.boostSetbackPeak=0;
        view.update(0,car,settings);
        withoutDolly.update(0,car,disabled);
        boost(car,1,level);
        const Car fixedCar=car;
        const float peak=.25f*(2+level)/5;
        for (int frame=1;frame<=fps*12/10;++frame) {
            view.update(1.0f/fps,car,settings);
            withoutDolly.update(1.0f/fps,car,disabled);
            const Vec3 forward{std::sin(view.heading),0,std::cos(view.heading)};
            const Vec3 right{forward.z,0,-forward.x};
            const Vec3 difference=view.eye-withoutDolly.eye;
            check(near(dot(difference,forward),-view.boostSetback,.00001f) &&
                  std::fabs(dot(difference,right))<.00001f && std::fabs(difference.y)<.00001f &&
                  length(view.look-withoutDolly.look)<.00001f && view.heading==withoutDolly.heading,
                  "the boost dolly moves only the eye backward along the current heading without changing aim target, orbit, height or lateral position");
            check(view.boostAmount==withoutDolly.boostAmount && view.boostFovAmount==withoutDolly.boostFovAmount &&
                  view.fovDegrees==withoutDolly.fovDegrees && car.speed==fixedCar.speed && car.x==fixedCar.x && car.yaw==fixedCar.yaw,
                  "position strength cannot alter jet timing, FOV, steering or physical speed");
            check(view.boostSetback>=0 && view.boostSetback<=peak+.00001f && view.boostViewAmount<=1,
                  "every charge tier has a bounded nonnegative position envelope at30/60/120Hz");
            if (frame==fps/10) check(near(view.boostSetback,peak,.0001f),
                                     "the independent dolly reaches its exact tier peak at100ms at every tested frame rate");
            if (frame==fps*3/10) check(near(view.boostSetback,peak,.0001f),
                                      "the independent dolly retains its tier peak through the200ms hold");
            if (frame==fps*7/10) check(near(view.boostSetback,peak*.5f,.0001f) && view.boostAmount==0 && view.boostFovAmount>0,
                                      "the dolly is halfway home at700ms after the jet ends and before the FOV return ends");
            if (frame==fps*11/10) check(view.boostSetback==0,
                                        "the independent dolly finishes after its100ms attack,200ms hold and800ms return");
        }
    }
    Car car;
    car.speed=22;
    CameraPresentation peak;
    peak.update(0,car,settings);
    boost(car);
    peak.update(.099f,car,settings);
    const float justBefore=peak.boostSetback;
    peak.update(.001f,car,settings);
    const float atPeak=peak.boostSetback;
    peak.update(.001f,car,settings);
    peakVelocity=std::fmax(std::fabs(atPeak-justBefore),std::fabs(peak.boostSetback-atPeak))/.001f;
    check(near(atPeak,.25f) && peakVelocity<.005f,
          "camera position velocity approaches zero from both sides of the dolly peak instead of reversing with a sharp velocity kink");
    peak.update(.998f,car,settings);
    const float almostHome=peak.boostSetback;
    peak.update(.001f,car,settings);
    tailVelocity=std::fabs(peak.boostSetback-almostHome)/.001f;
    check(peak.boostSetback==0 && tailVelocity<.005f,
          "the returning dolly arrives at the base pose with negligible final velocity and no final clamp step");

    CameraPresentation weak;
    car={}; car.speed=22;
    weak.update(0,car,settings);
    boost(car);
    weak.update(.15f,car,settings);
    const float previous=weak.boostViewAmount;
    boost(car,2,1);
    weak.update(.000001f,car,settings);
    check(weak.boostTriggered && near(weak.boostViewAmount,previous,.000001f),
          "a weaker reboost begins continuously at the current dolly position without moving forward or stacking distance");
    weak.update(.099999f,car,settings);
    check(near(weak.boostViewAmount,previous,.00001f) && weak.boostSetback<=.25f,
          "a weaker fresh boost retains a farther-out existing camera position through its attack rather than catching it forward");
    weak.update(.4f,car,settings);
    const float beforeStronger=weak.boostViewAmount;
    boost(car,3,3);
    weak.update(.000001f,car,settings);
    check(std::fabs(weak.boostViewAmount-beforeStronger)<.00001f,
          "a stronger reboost also starts from its actual current camera position");
    weak.update(.099999f,car,settings);
    check(near(weak.boostViewAmount,1) && near(weak.boostSetback,.25f),
          "reboost replaces the independent position envelope at its capped peak without additive camera travel");

    CameraPresentation live;
    car={}; car.speed=22;
    live.update(0,car,settings);
    boost(car);
    live.update(.05f,car,settings);
    CameraPresentation unedited=live;
    FeelSettings longer=settings;
    longer.boostSetbackAttack=.2f;
    longer.boostSetbackReturn=1.6f;
    const Vec3 pausedEye=live.eye;
    live.applySettings(car,settings,longer);
    check(near(live.boostViewAmount,.5f) && length(live.eye-pausedEye)==0 && !live.boostTriggered,
          "changing dolly timing during pause preserves current eye position and attack progress without restarting the reward");
    live.update(.05f,car,longer);
    unedited.update(.05f,car,settings);
    check(near(live.boostViewAmount,.896484375f) && live.boostAmount==unedited.boostAmount &&
          live.boostFovAmount==unedited.boostFovAmount,
          "live dolly timing stretches only its own attack while leaving the jet and lens clocks unchanged");
    live.update(.05f,car,longer);
    live.update(1.0f,car,longer);
    check(near(live.boostViewAmount,.5f,.0001f),
          "the live-edited dolly reaches half return after the remaining newly selected duration");
    FeelSettings amplitude=longer;
    amplitude.boostSetbackPeak=.6f;
    const float amount=live.boostViewAmount,jet=live.boostAmount,lens=live.boostFovAmount;
    live.applySettings(car,longer,amplitude);
    check(near(live.boostSetback,.3f) && live.boostViewAmount==amount && live.boostAmount==jet && live.boostFovAmount==lens,
          "paused live amplitude changes immediately recompose the current position without advancing any envelope");
    amplitude.boostSetbackPeak=0;
    live.applySettings(car,longer,amplitude);
    check(live.boostSetback==0 && live.boostViewAmount==amount,
          "zero dolly strength is an actual zero translation without destroying its ongoing timing state");
    const CameraPresentation frozen=live;
    live.update(4,car,amplitude,false);
    live.update(0,car,amplitude);
    live.update(-1,car,amplitude);
    live.update(std::numeric_limits<float>::infinity(),car,amplitude);
    check(live.boostViewAmount==frozen.boostViewAmount && length(live.eye-frozen.eye)==0,
          "pause, zero time and invalid elapsed time freeze the independent position clock at its current pose");

    CameraPresentation tail;
    car={}; car.speed=22;
    tail.update(0,car,settings);
    boost(car);
    tail.update(.5f,car,settings);
    CameraPresentation oldTail=tail;
    const float tailAmount=tail.boostViewAmount;
    tail.applySettings(car,settings,longer);
    check(tail.boostViewAmount==tailAmount, "a return-time edit also preserves the current normalized dolly return stage");
    tail.update(.4f,car,longer);
    oldTail.update(.4f,car,settings);
    check(near(tail.boostViewAmount,.5f,.0001f) && tail.boostAmount==oldTail.boostAmount && tail.boostFovAmount==oldTail.boostFovAmount,
          "retiming the dolly return changes its remaining duration independently of both other boost channels");

    CameraPresentation cancelled;
    car={}; car.speed=22;
    cancelled.update(0,car,settings);
    boost(car);
    cancelled.update(.1f,car,settings);
    car.driftHeld=true;
    car.boostRemaining=car.boostSpeedBonus=0;
    car.boostLevel=0;
    cancelled.update(.2f,car,settings);
    check(near(cancelled.boostViewAmount,.896484375f,.0001f),
          "an interrupted boost fades position once from its current pose using the independent smooth return");
    cancelled.update(.2f,car,settings);
    check(near(cancelled.boostViewAmount,.5f,.0001f),
          "persistent interruption cannot repeatedly restart the position fade");
    cancelled.applySettings(car,settings,longer);
    check(near(cancelled.boostViewAmount,.5f,.0001f), "live return edits preserve an in-progress position cancellation fade");
    cancelled.update(.8f,car,longer);
    check(cancelled.boostViewAmount==0 && cancelled.boostSetback==0 && !cancelled.boostTriggered,
          "the retimed cancellation still returns finitely to the base pose without a new boost event");
    for (float attack : {.03f,.5f}) for (float returning : {.2f,2.0f}) {
        FeelSettings extremes=settings;
        extremes.boostSetbackAttack=attack;
        extremes.boostSetbackReturn=returning;
        car={}; car.speed=22;
        CameraPresentation test;
        test.update(0,car,extremes);
        boost(car);
        test.update(attack,car,extremes);
        check(near(test.boostSetback,.25f) && std::isfinite(test.eye.z),
              "every allowed position attack and return extreme has a finite bounded peak");
        test.update(extremes.boostSetbackHold+returning,car,extremes);
        check(test.boostViewAmount==0 && test.boostSetback==0,
              "every allowed independent position timing combination has a finite exact endpoint");
    }
    std::printf("boost dolly: tier peaks .15/.20/.25m at100ms, hold through300ms, half return700ms, home1100ms; peak/tail sampled speed %.5f/%.5fm/s\n",
                peakVelocity,tailVelocity);
}

void testBoostCameraHoldsAndIntent() {
    const FeelSettings settings;
    for (int fps : {30,60,120}) for (int level=1;level<=3;++level) {
        Car car;
        car.speed=22;
        CameraPresentation view;
        view.update(0,car,settings);
        boost(car,1,level);
        car.boostSpeedBonus=0; // The physical attack starts at zero, but the release already exists.
        for (int frame=1;frame<=fps*22/10;++frame) {
            view.update(1.0f/fps,car,settings);
            check((frame==1)==view.boostTriggered && view.boostEventSerial==1,
                  "a zero-initial-speed reward starts camera envelopes exactly once from its intent serial");
            const float tier=(2+level)/5.0f;
            if (frame==fps/10 || frame==fps*2/10 || frame==fps*3/10)
                check(near(view.boostViewAmount,tier,.0002f) && near(view.boostFovAmount,tier,.0002f),
                      "all tiers share the100ms opening and200ms camera position and lens hold");
            if (frame==fps*7/10) check(near(view.boostViewAmount,tier*.5f,.0002f),
                                      "held position emphasis reaches half return at700ms");
            if (frame==fps*11/10) check(view.boostViewAmount==0 && view.boostFovAmount>0,
                                        "position finishes at1100ms while the separately longer lens remains active");
            if (frame==fps*12/10) check(near(view.boostFovAmount,tier*.5f,.0002f),
                                        "held lens emphasis reaches half return at1200ms");
            if (frame==fps*21/10) check(view.boostFovAmount==0,
                                        "held lens emphasis finishes after its100ms opening200ms hold and1800ms return");
        }
    }

    Car car;
    car.speed=22;
    CameraPresentation held;
    held.update(0,car,settings);
    boost(car);
    held.update(.2f,car,settings);
    FeelSettings longer=settings;
    longer.boostSetbackHold=longer.boostFovHold=.4f;
    held.applySettings(car,settings,longer);
    check(near(held.boostViewAmount,1) && near(held.boostFovAmount,1) && !held.boostTriggered,
          "paused live hold edits preserve peak position and lens amounts without a new reward");
    held.update(.2f,car,longer);
    check(near(held.boostViewAmount,1) && near(held.boostFovAmount,1),
          "a half-finished hold retains its normalized progress when its duration doubles");
    FeelSettings noHold=longer;
    noHold.boostSetbackHold=noHold.boostFovHold=0;
    held.applySettings(car,longer,noHold);
    check(near(held.boostViewAmount,1) && near(held.boostFovAmount,1),
          "removing an active hold begins return at the same peak without a discontinuity or division by zero");
    held.update(.4f,car,noHold);
    check(near(held.boostViewAmount,.5f,.0002f), "zero-hold position proceeds through its full remaining return");

    CameraPresentation returning;
    car={};car.speed=22;
    returning.update(0,car,noHold);
    boost(car);
    returning.update(.3f,car,noHold);
    const float position=returning.boostViewAmount,lens=returning.boostFovAmount;
    returning.applySettings(car,noHold,settings);
    check(returning.boostViewAmount==position && returning.boostFovAmount==lens,
          "adding a hold during return preserves the return stage rather than jumping back to peak");
    returning.update(.2f,car,settings);
    check(near(returning.boostViewAmount,.5f,.0002f), "return-stage progress survives a zero-to-positive hold edit");

    const Track track=temporalTrack();
    Car physical;
    physical.reset(track);
    physical.speed=80/3.6f;
    CameraPresentation actual;
    actual.update(0,physical,settings);
    DriveInput input;
    input.throttle=input.handbrake=1;input.steer=.6f;
    for(int frame=0;frame<200 && physical.driftChargeLevel<1;++frame) {
        physical.update(kDt,input,track);actual.update(kDt,physical,settings);
    }
    input.handbrake=input.steer=0;
    physical.update(kDt,input,track);
    check(physical.boostSerial==1 && physical.boostLevel==1 && physical.boostRemaining>0 && physical.boostSpeedBonus==0,
          "an actual manual release establishes reward intent before the new speed attack adds any bonus");
    actual.applySettings(physical,settings,settings);
    actual.update(kDt,physical,settings);
    check(actual.boostTriggered && actual.boostViewAmount>0 && actual.boostFovAmount>0 && actual.boostEventSerial==1,
          "paused baseline refresh cannot consume a pending zero-bonus physical release");
    physical.update(kDt,input,track);actual.update(kDt,physical,settings);
    check(!actual.boostTriggered && actual.boostEventSerial==1,
          "the real physical attack advancing above zero cannot replay its already observed camera event");
}

void testNormalSteeringPreview() {
    const FeelSettings settings;
    FeelSettings disabled=settings;
    disabled.cameraSteerLookAhead=0;
    float maxOptical=0,maxNeutralReverse=0;
    std::array<float,2> mirrored{};
    for (int fps : {30,60,120}) for (float side : {-1.0f,1.0f}) {
        const float dt=1.0f/fps;
        Car car;
        car.speed=22;car.yaw=car.travelYaw=side*179*kPi/180;
        CameraPresentation view,plain;
        view.update(0,car,settings);plain.update(0,car,disabled);
        for(int frame=0;frame<fps*2;++frame) {
            car.yawRate=side*1.2f;
            car.yaw=car.travelYaw=wrapPi(car.yaw+car.yawRate*dt);
            const Car before=car;
            view.update(dt,car,settings);plain.update(dt,car,disabled);
            const float optical=wrapPi(view.lookHeading-plain.lookHeading);
            maxOptical=std::fmax(maxOptical,std::fabs(optical));
            check(near(optical,view.steerLookOffset,.0001f) && optical*side>=0 &&
                  std::fabs(optical)<=8*kPi/180+.00001f,
                  "normal turn anticipation is the actual bounded optical yaw rather than an attenuated look-target angle");
            check(length(view.eye-plain.eye)<.00001f && view.heading==plain.heading &&
                  length(view.anchor-plain.anchor)<.00001f && view.fovDegrees==plain.fovDegrees &&
                  car.x==before.x && car.yaw==before.yaw && car.speed==before.speed,
                  "normal steering preview changes only the aim ray, never orbit, eye, lens or vehicle motion");
        }
        check(near(view.steerLookOffset,side*8*kPi/180,.0001f),
              "a steady forward circle reaches the selected actual eight-degree preview on either side of the angle seam");
        mirrored[side>0?1:0]=view.lookHeading;
        const float frozenOffset=view.steerLookOffset;
        const Vec3 frozenLook=view.look;
        view.update(3,car,settings,false);
        check(view.steerLookOffset==frozenOffset && length(view.look-frozenLook)==0,
              "pause freezes normal steering anticipation without advancing its filter");
        FeelSettings unrelated=settings;unrelated.smokeScale=2;
        view.applySettings(car,settings,unrelated);
        check(view.steerLookOffset==frozenOffset,
              "unrelated live setting edits cannot perturb steering preview state");
        view.clearBoostCue();
        check(near(view.steerLookOffset,frozenOffset) && length(view.look-frozenLook)<.00001f,
              "clearing boost layers retains the existing non-boost steering preview");
        float previousLook=view.lookHeading;
        car.yawRate=0;
        for(int frame=0;frame<fps;++frame) {
            view.update(dt,car,settings);plain.update(dt,car,disabled);
            const float step=side*wrapPi(view.lookHeading-previousLook);
            maxNeutralReverse=std::fmax(maxNeutralReverse,-step);
            check(step>=-.0001f, "centering full-circle steering does not whip optical aim backward while the chase orbit catches up");
            previousLook=view.lookHeading;
        }
        check(std::fabs(view.steerLookOffset)<.01f*kPi/180 &&
              std::fabs(wrapPi(view.lookHeading-car.yaw))<.02f*kPi/180,
              "neutral steering settles both the ordinary chase and optical anticipation without a second aim sweep");
        for(int frame=0;frame<fps;++frame) {
            car.yawRate=-side*1.2f;
            car.yaw=car.travelYaw=wrapPi(car.yaw+car.yawRate*dt);
            view.update(dt,car,settings);
            check(std::fabs(view.steerLookOffset)<=8*kPi/180+.00001f && std::isfinite(view.lookHeading),
                  "countersteering retains a finite bounded aim preview through the opposite world-angle seam");
        }
        check(-side*view.steerLookOffset>7.9f*kPi/180,
              "a sustained opposite turn moves the preview to the new inside rather than retaining the old direction");
        FeelSettings smaller=settings;smaller.cameraSteerLookMax=4;
        const Vec3 eye=view.eye;
        const float heading=view.heading,fov=view.fovDegrees;
        view.applySettings(car,settings,smaller);
        check(near(view.steerLookOffset,-side*4*kPi/180) && length(view.eye-eye)==0 &&
              view.heading==heading && view.fovDegrees==fov && !view.boostTriggered,
              "paused preview-limit edits immediately recompose only the optical ray without a simulated frame or reward");
        smaller.cameraSteerLookMax=0;
        view.applySettings(car,settings,smaller);
        check(view.steerLookOffset==0 && near(view.lookHeading,view.heading,.0001f),
              "zero preview strength immediately clears the visible steering offset");
    }
    check(std::fabs(wrapPi(mirrored[0]+mirrored[1]))<.0001f,
          "left and right normal steering previews are mirrored across the world-angle seam");
    for(float speed : {-8.0f,0.0f,5.0f,22.0f}) {
        Car car;car.speed=speed;car.yawRate=speed==22?0:1.2f;
        CameraPresentation view;view.update(0,car,settings);run(view,1,kDt,car,settings);
        check(view.steerLookOffset==0,
              "reverse, stopped, walking-speed and straight driving do not create a steering preview");
    }
    Car car;car.speed=22;car.yawRate=1.2f;
    CameraPresentation fading;fading.update(0,car,settings);run(fading,1,kDt,car,settings);
    const float prior=fading.steerLookOffset;
    car.drifting=car.driftHoldActive=car.driftHeld=true;car.driftPhase=DriftPhase::Entry;car.driftAge=.01f;
    fading.update(kDt,car,settings);
    check(fading.steerLookOffset>0 && fading.steerLookOffset<prior,
          "entering drift fades a prior normal peek gently instead of kicking it off or strengthening it");
    run(fading,1,kDt,car,settings);
    check(std::fabs(fading.steerLookOffset)<.01f*kPi/180,
          "held drift supplies no new normal-steering anticipation after its entry fade");
    std::printf("normal steering preview: max optical %.2fdeg, worst full-circle neutral reverse %.4fdeg/frame\n",
                maxOptical*180/kPi,maxNeutralReverse*180/kPi);
}

struct NormalSteeringTrace {
    std::array<float,7> optical{},aim{};
    float maxOptical=0,maxStep=0,neutralBack=0;
};

NormalSteeringTrace normalSteeringTrace(int fps, float side) {
    const Track track=temporalTrack();
    const FeelSettings settings;
    FeelSettings disabled=settings;disabled.cameraSteerLookAhead=0;
    Car car;car.reset(track);car.speed=80/3.6f;
    car.yaw=car.travelYaw=side*179*kPi/180;
    CameraPresentation view,plain;
    view.update(0,car,settings);plain.update(0,car,disabled);
    NormalSteeringTrace trace;
    for(int tick=1;tick<=840;++tick) {
        if(tick%2==0) {
            const int frame=tick/2-1;
            DriveInput input;input.throttle=1;
            input.steer=frame<180?side:frame<300?0:-side;
            car.update(kDt,input,track);
        }
        if(tick%(120/fps)==0) {
            const float previous=view.lookHeading;
            view.update(1.0f/fps,car,settings);plain.update(1.0f/fps,car,disabled);
            const float optical=wrapPi(view.lookHeading-plain.lookHeading);
            const float step=wrapPi(view.lookHeading-previous);
            trace.maxOptical=std::fmax(trace.maxOptical,std::fabs(optical));
            trace.maxStep=std::fmax(trace.maxStep,std::fabs(step));
            if(tick>360 && tick<=600) trace.neutralBack=std::fmax(trace.neutralBack,-side*step);
            check(view.motionPhase==CameraMotionPhase::Grip && near(optical,view.steerLookOffset,.0001f) &&
                  std::fabs(optical)<=8*kPi/180+.00001f && length(view.eye-plain.eye)<.00001f &&
                  view.heading==plain.heading && view.fovDegrees==plain.fovDegrees,
                  "real normal circle, center and countersteer inputs change only a bounded optical preview at30/60/120Hz");
            if(tick==600) check(std::fabs(view.steerLookOffset)<.001f*kPi/180 &&
                                 std::fabs(wrapPi(view.lookHeading-car.yaw))<.005f*kPi/180,
                                 "the real centered steering trace settles without a lingering preview or second rotation");
        }
        if(tick%120==0) {
            trace.optical[tick/120-1]=view.steerLookOffset;
            trace.aim[tick/120-1]=view.lookHeading;
        }
    }
    check(trace.maxStep<8*kPi/180 && trace.neutralBack<.15f*kPi/180,
          "real steering anticipation has no large angular step or backward neutral whip");
    return trace;
}

void testActualNormalSteeringPreview() {
    const NormalSteeringTrace reference=normalSteeringTrace(120,1);
    const NormalSteeringTrace mirror=normalSteeringTrace(120,-1);
    float maxAimDifference=0,maxNeutralBack=0;
    for(size_t index=0;index<reference.aim.size();++index)
        check(std::fabs(wrapPi(reference.aim[index]+mirror.aim[index]))<.0001f &&
              near(reference.optical[index],-mirror.optical[index],.0001f),
              "real left/right normal steering and countersteering remain mirrored across angle seams");
    for(int fps : {30,60,120}) {
        const NormalSteeringTrace trace=normalSteeringTrace(fps,1);
        maxNeutralBack=std::fmax(maxNeutralBack,trace.neutralBack);
        for(size_t index=0;index<reference.aim.size();++index) {
            const float difference=std::fabs(wrapPi(trace.aim[index]-reference.aim[index]));
            maxAimDifference=std::fmax(maxAimDifference,difference);
            check(difference<1.5f*kPi/180 && std::fabs(trace.optical[index]-reference.optical[index])<.2f*kPi/180,
                  "real60Hz normal steering produces comparable optical preview and camera landmarks at30/60/120Hz");
        }
    }
    std::printf("real normal preview: max optical %.2fdeg, max FPS aim difference %.2fdeg, neutral backward step %.4fdeg\n",
                reference.maxOptical*180/kPi,maxAimDifference*180/kPi,maxNeutralBack*180/kPi);
}

void beginCameraLaunch(Car& car, const CameraPresentation& view) {
    car.driftHeld = car.driftHoldActive = false;
    car.driftPhase = DriftPhase::None;
    car.releaseEaseActive = car.releaseUsesCameraHeading = true;
    car.releaseEaseElapsed = 0;
    car.releaseHeading = view.lookHeading;
}

void poseCameraLaunch(Car& car, float bodyFrom, float travelFrom, float target, float elapsed, float duration) {
    const float t = clampf(elapsed / duration, 0, 1);
    const float remaining = 1 - t * t * t * (t * (t * 6 - 15) + 10);
    car.releaseHeading = target;
    car.yaw = wrapPi(target + wrapPi(bodyFrom - target) * remaining);
    car.travelYaw = wrapPi(target + wrapPi(travelFrom - target) * remaining);
    car.slipAngle = wrapPi(car.yaw - car.travelYaw);
    car.releaseEaseElapsed = elapsed;
    car.releaseEaseActive = elapsed < duration;
    car.drifting = car.releaseEaseActive;
}

void testCameraDirectedLaunchContinuity() {
    for (float side : {-1.0f, 1.0f}) for (float seam : {-179.0f, 179.0f})
        for (float exitTime : {.15f, .55f, 1.2f}) {
        FeelSettings settings = baselineSettings();
        settings.cameraExitTime = exitTime;
        Car car;
        setHeld(car, side * kPi / 3);
        car.travelYaw = seam * kPi / 180;
        car.yaw = wrapPi(car.travelYaw + car.slipAngle);
        CameraPresentation view;
        view.update(0, car, settings);
        const float opticalBefore = view.lookHeading, orbitBefore = view.heading;
        const float bodyFrom = car.yaw, travelFrom = car.travelYaw;
        beginCameraLaunch(car, view);
        const float target = car.releaseHeading;
        view.update(0.000001f, car, settings);
        check(std::fabs(wrapPi(view.lookHeading - opticalBefore)) < .00001f &&
              std::fabs(wrapPi(view.heading - orbitBefore)) < .00001f,
              "capturing the visible optical launch line preserves the first eye and look headings across both angle seams");
        for (int frame = 1; frame <= 2000; ++frame) {
            const float elapsed = frame * .001f;
            poseCameraLaunch(car, bodyFrom, travelFrom, target, elapsed, settings.releaseDirectionTime);
            const float previousLook = view.lookHeading;
            view.update(.001f, car, settings);
            const float progress = clampf((elapsed + .000001f) / exitTime, 0, 1);
            const float expectedOrbit = wrapPi(target + wrapPi(orbitBefore - target) *
                                               std::pow(1 - progress, settings.cameraExitPower));
            check(std::fabs(wrapPi(view.heading - expectedOrbit)) < .00003f,
                  "launch orbit follows the captured release target rather than the independently counterrotating body");
            check(std::fabs(wrapPi(view.lookHeading - target)) < .00003f &&
                  std::fabs(wrapPi(view.lookHeading - previousLook)) < .00003f,
                  "the visible launch corridor stays continuous through250ms body alignment and shorter or longer camera exits");
        }
        check(view.motionPhase == CameraMotionPhase::Grip && view.boostPreview < .01f,
              "camera-directed launch completes normally and clears its old drift preview without a new backlog");
    }
}

void testCameraDirectedLaunchSteeringAndLiveSettings() {
    FeelSettings settings = baselineSettings();
    Car car;
    setHeld(car);
    CameraPresentation view;
    view.update(0, car, settings);
    const float bodyFrom = car.yaw, travelFrom = car.travelYaw;
    beginCameraLaunch(car, view);
    const CameraPresentation frozen = view;
    view.update(3, car, settings, false);
    check(view.heading == frozen.heading && view.lookHeading == frozen.lookHeading &&
          view.motionPhase == frozen.motionPhase,
          "pausing before a camera-directed release freezes its displayed pose and defers the new exit");
    view.update(.001f, car, settings);
    const float initialTarget = car.releaseHeading;
    float target = initialTarget;
    for (int frame = 1; frame <= 65; ++frame) {
        target = wrapPi(initialTarget + .2f * frame * .01f);
        poseCameraLaunch(car, bodyFrom, travelFrom, target, frame * .01f, settings.releaseDirectionTime);
        const float previousLook = view.lookHeading;
        view.update(.01f, car, settings);
        if (view.motionPhase == CameraMotionPhase::Exit || car.releaseEaseActive)
            check(std::fabs(wrapPi(view.lookHeading - target)) < .0001f &&
                  std::fabs(wrapPi(view.lookHeading - previousLook)) < .003f,
                  "normal steering carries the launch aim along the live target without adding the body's counterrotation");
        if (frame == 10) {
            const float oldHeading = view.heading, oldLook = view.lookHeading;
            FeelSettings changed = settings;
            changed.cameraExitTime = .9f;
            changed.cameraExitPower = 1.6f;
            view.applySettings(car, settings, changed);
            check(near(wrapPi(view.heading - oldHeading), 0) && near(wrapPi(view.lookHeading - oldLook), 0),
                  "paused live exit timing edits preserve both orbit and optical aim while the body still turns");
            settings = changed;
        }
    }
    car.releaseEaseActive = car.drifting = false;
    car.travelYaw = car.yaw = target;
    car.slipAngle = car.yawRate = 0;
    run(view, 2, kDt, car, settings);
    check(view.motionPhase == CameraMotionPhase::Grip && near(wrapPi(view.lookHeading - target), 0),
          "an aligned launch returns to the ordinary live body heading even while release metadata remains available");
}

void testCameraDirectedLaunchInterruptions() {
    FeelSettings settings = baselineSettings();
    for (bool reenter : {false, true}) {
        Car car;
        setHeld(car);
        CameraPresentation view;
        view.update(0, car, settings);
        const float bodyFrom = car.yaw, travelFrom = car.travelYaw;
        beginCameraLaunch(car, view);
        const float target = car.releaseHeading;
        poseCameraLaunch(car, bodyFrom, travelFrom, target, .1f, settings.releaseDirectionTime);
        view.update(.1f, car, settings);
        const float oldHeading = view.heading, oldLook = view.lookHeading;
        car.releaseEaseActive = car.releaseUsesCameraHeading = false;
        car.drifting = true;
        if (reenter) {
            car.driftHeld = car.driftHoldActive = true;
            car.driftPhase = DriftPhase::Entry;
            car.driftAge = 0;
        }
        view.update(.000001f, car, settings);
        check(std::fabs(wrapPi(view.heading - oldHeading)) < .00001f &&
              std::fabs(wrapPi(view.lookHeading - oldLook)) < .00001f,
              "a cancelled or interrupted camera launch rebases its current displayed orbit and optical ray exactly once");
        check(view.motionPhase == (reenter ? CameraMotionPhase::Entry : CameraMotionPhase::Recovery),
              "a new drift selects entry while an invalidated release selects its separate recovery");
        if (reenter) {
            const float oldLook = view.lookHeading;
            car.driftHeld = car.driftHoldActive = false;
            car.driftPhase = DriftPhase::None;
            view.update(.000001f, car, settings);
            check(std::fabs(wrapPi(view.lookHeading - oldLook)) < .00001f,
                  "cancelling a rapid reentry cannot resurrect the earlier launch's look origin");
        }
        car.yaw = car.travelYaw = target;
        car.slipAngle = car.yawRate = 0;
        car.drifting = false;
        run(view, 2, kDt, car, settings);
        check(view.motionPhase == CameraMotionPhase::Grip && near(wrapPi(view.heading - target), 0) &&
              near(wrapPi(view.lookHeading - target), 0),
              "rapid launch interruption leaves no accumulated world-angle or optical preview tail");
    }
}

void testActualCameraDirectedLaunch() {
    const Track track = temporalTrack();
    FeelSettings settings = baselineSettings();
    for (DriftModel model : {DriftModel::Inertia, DriftModel::LineControl}) for (float side : {-1.0f, 1.0f}) {
        Car car;
        car.driftModel = model;
        car.reset(track);
        car.applyTuning(settings);
        car.speed = 80 / 3.6f;
        CameraPresentation view;
        view.update(0, car, settings);
        DriveInput input;
        input.throttle = input.handbrake = 1;
        input.steer = side * .6f;
        for (int frame = 0; frame < 180; ++frame) {
            car.update(kDt, input, track);
            view.update(kDt, car, settings);
        }
        check(car.driftChargeLevel == 3, "an actual held run charges before exercising camera-directed release");
        const float capturedLook = view.lookHeading;
        const float previousOrbit = view.heading;
        input.handbrake = 0;
        input.steer = 0;
        input.hasBoostHeading = true;
        input.boostHeading = capturedLook;
        car.update(kDt, input, track);
        view.update(kDt, car, settings);
        check(car.releaseUsesCameraHeading && car.releaseEaseActive && view.motionPhase == CameraMotionPhase::Exit,
              "a real charged release selects the captured camera target and the camera's finite exit together");
        check(std::fabs(wrapPi(view.lookHeading - capturedLook)) < 1 * kPi / 180 &&
              std::fabs(wrapPi(view.heading - previousOrbit)) < 3 * kPi / 180,
              "the real release frame does not aim back at the counterrotating nose or snap its chase orbit");
        for (int frame = 0; frame < 90; ++frame) {
            input.hasBoostHeading = false;
            input.steer = frame < 18 ? side * .2f : 0;
            const float oldLook = view.lookHeading;
            car.update(kDt, input, track);
            view.update(kDt, car, settings);
            const float target = car.releaseEaseActive ? car.releaseHeading : car.yaw;
            if (view.motionPhase == CameraMotionPhase::Exit || car.releaseEaseActive)
                check(std::fabs(wrapPi(view.lookHeading - target)) < 1 * kPi / 180,
                      "the actual vehicle and camera share one live release target across the250ms alignment boundary");
            check(std::fabs(wrapPi(view.lookHeading - oldLook)) < 5 * kPi / 180,
                  "actual launch steering and restored ordinary driving remain optically continuous");
        }
        check(!car.releaseEaseActive && view.motionPhase == CameraMotionPhase::Grip,
              "actual camera launch leaves both the body ease and camera exit after their independent durations");
    }
}

void testAirborneVerticalFraming() {
    for(int fps : {30,60,120}) {
        Car car;
        FeelSettings settings;
        CameraPresentation view;
        view.update(0,car,settings);
        float maxLag=0;
        for(int frame=1;frame<=fps*2;++frame) {
            const float t=float(frame)/fps;
            car.grounded=false;
            car.y=12*t-0.5f*9.81f*t*t;
            car.z=20*t;
            view.update(1.0f/fps,car,settings);
            maxLag=std::max(maxLag,std::fabs(car.y-view.anchor.y));
            check(std::isfinite(view.eye.y)&&std::fabs(view.lookHeading)<0.0001f,
                  "vertical flight never rotates the optical horizontal direction or poisons the camera");
        }
        check(maxLag<0.8f,"airborne camera follows vertical flight closely enough to retain the car in frame");
    }
}

int main() {
    testFraming();
    testReferenceComposition();
    testSeparateResponses();
    testBoostDirectionPreviewProjection();
    testBoostDirectionPreviewTransitions();
    testChargeLensAndTranslation();
    testChargeToBoostAndFinish();
    testSlowLensTimingAndReboost();
    testNaturalBoostExpiryAndSlowLook();
    testActualChargeBuildup();
    testEntryAndManualExit();
    testInterruptionsAndPhasePause();
    testActualCarFrameRates();
    testEventAndLookahead();
    testPauseCancelAndReset();
    testRuntimeParameters();
    testMovingPhaseReferences();
    testRapidManualInterruptions();
    testLiveCameraSettings();
    testIndependentBoostDolly();
    testBoostCameraHoldsAndIntent();
    testNormalSteeringPreview();
    testActualNormalSteeringPreview();
    testCameraDirectedLaunchContinuity();
    testCameraDirectedLaunchSteeringAndLiveSettings();
    testCameraDirectedLaunchInterruptions();
    testActualCameraDirectedLaunch();
    testAirborneVerticalFraming();
    if (failures) return EXIT_FAILURE;
    std::puts("presentation tests passed: reference framing, camera-directed launch, continuous optical exits, real-car frame rates, bounded charge/release lens, translation-only shake, pause/cancel/reset/replay");
    return EXIT_SUCCESS;
}
