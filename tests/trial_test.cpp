#include "trial.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>

namespace {
constexpr float dt = 1.0f / 60.0f;
constexpr float radians = 0.0174532925199433f;
int failures = 0;

void check(bool condition, const char* label) {
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", label);
        ++failures;
    }
}

void sample(TrialSession& trial, float s, float degrees = 0, float speedKmh = 80,
            bool asphalt = true, float impact = 0) {
    Car car;
    car.ground.s = s;
    car.speed = speedKmh / 3.6f;
    car.slipAngle = degrees * radians;
    car.onAsphalt = asphalt;
    car.impact = impact;
    DriveInput input;
    input.throttle = 0.5f;
    input.steer = -0.25f;
    trial.tick(car, input, dt);
}

void enter(TrialSession& trial) {
    sample(trial, 34);
    sample(trial, 35);
}

void hold(TrialSession& trial, int frames = 90, float angle = 50, float startS = 40) {
    for (int i = 0; i < frames; ++i) sample(trial, startS + i * 0.2f, angle);
}

void finish(TrialSession& trial, float angle = 7.9f, float speed = 65) {
    // Traverse forward in small increments; shortcuts cannot count as a gate crossing.
    float s = trial.samples.empty() ? 15 : trial.samples.back().s;
    while (s < TrialCourse::finishS + 0.05f) {
        s += 1;
        sample(trial, s, angle, speed);
    }
}

void geometry() {
    Track track;
    track.makeTestCorner();
    check(!track.closed && std::fabs(track.halfWidth * 2 - 13) < 0.001f,
          "comparison road is open and 13m wide");
    check(std::fabs(track.length - (35 + 50 * 1.5707963267948966f + 45)) < 0.001f,
          "R50 90 degree course has the expected analytic length");
    auto spawn = track.query(track.spawnX, track.spawnZ, 0);
    check(spawn.found && std::fabs(spawn.s - 15) < 0.001f && std::fabs(spawn.yaw) < 0.001f,
          "spawn is 20m before the corner with forward heading");
    for (int i = 0; i <= 10; ++i) {
        float t = 1.5707963267948966f * i / 10;
        float x = 50 - 50 * std::cos(t);
        float z = 50 * std::sin(t);
        auto pose = track.query(x, z, 0, 35 + 50 * t);
        check(std::fabs(pose.s - (35 + 50 * t)) < 0.002f && pose.xzDist < 0.002f,
              "arc query follows the generated circle without closing-road interference");
        check(std::fabs(wrapPi(pose.yaw - t)) < 0.018f, "arc heading matches right turn tangent");
    }
    auto exit = track.query(62, 50, 0, TrialCourse::finishS);
    check(std::fabs(exit.s - TrialCourse::finishS) < 0.002f && std::fabs(exit.yaw - 1.5707963f) < 0.001f,
          "finish gate is 12m after the curve");
    auto endpoint = track.query(95, 50, 0);
    check(std::fabs(endpoint.s - track.length) < 0.002f, "open endpoint does not wrap to start");
    check(!track.visual.indices.empty(), "test course produces visible road geometry");
}

void scaledGeometry() {
    Track original;
    original.makeTestCorner();
    original.originLat=34.8f;
    original.originLon=136.5f;
    Track enlarged=original;
    enlarged.scaleBy(1.4f);
    for(float angle : {0.1f,0.5f,1.0f,1.5f}) {
        const float x=50-50*std::cos(angle),z=50*std::sin(angle);
        const float s=35+50*angle;
        const auto before=original.query(x,z,0,s);
        const auto after=enlarged.query(x*1.4f,z*1.4f,0,s*1.4f);
        check(after.found && after.xzDist<0.003f && std::fabs(after.s-before.s*1.4f)<0.003f &&
              std::fabs(wrapPi(after.yaw-before.yaw))<0.0001f,
              "scaled corner keeps its analytic radius and tangent at corresponding world positions");
    }
    const auto originalOutside=original.query(8,0,0,20);
    const auto enlargedInside=enlarged.query(8,0,0,28);
    check(std::fabs(originalOutside.lateral)>original.halfWidth &&
          std::fabs(enlargedInside.lateral)<enlarged.halfWidth,
          "enlarging the stage creates usable road width rather than changing only its rendering");
    const auto oldWall=original.query(original.wallLateral()+0.1f,0,0,20);
    const auto newWall=enlarged.query((original.wallLateral()+0.1f)*1.4f,0,0,28);
    check(std::fabs(oldWall.lateral)>original.wallLateral() &&
          std::fabs(newWall.lateral)>enlarged.wallLateral() &&
          std::fabs(newWall.rightX-oldWall.rightX)<0.0001f,
          "scaled wall query retains the same outward side and scales the collision boundary with the road");
    bool visualMatches=enlarged.visual.vertices.size()==original.visual.vertices.size();
    for(size_t i=0;i<original.visual.vertices.size();++i) {
        const float expected=original.visual.vertices[i]*(i%9<3?1.4f:1.0f);
        visualMatches&=std::fabs(enlarged.visual.vertices[i]-expected)<0.0001f;
    }
    check(visualMatches && enlarged.visual.indices==original.visual.indices &&
          enlarged.originLat==original.originLat && enlarged.originLon==original.originLon,
          "stage mesh positions scale with the surface while normals, colours, topology and geographic origin remain intact");

    // Two crossing roads at different heights exercise the same height/preferS
    // selection used by Suzuka's overpass, independently of external data files.
    Track bridge;
    bridge.samples={{0,-60,0,.025f,0},{0,60,0,.025f,0},
                    {60,60,6,-.03f,0},{60,0,6,-.03f,0},
                    {-60,0,6,-.03f,0},{-60,-60,0,.025f,0}};
    bridge.samples[3].bridge=bridge.samples[4].bridge=true;
    for(size_t i=0;i<bridge.samples.size();++i) {
        auto& a=bridge.samples[i];
        const auto& b=bridge.samples[(i+1)%bridge.samples.size()];
        a.s=bridge.length;
        bridge.length+=std::hypot(b.x-a.x,b.z-a.z);
    }
    const float lowerS=60,upperS=bridge.samples[3].s+60;
    const auto lower=bridge.query(0,0,0,lowerS),upper=bridge.query(0,0,6,upperS);
    bridge.scaleBy(2.5f);
    const auto lowerScaled=bridge.query(0,0,0,lowerS*2.5f);
    const auto upperScaled=bridge.query(0,0,15,upperS*2.5f);
    check(lower.centerY==0 && upper.centerY==6 && lowerScaled.centerY==0 && upperScaled.centerY==15 &&
          std::fabs(lowerScaled.s-lower.s*2.5f)<0.0001f &&
          std::fabs(upperScaled.s-upper.s*2.5f)<0.0001f &&
          upperScaled.cross==upper.cross && bridge.samples[3].bridge,
          "uniform scaling preserves upper/lower overpass selection, elevation, banking and route continuity");

    enlarged.scaleBy(1/1.4f);
    bool restored=enlarged.visual.indices==original.visual.indices;
    for(size_t i=0;i<original.visual.vertices.size();++i)
        restored&=std::fabs(enlarged.visual.vertices[i]-original.visual.vertices[i])<0.0001f;
    const auto restoredSpawn=enlarged.query(enlarged.spawnX,enlarged.spawnZ,enlarged.spawnY);
    check(restored && std::fabs(enlarged.length-original.length)<0.0001f &&
          std::fabs(restoredSpawn.s-15)<0.001f && enlarged.spawnYaw==original.spawnYaw,
          "scaling back restores the rendered course and spawn-to-surface relationship without changing heading");
    const auto vertices=enlarged.visual.vertices;
    const float length=enlarged.length,width=enlarged.wallLateral();
    for(float invalid : {0.0f,-1.0f,std::numeric_limits<float>::quiet_NaN(),
                         std::numeric_limits<float>::infinity()}) enlarged.scaleBy(invalid);
    check(enlarged.visual.vertices==vertices && enlarged.length==length && enlarged.wallLateral()==width,
          "zero, negative and nonfinite stage ratios leave valid geometry and collision dimensions unchanged");
}

bool meshColor(const std::vector<float>& vertices, size_t index, Vec3 color) {
    return std::fabs(vertices[index+6]-color.x)<.00001f &&
           std::fabs(vertices[index+7]-color.y)<.00001f &&
           std::fabs(vertices[index+8]-color.z)<.00001f;
}

void speedReferenceGeometry() {
    Track road;
    road.makeTestCorner();
    int reflectors=0, edgePaint=0;
    bool outsideRoad=true, lowPosts=true, paintOnRoad=true, leftStart=false, rightStagger=false;
    for(size_t index=0;index+8<road.visual.vertices.size();index+=9) {
        const auto& v=road.visual.vertices;
        if(meshColor(v,index,{1,.58f,.10f})) {
            ++reflectors;
            const auto surface=road.query(v[index],v[index+2],0);
            outsideRoad&=std::fabs(surface.lateral)>road.halfWidth+road.curbWidth+.25f &&
                         std::fabs(surface.lateral)<road.wallLateral()-.5f;
            lowPosts&=v[index+1]>=.47f && v[index+1]<=.62f;
            leftStart|=surface.lateral<0 && std::fabs(surface.s)<.1f;
            rightStagger|=surface.lateral>0 && std::fabs(surface.s-3)<.1f;
        }
        if(meshColor(v,index,{.80f,.82f,.78f})) {
            ++edgePaint;
            const auto surface=road.query(v[index],v[index+2],0);
            paintOnRoad&=std::fabs(surface.lateral)<road.halfWidth &&
                         std::fabs(surface.lateral)>road.halfWidth-.5f &&
                         std::fabs(v[index+1]-.025f)<.00001f;
        }
    }
    check(reflectors>200 && outsideRoad && lowPosts && leftStart && rightStagger,
          "roadside reflectors form a low staggered near-field rhythm outside the usable asphalt and inside its existing walls");
    check(edgePaint>100 && paintOnRoad,
          "short high-contrast edge marks stay on the pavement without raising the driving surface");
    check(road.visual.vertices.size()/9<16000 && road.visual.indices.size()/3<10000,
          "road references remain a modest static mesh rather than a large collection of separately drawn objects");

    Track arena;
    arena.makePracticeArea();
    size_t paintVertices=0;
    bool local=true, clearLaunch=true, flatPaint=true;
    for(size_t index=0;index+8<arena.visual.vertices.size();index+=9) {
        const auto& v=arena.visual.vertices;
        if(!meshColor(v,index,{.58f,.61f,.57f})) continue;
        ++paintVertices;
        local&=std::fabs(v[index])<161 && std::fabs(v[index+2])<161;
        clearLaunch&=std::fabs(v[index])>=8;
        flatPaint&=std::fabs(v[index+1]-.025f)<.00001f;
        const auto support=arena.query(v[index],v[index+2],v[index+1]);
        flatPaint&=support.found && support.centerY==0 && support.cross==0 &&
                   std::fabs(support.lateral)<arena.halfWidth;
    }
    check(paintVertices>2800 && paintVertices<3600 && local && clearLaunch && flatPaint,
          "practice speed marks are limited to the central160m patch, leave the launch strip open and never change its flat support");
    const auto original=arena.visual.vertices;
    arena.scaleBy(1.4f);
    bool scaled=arena.visual.vertices.size()==original.size();
    for(size_t index=0;index<original.size();++index)
        scaled&=std::fabs(arena.visual.vertices[index]-original[index]*(index%9<3?1.4f:1))<.0002f;
    check(scaled,"practice markings and roadside landmarks scale with the stage while retaining their vertex colours and normals");
}

void unsmoothedRoadReferences() {
    // A sharp dip and bank changes verify raw loaded geometry independently of
    // the flat fixtures. Cosmetic references must not resample the driving data.
    const auto path=std::filesystem::temp_directory_path()/
        ("racegame-reference-test-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())+".txt");
    std::vector<TrackSample> expected;
    {
        std::ofstream file(path);
        file<<std::setprecision(9)<<"half_width 6.5\ncurb_width 0.8\nspawn 0 0 0 0\nnames 1\n0 ORIGINAL\nsamples 16\n";
        for(int index=0;index<16;++index) {
            const float angle=index*6.28318530718f/16;
            TrackSample point;
            point.x=50*std::cos(angle); point.z=50*std::sin(angle);
            point.y=index==4?-3.0f:index==5?4.0f:float(index%3)*.25f;
            point.cross=index%2?.04f:-.02f;
            expected.push_back(point);
            file<<point.x<<' '<<point.z<<' '<<point.y<<' '<<point.cross<<" 0 0 0\n";
        }
    }
    Track loaded;
    const bool opened=loaded.load(path.string());
    std::filesystem::remove(path);
    check(opened && loaded.samples.size()==expected.size(),"raw elevated course loads with its cosmetic speed references");
    if(!opened || loaded.samples.size()!=expected.size()) return;
    bool raw=true;
    for(size_t index=0;index<expected.size();++index) {
        const auto& point=loaded.samples[index];
        raw&=point.x==expected[index].x && point.z==expected[index].z &&
             point.y==expected[index].y && point.cross==expected[index].cross;
    }
    const auto& a=loaded.samples[4];
    const auto& b=loaded.samples[5];
    const float midpointY=(a.y+b.y)*.5f;
    const auto support=loaded.query((a.x+b.x)*.5f,(a.z+b.z)*.5f,midpointY,(a.s+b.s)*.5f);
    check(raw && support.found && std::fabs(support.centerY-midpointY)<.0001f &&
          loaded.samples[4].y==-3 && loaded.samples[5].y==4,
          "near-field props preserve original dips, crests, banks and the unsmoothed piecewise road support exactly");
}

void successAndRecording() {
    TrialSession trial;
    enter(trial);
    hold(trial, 90, 35); // inclusive lower angle boundary and exact 1.5-second hold
    finish(trial);
    check(trial.metrics.finished && trial.metrics.passed && !trial.metrics.timedOut,
          "uninterrupted valid hold and clean 65kmh exit passes");
    check(std::fabs(trial.metrics.holdSeconds - 1.5f) < 0.0002f,
          "hold duration is measured in fixed simulation time");
    check(std::fabs(trial.metrics.entrySpeedKmh - 80) < 0.001f &&
          std::fabs(trial.metrics.exitSpeedKmh - 65) < 0.001f,
          "entry and exit speeds use kmh at their respective gates");
    check(trial.samples[2].input.throttle == 0.5f && trial.samples[2].input.steer == -0.25f &&
          trial.samples[2].speed > 0 && trial.samples[2].slipSpeed > 0,
          "per-step recording retains replay input and actual motion telemetry");
    const auto count = trial.samples.size();
    const float duration = trial.metrics.duration;
    sample(trial, 140, 0, 0, false, 100);
    check(trial.samples.size() == count && trial.metrics.duration == duration && trial.metrics.passed,
          "finished trial cannot be changed by later simulation steps");
    trial.reset();
    check(trial.samples.empty() && !trial.metrics.finished && trial.metrics.contacts == 0,
          "reset clears recording and all result state");
}

void failureConditions() {
    TrialSession split;
    enter(split);
    hold(split, 50, 50, 40);
    sample(split, 51, 34.9f);
    hold(split, 50, 50, 52);
    finish(split);
    check(!split.metrics.passed && split.metrics.holdSeconds < 1,
          "two short holds do not combine into a contiguous 1.5-second hold");

    TrialSession afterCorner;
    enter(afterCorner);
    for (float s = 40; s < TrialCourse::cornerExitS + 1; s += 1) sample(afterCorner, s);
    hold(afterCorner, 120, 50, TrialCourse::cornerExitS + 1);
    finish(afterCorner);
    check(afterCorner.metrics.holdSeconds == 0 && !afterCorner.metrics.passed,
          "sliding on the exit cannot replace holding through the corner");

    TrialSession tooSlow;
    enter(tooSlow);
    hold(tooSlow, 90, 70); // upper angle boundary counts
    finish(tooSlow, 7, 64.9f);
    check(!tooSlow.metrics.passed && tooSlow.metrics.holdSeconds > 1.49f,
          "upper hold angle counts but exit below 65kmh fails");

    TrialSession tooSideways;
    enter(tooSideways);
    hold(tooSideways);
    finish(tooSideways, 8.01f, 80);
    check(!tooSideways.metrics.passed, "exit angle at or above 8 degrees fails");

    for (float angle : {34.9f, 70.1f, -34.9f, -70.1f}) {
        TrialSession outOfRange;
        enter(outOfRange);
        hold(outOfRange, 120, angle);
        finish(outOfRange);
        check(!outOfRange.metrics.passed && outOfRange.metrics.holdSeconds == 0,
              "slip just outside either 35-70 degree boundary cannot count on either side");
    }

    for (float angle : {35.0f, 70.0f, -35.0f, -70.0f}) {
        TrialSession boundary;
        enter(boundary);
        hold(boundary, 90, angle);
        finish(boundary);
        check(boundary.metrics.passed && std::fabs(boundary.metrics.holdSeconds - 1.5f) < 0.0002f,
              "both inclusive 35-70 degree boundaries count for left and right slides");
    }
}

void contactsAndOffroad() {
    TrialSession trial;
    enter(trial);
    for (int i = 0; i < 30; ++i) sample(trial, 36 + i * 0.1f, 50, 80, false, 5);
    check(trial.metrics.contacts == 1 && trial.metrics.offroad == 1,
          "sustained wall scraping and offroad count as single episodes");
    for (int i = 0; i < 15; ++i) sample(trial, 40 + i * 0.1f);
    sample(trial, 42, 50, 80, false, 5);
    check(trial.metrics.contacts == 2 && trial.metrics.offroad == 2,
          "a later distinct departure and impact are counted again");
    for (int i = 0; i < 15; ++i) sample(trial, 43 + i * 0.1f);
    hold(trial, 90, 50, 46);
    finish(trial);
    check(!trial.metrics.passed, "clean recovery cannot erase previous contact/offroad failures");
}

void timeoutAndInvalidStep() {
    TrialSession trial;
    Car car;
    trial.tick(car, {}, 0);
    trial.tick(car, {}, -1);
    trial.tick(car, {}, std::numeric_limits<float>::quiet_NaN());
    check(trial.samples.empty() && trial.metrics.duration == 0, "invalid step durations do not mutate trial state");
    for (int i = 0; i < 910; ++i) sample(trial, 15);
    check(trial.metrics.finished && trial.metrics.timedOut && !trial.metrics.passed,
          "a stalled run times out after 15 seconds");
    check(trial.metrics.duration >= 15 && trial.metrics.duration < 15.02f,
          "timeout happens within one fixed step");

    trial.reset();
    sample(trial, 130); // cannot obtain a valid finish by spawning beyond the gate
    sample(trial, 131);
    check(!trial.metrics.finished, "finish requires a forward entry and finish-gate crossing");
}

void practiceGeometry() {
    Track arena;
    arena.makePracticeArea();
    check(arena.practiceArea && !arena.closed && arena.halfWidth >= 1000,
          "free practice is a broad flat area rather than a narrow comparison course");
    check(arena.spawnX == 0 && arena.spawnY == 0 && arena.spawnZ == 0 && arena.spawnYaw == 0,
          "practice starts in the middle with a reproducible forward pose");
    for (float x : {-999.0f, -400.0f, 0.0f, 400.0f, 999.0f}) {
        for (float z : {-999.0f, -400.0f, 0.0f, 400.0f, 999.0f}) {
            const auto pose = arena.query(x, z, 4, 1234);
            check(pose.found && pose.centerY == 0 && pose.cross == 0 &&
                  std::fabs(pose.lateral) < arena.halfWidth,
                  "all quadrants and far lateral positions remain flat asphalt");
            check(std::fabs(pose.rightX * pose.rightX + pose.rightZ * pose.rightZ - 1) < 0.001f,
                  "practice boundary normals stay normalized for collision response");
        }
    }
    auto east = arena.query(1001, 50, 0);
    auto north = arena.query(50, 1001, 0);
    auto west = arena.query(-1001, 50, 0);
    auto south = arena.query(50, -1001, 0);
    check(east.lateral > arena.wallLateral() && east.rightX == 1 && east.rightZ == 0 &&
          west.lateral < -arena.wallLateral() && west.rightX == 1 && west.rightZ == 0,
          "east and west outer limits report the correct collision axis");
    check(north.lateral > arena.wallLateral() && north.rightX == 0 && north.rightZ == 1 &&
          south.lateral < -arena.wallLateral() && south.rightX == 0 && south.rightZ == 1,
          "north and south outer limits report the correct collision axis");
    check(!arena.query(std::numeric_limits<float>::quiet_NaN(), 0, 0).found,
          "invalid positions cannot produce a valid practice surface");
    check(!arena.visual.indices.empty(), "free practice has visible pavement and ground references");
    arena.makeTestCorner();
    check(!arena.practiceArea, "returning to the comparison course restores its regular road query");
}

void practiceRecording() {
    TrialSession trial;
    Car car;
    car.speed = 80.0f / 3.6f;
    car.slipAngle = 50 * radians;
    car.onAsphalt = true;
    car.ground.s = TrialCourse::finishS + 20;
    DriveInput input;
    input.throttle = input.handbrake = 1;
    input.steer = -0.5f;
    input.hasBoostHeading = true;
    input.boostHeading = 3.12f;
    trial.tickPractice(car, input, 0);
    trial.tickPractice(car, input, std::numeric_limits<float>::infinity());
    check(trial.samples.empty() && trial.metrics.duration == 0,
          "invalid practice step times do not mutate measurements");
    for (int i = 0; i < 1200; ++i) {
        car.ground.s += 1;
        trial.tickPractice(car, input, dt);
    }
    check(trial.metrics.duration > 19.9f && !trial.metrics.finished &&
          !trial.metrics.timedOut && !trial.metrics.passed,
          "practice continues beyond both the old finish gate and 15-second timeout");
    check(trial.metrics.holdSeconds > 19.9f && trial.metrics.entrySpeedKmh > 79.99f &&
          std::fabs(trial.metrics.exitSpeedKmh - 80) < 0.001f,
          "practice records actual sliding and current speed without a success target");
    car.slipAngle = 9 * radians;
    trial.tickPractice(car, input, dt);
    const float validHold = trial.metrics.holdSeconds;
    check(validHold > 20, "practice sliding includes useful angles outside the old scoring band");
    car.slipAngle = 0;
    trial.tickPractice(car, input, dt);
    car.slipAngle = 50 * radians;
    trial.tickPractice(car, input, dt);
    check(trial.metrics.holdSeconds == validHold,
          "an interruption does not merge separate free-practice slides");
    const auto& recorded = trial.samples.back();
    check(recorded.input.throttle == 1 && recorded.input.handbrake == 1 && recorded.input.steer == -0.5f &&
          recorded.input.hasBoostHeading && recorded.input.boostHeading == 3.12f &&
          recorded.speed == car.speed && recorded.slipSpeed > 0,
          "practice preserves controls, captured launch heading and motion samples for later inspection");

    car.impact = 5;
    car.onAsphalt = false;
    for (int i = 0; i < 30; ++i) trial.tickPractice(car, input, dt);
    check(trial.metrics.contacts == 1 && trial.metrics.offroad == 1,
          "practice counts contact and offroad episodes without ending the session");
    car.impact = 0;
    car.onAsphalt = true;
    for (int i = 0; i < 15; ++i) trial.tickPractice(car, input, dt);
    car.impact = 5;
    car.onAsphalt = false;
    trial.tickPractice(car, input, dt);
    check(trial.metrics.contacts == 2 && trial.metrics.offroad == 2 && !trial.metrics.finished,
          "later practice incidents are counted while manual driving continues");

    trial.reset();
    car.impact = 0;
    car.onAsphalt = true;
    for (std::size_t i = 0; i < PracticeSession::maxSamples; ++i) trial.tickPractice(car, input, dt);
    check(trial.samples.size() == PracticeSession::maxSamples && !trial.samplesTruncated,
          "practice records samples up to its memory bound");
    const float lastRecordedTime = trial.samples.back().time;
    const float priorDuration = trial.metrics.duration;
    for (int i = 0; i < 60; ++i) trial.tickPractice(car, input, dt);
    check(trial.samples.size() == PracticeSession::maxSamples && trial.samplesTruncated &&
          trial.samples.back().time == lastRecordedTime && trial.metrics.duration > priorDuration + 0.9f &&
          !trial.metrics.finished && !trial.metrics.timedOut,
          "reaching the recording limit bounds memory but never stops practice or current measurements");
    trial.reset();
    check(trial.samples.empty() && !trial.samplesTruncated && trial.metrics.duration == 0,
          "manual practice reset clears measurements and recording-limit state");
}
void airborneRecording() {
    TrialSession trial;
    Car car;
    DriveInput input;
    car.speed=25;car.slipAngle=50*3.14159265f/180;
    trial.tickPractice(car,input,1.0f/60);
    const float groundedHold=trial.metrics.holdSeconds;
    car.grounded=false;car.onAsphalt=false;car.y=3;car.verticalSpeed=-4;
    for(int i=0;i<60;++i) trial.tickPractice(car,input,1.0f/60);
    check(trial.metrics.offroad==0&&trial.metrics.holdSeconds==groundedHold,
          "flight over grass counts neither road departure contact nor sustained tire drift");
    const auto& airborne=trial.samples.back();
    check(!airborne.grounded&&airborne.height==3&&airborne.verticalSpeed==-4,
          "flight telemetry retains physical height and vertical momentum");
    car.grounded=true;car.verticalSpeed=0;
    trial.tickPractice(car,input,1.0f/60);
    check(trial.metrics.offroad==1,"landing on grass begins one offroad contact episode");
}
} // namespace

int main() {
    geometry();
    scaledGeometry();
    speedReferenceGeometry();
    unsmoothedRoadReferences();
    successAndRecording();
    failureConditions();
    contactsAndOffroad();
    timeoutAndInvalidStep();
    practiceGeometry();
    practiceRecording();
    airborneRecording();
    if (failures) return EXIT_FAILURE;
    std::puts("trial tests passed: R50 regression, static speed references, original elevations, scaled surface/mesh/walls/overpass, free-practice geometry/continuous driving, bounded recording, contacts/offroad, reset");
    return EXIT_SUCCESS;
}
