#pragma once

#include "car.hpp"
#include "meshbuild.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <vector>

// Presentation-only flourishes: tail-light ribbons, sparks, release flashes and
// camera shake. Nothing here feeds back into the car's physics.
struct DriftEffects {
    struct TrailPoint { Vec3 left, right; Vec3 color; float age = 0; float life = 0.3f; bool breakAfter = false; };
    struct Spark { Vec3 position, velocity, color; float age = 0, life = 0.4f, size = 0.05f; };
    struct Flash { Vec3 position; int level = 1; float age = 0, life = 0.4f, radius = 4; };

    std::deque<TrailPoint> trail[2];
    std::vector<Spark> sparks;
    std::vector<Flash> flashes;
    float shake = 0;          // Camera shake amplitude, metres.
    float shakeClock = 0;
    float boostGlow = 0;      // Extra tail-light intensity after a release, 0..1.

    // Charge colours match the existing tyre rings: blue, orange, pink.
    static Vec3 levelColor(int level) {
        static const Vec3 colors[4] = {{1.0f, 0.10f, 0.08f}, {0.15f, 0.55f, 1.0f}, {1.0f, 0.45f, 0.06f}, {1.0f, 0.18f, 0.75f}};
        return colors[std::clamp(level, 0, 3)];
    }

    void reset() {
        trail[0].clear(); trail[1].clear();
        sparks.clear(); flashes.clear();
        shake = shakeClock = boostGlow = 0;
        observedBoost_ = observedLanding_ = 0;
        wasEntry_ = false;
        primed_ = false;
        sparkCarry_ = 0;
    }

    // body: the rendered sprung-body matrix. camera: current eye position.
    void tick(float dt, const Car& car, const Mat4& body, Vec3 camera) {
        if (!primed_) { observedBoost_ = car.boostSerial; observedLanding_ = car.landingSerial; primed_ = true; }
        for (auto& side : trail) {
            for (auto& p : side) p.age += dt;
            while (!side.empty() && side.front().age >= side.front().life) side.pop_front();
        }
        for (auto& s : sparks) {
            s.age += dt;
            s.velocity.y -= 9.0f * dt;
            s.velocity = s.velocity * std::exp(-2.2f * dt);
            s.position = s.position + s.velocity * dt;
        }
        sparks.erase(std::remove_if(sparks.begin(), sparks.end(), [](const Spark& s) { return s.age >= s.life; }), sparks.end());
        for (auto& f : flashes) f.age += dt;
        flashes.erase(std::remove_if(flashes.begin(), flashes.end(), [](const Flash& f) { return f.age >= f.life; }), flashes.end());
        shake *= std::exp(-9.0f * dt);
        if (shake < 0.0005f) shake = 0;
        shakeClock = std::fmod(shakeClock + dt, 100.0f);
        boostGlow = std::fmax(0.0f, boostGlow - dt * 1.4f);

        const Vec3 forward{std::sin(car.yaw), 0, std::cos(car.yaw)};
        const Vec3 right{std::cos(car.yaw), 0, -std::sin(car.yaw)};
        const Vec3 rearLeft = transformPoint(body, {-0.62f, 0.52f, -2.18f});
        const Vec3 rearRight = transformPoint(body, {0.62f, 0.52f, -2.18f});
        const float kmh = std::fabs(car.speed) * 3.6f;

        // Tail-light ribbons: faint red at speed, charge-coloured while sliding,
        // long and bright while a boost runs.
        const bool boosting = car.boostRemaining > 0 && car.boostLevel > 0;
        const int charge = boosting ? car.boostLevel : (car.driftHoldActive ? car.driftChargeLevel : 0);
        float intensity = clampf((kmh - 120) / 80, 0, 1) * 0.22f;
        float life = 0.14f;
        if (car.drifting && car.driftHoldActive) { intensity = std::fmax(intensity, 0.55f + 0.15f * charge); life = 0.45f + 0.1f * charge; }
        if (boosting) { intensity = std::fmax(intensity, 0.75f + 0.25f * boostGlow); life = 0.7f; }
        const Vec3 color = levelColor(charge) * intensity;
        const bool emitting = intensity > 0.02f && car.speed > 3;
        const Vec3 lights[2] = {rearLeft, rearRight};
        for (int i = 0; i < 2; ++i) {
            auto& side = trail[i];
            if (!emitting) { if (!side.empty()) side.back().breakAfter = true; continue; }
            Vec3 toCamera = camera - lights[i];
            Vec3 along = side.empty() ? forward * -1.0f : lights[i] - side.back().left * 0.5f - side.back().right * 0.5f;
            Vec3 width = cross(along, toCamera);
            const float len = length(width);
            width = len > 1e-4f ? width * (1.0f / len) : Vec3{0, 1, 0};
            const float half = 0.028f + 0.04f * clampf(intensity, 0, 1);
            side.push_back({lights[i] - width * half, lights[i] + width * half, color, 0, life, false});
            if (side.size() > 240) side.pop_front();
        }

        // Drift sparks fly from the outer rear tyre once a charge stage is lit.
        const bool sliding = car.grounded && car.onAsphalt && car.drifting && car.driftHoldActive && car.speed > 8;
        if (sliding && car.driftChargeLevel > 0) {
            sparkCarry_ += dt * (25.0f + 35.0f * car.driftChargeLevel);
            const float outward = car.driftDirection > 0 ? -1.0f : 1.0f;
            const Vec3 tyre = Vec3{car.x, car.y + 0.08f, car.z} + forward * -1.38f + right * (outward * 0.9f);
            const Vec3 travel{std::sin(car.travelYaw), 0, std::cos(car.travelYaw)};
            while (sparkCarry_ >= 1) {
                sparkCarry_ -= 1;
                const Vec3 kick = travel * (car.speed * 0.55f) + right * (outward * random(1.5f, 4.5f)) +
                                  Vec3{0, random(1.5f, 4.0f), 0} + forward * random(-2.5f, -0.5f);
                spawn(tyre, kick, mix(levelColor(car.driftChargeLevel), {1, 1, 0.9f}, random(0, 0.35f)), random(0.18f, 0.38f), 0.045f);
            }
        } else {
            sparkCarry_ = 0;
        }

        // Release: a ground flash, a fan of sparks and a firm camera kick.
        if (car.boostSerial != observedBoost_) {
            observedBoost_ = car.boostSerial;
            if (car.boostLevel > 0) {
                const int level = car.boostLevel;
                flashes.push_back({{car.x, car.y + 0.06f, car.z}, level, 0, 0.32f + 0.06f * level, 3.5f + 1.5f * level});
                const Vec3 tail = (rearLeft + rearRight) * 0.5f;
                const Vec3 travel{std::sin(car.travelYaw), 0, std::cos(car.travelYaw)};
                for (int n = 0; n < 24 + 16 * level; ++n) {
                    const float a = random(0, 6.2831853f);
                    const Vec3 spray = forward * random(-9.0f, -3.0f) + right * (std::cos(a) * random(2, 7)) +
                                       Vec3{0, std::fabs(std::sin(a)) * random(2, 6), 0} + travel * (car.speed * 0.6f);
                    spawn(tail, spray, mix(levelColor(level), {1, 1, 1}, random(0, 0.5f)), random(0.25f, 0.55f), 0.06f);
                }
                shake = std::fmax(shake, 0.05f + 0.035f * level);
                boostGlow = 1;
            }
        }

        // Entering a slide snaps the camera a touch so the kick is felt.
        const bool entry = car.driftPhase == DriftPhase::Entry;
        if (entry && !wasEntry_) shake = std::fmax(shake, 0.035f);
        wasEntry_ = entry;

        // Barrier scrapes throw orange sparks off the side that touched.
        if (car.impact > 1.5f) {
            const float side = car.ground.lateral > 0 ? 1.0f : -1.0f;
            const Vec3 contact = Vec3{car.x, car.y + 0.4f, car.z} + right * 0.0f +
                                 Vec3{car.ground.rightX, 0, car.ground.rightZ} * (side * 0.95f);
            const Vec3 travel{std::sin(car.travelYaw), 0, std::cos(car.travelYaw)};
            const int count = std::min(40, int(4 + car.impact * 1.5f));
            for (int n = 0; n < count; ++n) {
                const Vec3 v = travel * (car.speed * random(0.3f, 0.8f)) +
                               Vec3{car.ground.rightX, 0, car.ground.rightZ} * (-side * random(1, 5)) + Vec3{0, random(1, 5), 0};
                spawn(contact, v, mix(Vec3{1, 0.55f, 0.12f}, {1, 0.95f, 0.6f}, random(0, 0.6f)), random(0.2f, 0.5f), 0.05f);
            }
            shake = std::fmax(shake, clampf(car.impact * 0.006f, 0.02f, 0.16f));
        }

        // A hard landing thumps the view.
        if (car.landingSerial != observedLanding_) {
            observedLanding_ = car.landingSerial;
            if (car.landingImpact > 1.0f) shake = std::fmax(shake, clampf(car.landingImpact * 0.012f, 0.02f, 0.12f));
        }
        if (sparks.size() > 900) sparks.erase(sparks.begin(), sparks.begin() + (sparks.size() - 900));
    }

    Vec3 shakeOffset() const {
        if (shake <= 0) return {};
        const float t = shakeClock;
        return Vec3{std::sin(t * 71.0f) + 0.5f * std::sin(t * 113.0f),
                    std::sin(t * 89.0f + 1.3f) * 0.8f,
                    std::sin(t * 97.0f + 2.1f) * 0.5f} * shake;
    }

    // Additive ribbon/streak geometry in world space; colours carry the fade.
    void buildGlow(CpuMesh& mesh, Vec3 camera) const {
        for (const auto& side : trail) {
            for (size_t i = 1; i < side.size(); ++i) {
                const auto& a = side[i - 1];
                const auto& b = side[i];
                if (a.breakAfter) continue;
                const float fa = fade(a), fb = fade(b);
                if (fa <= 0.001f && fb <= 0.001f) continue;
                addQuadColors(mesh, a.left, a.right, b.right, b.left, a.color * fa, b.color * fb);
            }
        }
        for (const auto& s : sparks) {
            const float k = 1 - s.age / s.life;
            const Vec3 tail = s.position - s.velocity * 0.025f;
            Vec3 width = cross(s.position - tail, camera - s.position);
            const float len = length(width);
            if (len < 1e-5f) continue;
            width = width * (s.size / len);
            const Vec3 c = s.color * (0.4f + 0.6f * k);
            addQuadColors(mesh, tail - width, tail + width, s.position + width, s.position - width, c * 0.2f, c);
        }
    }

    // Flat expanding discs: level, model matrix and opacity for drawGlow(radial).
    template <class Fn>
    void forEachFlash(Fn&& fn) const {
        for (const auto& f : flashes) {
            const float t = f.age / f.life;
            const float r = f.radius * (0.35f + 0.65f * (1 - (1 - t) * (1 - t)));
            fn(f.level, basis(f.position, Vec3{r, 0, 0}, Vec3{0, 0, r}, Vec3{0, 1, 0}), 0.9f * (1 - t) * (1 - t));
        }
    }

private:
    unsigned observedBoost_ = 0, observedLanding_ = 0;
    bool wasEntry_ = false, primed_ = false;
    float sparkCarry_ = 0;
    std::uint32_t seed_ = 0x9e3779b9u;

    float random(float low, float high) {
        seed_ ^= seed_ << 13; seed_ ^= seed_ >> 17; seed_ ^= seed_ << 5;
        return low + (high - low) * float(seed_ & 0xffffff) / float(0xffffff);
    }
    void spawn(Vec3 p, Vec3 v, Vec3 c, float life, float size) { sparks.push_back({p, v, c, 0, life, size}); }
    static Vec3 mix(Vec3 a, Vec3 b, float t) { return a + (b - a) * t; }
    static float fade(const TrailPoint& p) {
        const float t = clampf(p.age / p.life, 0, 1);
        return (1 - t) * (1 - t);
    }
    static Vec3 transformPoint(const Mat4& m, Vec3 p) {
        return {m.m[0] * p.x + m.m[4] * p.y + m.m[8] * p.z + m.m[12],
                m.m[1] * p.x + m.m[5] * p.y + m.m[9] * p.z + m.m[13],
                m.m[2] * p.x + m.m[6] * p.y + m.m[10] * p.z + m.m[14]};
    }
    static void addQuadColors(CpuMesh& mesh, Vec3 a, Vec3 b, Vec3 c, Vec3 d, Vec3 ca, Vec3 cb) {
        const unsigned base = unsigned(mesh.vertices.size() / 9);
        const Vec3 n{0, 1, 0};
        for (auto [p, col] : {std::pair<Vec3, Vec3>{a, ca}, {b, ca}, {c, cb}, {d, cb}}) {
            mesh.vertices.insert(mesh.vertices.end(), {p.x, p.y, p.z, n.x, n.y, n.z, col.x, col.y, col.z});
        }
        mesh.indices.insert(mesh.indices.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
    }
};
