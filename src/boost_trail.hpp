#pragma once

#include "car.hpp"
#include "meshbuild.hpp"

#include <cmath>
#include <deque>

// Two glowing ribbons that stream from the boost jets while a boost runs.
// Presentation only: nothing here touches the car's physics.
struct BoostTrail {
    struct Point { Vec3 position, color; float age = 0; bool breakAfter = false; };
    static constexpr float kLife = 0.5f;      // Seconds a ribbon point stays visible.
    static constexpr float kHalfWidth = 0.055f; // Half the ribbon width, metres.

    std::deque<Point> sides[2];

    // Same tier colours as the charge rings on the tyres: blue, orange, pink.
    static Vec3 levelColor(int level) {
        static const Vec3 colors[3] = {{0.08f, 0.48f, 1.0f}, {1.0f, 0.34f, 0.035f}, {1.0f, 0.12f, 0.66f}};
        return colors[level < 1 ? 0 : level > 3 ? 2 : level - 1];
    }

    void reset() { sides[0].clear(); sides[1].clear(); }

    // body: the rendered sprung-body matrix, so the ribbons leave the jets.
    void tick(float dt, const Car& car, const Mat4& body) {
        for (auto& side : sides) {
            for (auto& point : side) point.age += dt;
            while (!side.empty() && side.front().age >= kLife) side.pop_front();
        }
        const float amount = car.boostEnvelopeAmount();
        const bool boosting = car.boostRemaining > 0 && car.boostLevel > 0 && car.speed > 3 && amount > 0.02f;
        for (int i = 0; i < 2; ++i) {
            auto& side = sides[i];
            if (!boosting) {
                if (!side.empty()) side.back().breakAfter = true;
                continue;
            }
            const Vec3 jet = transform(body, {i == 0 ? -0.53f : 0.53f, 0.40f, -2.16f});
            side.push_back({jet, levelColor(car.boostLevel) * (0.35f + 0.65f * amount), 0, false});
            if (side.size() > 120) side.pop_front();
        }
    }

    // Camera-facing ribbons; vertex colours carry the fade for additive drawing.
    void build(CpuMesh& mesh, Vec3 camera) const {
        for (const auto& side : sides) {
            for (size_t i = 1; i < side.size(); ++i) {
                const Point& a = side[i - 1];
                const Point& b = side[i];
                if (a.breakAfter) continue;
                const Vec3 along = b.position - a.position;
                Vec3 across = cross(along, camera - b.position);
                const float length = std::sqrt(dot(across, across));
                if (length < 1e-5f) continue;
                across = across * (kHalfWidth / length);
                const Vec3 ca = a.color * fade(a), cb = b.color * fade(b);
                const auto base = static_cast<std::uint32_t>(mesh.vertices.size() / 9);
                mesh.addVertex(a.position - across, {0, 1, 0}, ca);
                mesh.addVertex(a.position + across, {0, 1, 0}, ca);
                mesh.addVertex(b.position + across, {0, 1, 0}, cb);
                mesh.addVertex(b.position - across, {0, 1, 0}, cb);
                mesh.addRawTriangle(base, base + 1, base + 2);
                mesh.addRawTriangle(base, base + 2, base + 3);
            }
        }
    }

private:
    static float fade(const Point& point) {
        const float t = point.age / kLife;
        return t >= 1 ? 0 : (1 - t) * (1 - t);
    }
    static Vec3 transform(const Mat4& m, Vec3 p) {
        return {m.m[0] * p.x + m.m[4] * p.y + m.m[8] * p.z + m.m[12],
                m.m[1] * p.x + m.m[5] * p.y + m.m[9] * p.z + m.m[13],
                m.m[2] * p.x + m.m[6] * p.y + m.m[10] * p.z + m.m[14]};
    }
};
