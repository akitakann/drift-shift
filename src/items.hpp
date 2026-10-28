#pragma once

#include "car.hpp"
#include "meshbuild.hpp"
#include "track.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

// Charge cells: block-built pickups laid along each bend. Collecting one while
// sliding pushes the drift charge a stage forward. They respawn after a while.
struct ChargeItems {
    struct Item { Vec3 position; float spin = 0; float respawn = 0; };
    struct Pickup { Vec3 position; bool charged = false; int step = 0; int level = 0; };

    static constexpr float kRadius = 3.2f;      // Horizontal pickup reach, metres.
    static constexpr float kChargeAmount = 0.55f; // Raw charge seconds per cell (about one stage).
    static constexpr float kRespawn = 7.0f;
    static constexpr float kSpacing = 16.0f;    // Along-track gap between cells in a bend.
    static constexpr float kHover = 1.15f;

    std::vector<Item> items;
    float time = 0;

    void build(const Track& track) {
        items.clear();
        time = chainTimer_ = 0;
        chain_ = 0;
        if (track.practiceArea || track.samples.size() < 8) return;
        const int n = int(track.samples.size());
        auto heading = [&](int i) {
            const auto& a = track.samples[(i % n + n) % n];
            const auto& b = track.samples[((i + 1) % n + n) % n];
            return std::atan2(b.x - a.x, b.z - a.z);
        };
        auto indexAt = [&](float s) {
            int best = 0; float gap = 1e9f;
            for (int i = 0; i < n; ++i) {
                float g = std::fabs(track.samples[i].s - s);
                if (track.closed) g = std::fmin(g, track.length - g);
                if (g < gap) { gap = g; best = i; }
            }
            return best;
        };
        // Walk the lap; inside a bend (heading change over +/-25 m beyond a
        // threshold) drop a cell every kSpacing metres, nudged to the inside.
        const float step = 4.0f;
        float lastPlaced = -1e9f;
        for (float s = 0; s < track.length; s += step) {
            const int i = indexAt(s);
            const int back = indexAt(s - 25), ahead = indexAt(s + 25);
            const float turn = wrapPi(heading(ahead) - heading(back));
            if (std::fabs(turn) < 0.30f) continue;
            if (s - lastPlaced < kSpacing) continue;
            lastPlaced = s;
            const auto& a = track.samples[i];
            const auto& b = track.samples[(i + 1) % n];
            const float dx = b.x - a.x, dz = b.z - a.z, len = std::sqrt(dx * dx + dz * dz);
            if (len < 1e-4f) continue;
            const float rightX = dz / len, rightZ = -dx / len;
            const float lateral = (turn > 0 ? 1.0f : -1.0f) * track.halfWidth * 0.30f;
            Vec3 p{a.x + rightX * lateral, a.y + lateral * a.cross + kHover, a.z + rightZ * lateral};
            items.push_back({p, s * 0.37f, 0});
        }
    }

    void reset() {
        for (auto& item : items) item.respawn = 0;
        time = chainTimer_ = 0;
        chain_ = 0;
    }

    // Collect cells the car passes through; returns this step's pickups.
    std::vector<Pickup> update(float dt, Car& car) {
        std::vector<Pickup> out;
        time += dt;
        chainTimer_ = std::fmax(0.0f, chainTimer_ - dt);
        if (chainTimer_ <= 0) chain_ = 0;
        for (auto& item : items) {
            item.spin += dt * 2.4f;
            if (item.respawn > 0) { item.respawn = std::fmax(0.0f, item.respawn - dt); continue; }
            const float dx = car.x - item.position.x, dz = car.z - item.position.z;
            const float dy = car.y + 0.6f - item.position.y;
            if (dx * dx + dz * dz > kRadius * kRadius || std::fabs(dy) > 3.0f) continue;
            item.respawn = kRespawn;
            const bool charged = car.addCharge(kChargeAmount);
            out.push_back({item.position, charged, chain_, car.driftChargeLevel});
            chain_ = std::min(chain_ + 1, 12);
            chainTimer_ = 1.6f;
        }
        return out;
    }

    // Solid, lit blocks in the car's flat-shaded style: a chunky frame cube
    // turning around a bright core. glow receives the cores for an additive pass.
    void buildMeshes(CpuMesh& solid, CpuMesh& glow, Vec3 camera) const {
        const Vec3 frame{0.95f, 0.78f, 0.10f};
        const Vec3 core{1.0f, 0.92f, 0.45f};
        for (const auto& item : items) {
            if (item.respawn > 0) {
                // Grow back in during the last half second.
                if (item.respawn > 0.5f) continue;
            }
            const Vec3 d = item.position - camera;
            if (d.x * d.x + d.z * d.z > 420.0f * 420.0f) continue;
            const float grow = item.respawn > 0 ? 1 - item.respawn / 0.5f : 1;
            const float bob = std::sin(time * 2.6f + item.spin) * 0.18f;
            const Vec3 c = item.position + Vec3{0, bob, 0};
            const float yaw = item.spin, tilt = 0.6154797f; // Corner-up cube.
            const float half = 0.62f * grow, bar = 0.11f * grow;
            auto rot = [&](Vec3 v) {
                // Tilt about X, then spin about Y.
                const float ct = std::cos(tilt), st = std::sin(tilt);
                Vec3 t{v.x, v.y * ct - v.z * st, v.y * st + v.z * ct};
                const float cy = std::cos(yaw), sy = std::sin(yaw);
                return Vec3{t.x * cy + t.z * sy, t.y, -t.x * sy + t.z * cy};
            };
            auto box = [&](CpuMesh& mesh, Vec3 mn, Vec3 mx, Vec3 color) {
                Vec3 p[8] = {{mn.x, mn.y, mn.z}, {mx.x, mn.y, mn.z}, {mx.x, mx.y, mn.z}, {mn.x, mx.y, mn.z},
                             {mn.x, mn.y, mx.z}, {mx.x, mn.y, mx.z}, {mx.x, mx.y, mx.z}, {mn.x, mx.y, mx.z}};
                for (auto& q : p) q = c + rot(q);
                const int faces[6][4] = {{3, 2, 6, 7}, {0, 1, 5, 4}, {1, 2, 6, 5}, {0, 4, 7, 3}, {4, 5, 6, 7}, {0, 3, 2, 1}};
                const Vec3 normals[6] = {{0, 1, 0}, {0, -1, 0}, {1, 0, 0}, {-1, 0, 0}, {0, 0, 1}, {0, 0, -1}};
                for (int f = 0; f < 6; ++f)
                    mesh.addQuad(p[faces[f][0]], p[faces[f][1]], p[faces[f][2]], p[faces[f][3]], rot(normals[f]), color);
            };
            // Twelve edge bars make the frame.
            for (int axis = 0; axis < 3; ++axis) for (int u = -1; u <= 1; u += 2) for (int v = -1; v <= 1; v += 2) {
                Vec3 mn, mx;
                float* lo = &mn.x; float* hi = &mx.x;
                const int a1 = (axis + 1) % 3, a2 = (axis + 2) % 3;
                lo[axis] = -half; hi[axis] = half;
                lo[a1] = u * half - bar; hi[a1] = u * half + bar;
                lo[a2] = v * half - bar; hi[a2] = v * half + bar;
                box(solid, mn, mx, frame);
            }
            const float k = 0.30f * grow;
            box(glow, {-k, -k, -k}, {k, k, k}, core * (0.75f + 0.25f * std::sin(time * 9 + item.spin)));
        }
    }

private:
    int chain_ = 0;
    float chainTimer_ = 0;
};
