#pragma once

#include "car.hpp"
#include "meshbuild.hpp"
#include "track.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

// Dash cells: rows of block-built pickups down the middle of each straight and
// corner exit. Driving through one gives a short push. Every cell comes back
// when a new lap begins, so each lap offers the same line.
struct DashItems {
    struct Item { Vec3 position; float spin = 0; bool taken = false; float regrow = 0; };
    struct Pickup { Vec3 position; int chain = 0; };

    static constexpr float kRadius = 3.2f;       // Horizontal pickup reach, metres.
    static constexpr float kHover = 1.15f;       // Height of a cell's centre above the road.
    static constexpr float kRowSpacing = 14.0f;  // Along-track gap between cells in a row.
    static constexpr int kRowLength = 4;
    static constexpr float kExitOffset = 24.0f;  // A row starts this far into a straight.
    static constexpr float kStraightRepeat = 450.0f; // A long straight gets another row this often.
    static constexpr float kBendTurn = 0.18f;    // Heading change over 50 m that counts as a bend, rad.

    std::vector<Item> items;
    float time = 0;

    void build(const Track& track) {
        items.clear();
        time = chainTimer_ = 0;
        chain_ = 0;
        if (track.practiceArea || !track.closed || track.samples.size() < 8 || !(track.length > 0)) return;
        const int n = int(track.samples.size());
        // Samples are evenly spread by s, so the index of a distance is a lookup.
        auto indexAt = [&](float s) {
            s = std::fmod(s, track.length);
            if (s < 0) s += track.length;
            const auto it = std::lower_bound(track.samples.begin(), track.samples.end(), s,
                [](const TrackSample& sample, float value) { return sample.s < value; });
            return it == track.samples.end() ? n - 1 : int(it - track.samples.begin());
        };
        auto heading = [&](int i) {
            const auto& a = track.samples[(i % n + n) % n];
            const auto& b = track.samples[((i + 1) % n + n) % n];
            return std::atan2(b.x - a.x, b.z - a.z);
        };
        auto bendAt = [&](float s) {
            return std::fabs(wrapPi(heading(indexAt(s + 25)) - heading(indexAt(s - 25)))) > kBendTurn;
        };
        auto place = [&](float s) {
            const int i = indexAt(s);
            const auto& a = track.samples[i];
            items.push_back({{a.x, a.y + kHover, a.z}, s * 0.37f, false, 0});
        };
        const float step = 4.0f;
        float straight = 0, lastRow = -1e9f;
        bool afterBend = true;
        for (float s = 0; s < track.length; s += step) {
            if (bendAt(s)) { straight = 0; afterBend = true; continue; }
            straight += step;
            const float rowEnd = s + kRowSpacing * (kRowLength - 1);
            const bool roomAhead = rowEnd < track.length && !bendAt(rowEnd);
            const bool exitRow = afterBend && straight >= kExitOffset;
            const bool repeatRow = !afterBend && s - lastRow >= kStraightRepeat;
            if ((exitRow || repeatRow) && roomAhead) {
                for (int k = 0; k < kRowLength; ++k) place(s + kRowSpacing * k);
                lastRow = s;
                afterBend = false;
            } else if (exitRow) {
                afterBend = false; // Too little straight for a whole row here.
            }
        }
    }

    void respawn() {
        for (auto& item : items) if (item.taken) { item.taken = false; item.regrow = 0.5f; }
        chain_ = 0;
        chainTimer_ = 0;
    }

    // Collects the cells the car drives through and pushes the car once for each.
    std::vector<Pickup> update(float dt, Car& car) {
        std::vector<Pickup> out;
        time += dt;
        chainTimer_ = std::fmax(0.0f, chainTimer_ - dt);
        if (chainTimer_ <= 0) chain_ = 0;
        for (auto& item : items) {
            item.spin += dt * 2.4f;
            item.regrow = std::fmax(0.0f, item.regrow - dt);
            if (item.taken) continue;
            const float dx = car.x - item.position.x, dz = car.z - item.position.z;
            const float dy = car.y + 0.6f - item.position.y;
            if (dx * dx + dz * dz > kRadius * kRadius || std::fabs(dy) > 3.0f) continue;
            item.taken = true;
            car.addKick();
            out.push_back({item.position, chain_});
            chain_ = std::min(chain_ + 1, 12);
            chainTimer_ = 1.2f;
        }
        return out;
    }

    // Solid, lit blocks in the car's flat-shaded style: a chunky frame cube
    // turning around a bright core. glow receives the cores for an additive pass.
    void buildMeshes(CpuMesh& solid, CpuMesh& glow, Vec3 camera) const {
        const Vec3 frame{0.95f, 0.78f, 0.10f};
        const Vec3 core{1.0f, 0.92f, 0.45f};
        for (const auto& item : items) {
            if (item.taken) continue;
            const Vec3 d = item.position - camera;
            if (d.x * d.x + d.z * d.z > 420.0f * 420.0f) continue;
            const float grow = item.regrow > 0 ? 1 - item.regrow / 0.5f : 1;
            const float bob = std::sin(time * 2.6f + item.spin) * 0.18f;
            const Vec3 c = item.position + Vec3{0, bob, 0};
            const float yaw = item.spin, tilt = 0.6154797f; // Corner-up cube.
            const float half = 0.62f * grow, bar = 0.11f * grow;
            auto rot = [&](Vec3 v) {
                const float ct = std::cos(tilt), st = std::sin(tilt);
                const Vec3 t{v.x, v.y * ct - v.z * st, v.y * st + v.z * ct};
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
            auto set = [](Vec3& v, int axis, float value) { (axis == 0 ? v.x : axis == 1 ? v.y : v.z) = value; };
            for (int axis = 0; axis < 3; ++axis) for (int u = -1; u <= 1; u += 2) for (int v = -1; v <= 1; v += 2) {
                Vec3 mn, mx;
                const int a1 = (axis + 1) % 3, a2 = (axis + 2) % 3;
                set(mn, axis, -half); set(mx, axis, half);
                set(mn, a1, u * half - bar); set(mx, a1, u * half + bar);
                set(mn, a2, v * half - bar); set(mx, a2, v * half + bar);
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
