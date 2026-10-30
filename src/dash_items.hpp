#pragma once

#include "car.hpp"
#include "meshbuild.hpp"
#include "track.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

// Dash cells laid out to be driven. Through each bend they trace a drifting
// line, wide on entry, tight at the apex and wide again on exit. On the
// straights short rows sit to one side, then the other, far enough apart to
// steer across at speed. Driving through a cell gives a short push, and adds
// to the charge while a drift is held. Every cell returns when a lap begins.
struct DashItems {
    enum class Kind { Line, Weave };
    struct Item {
        Vec3 position;
        float spin = 0;
        bool taken = false;
        float regrow = 0;
        float s = 0;       // Distance along the lap.
        float lateral = 0; // Offset from the centre line, right positive.
        Kind kind = Kind::Line;
    };
    struct Pickup { Vec3 position; int chain = 0; };

    static constexpr float kRadius = 3.2f;        // Horizontal pickup reach, metres.
    static constexpr float kHover = 1.15f;        // Height of a cell's centre above the road.
    static constexpr float kEdgeMargin = 3.4f;    // A cell's centre stays this far inside the road edge.
    static constexpr float kBendTurn = 0.18f;     // Heading change over 50 m that counts as a bend, rad.
    static constexpr float kBendMerge = 40.0f;    // Same-handed turning this close is one bend.
    static constexpr float kBendMinTurn = 0.35f;  // A bend turns the road at least this much, rad.
    static constexpr float kLineLead = 12.0f;     // The line is wide at least this far before a bend's turn-in,
    static constexpr float kLineTail = 18.0f;     // and this far after its exit,
    static constexpr float kApproach = 5.0f;      // and never closer to the apex than this many half widths.
    static constexpr float kSameJoin = 8.0f;      // Same-handed bends this many half widths apart share a line.
    static constexpr float kOutside = 0.85f;      // Share of the swing the line takes to the outside.
    static constexpr float kFullTurn = 1.2f;      // A bend this sharp swings across the whole road, rad;
    static constexpr float kMinSwing = 0.4f;      // a gentle one uses at least this share of it.
    static constexpr float kLineSpacing = 30.0f;  // Preferred gap between cells along a bend's line.
    static constexpr int kLineMin = 3, kLineMax = 5;
    static constexpr float kWeaveSpacing = 14.0f; // Gap between cells within a straight row.
    static constexpr int kWeaveRow = 3;
    static constexpr float kWeavePitch = 300.0f;  // Start-to-start distance between rows on opposite sides.
    static constexpr float kWeaveSide = 0.7f;     // Share of the usable half width a row sits off centre.
    static constexpr float kWeaveClear = 70.0f;   // Rows keep this far from a bend's cells.
    static constexpr float kChainWindow = 1.6f;   // Seconds a chain stays alive between cells.

    std::vector<Item> items;
    std::vector<float> line; // The drifting line's offset at each track sample, right positive.
    float time = 0;

    void build(const Track& track) {
        items.clear();
        line.clear();
        keys_.clear();
        time = chainTimer_ = 0;
        chain_ = 0;
        if (track.practiceArea || !track.closed || track.samples.size() < 8 || !(track.length > 0)) return;
        const float length = track.length;
        const float limit = std::fmax(0.0f, track.halfWidth - kEdgeMargin);

        // Bends: runs where the centre line turns one way. A change of hands
        // starts a new bend, so an S is two bends.
        const float step = 4.0f;
        struct Bend { float a, b, apex, sign, swing, in, out; bool linkPrev, linkNext; };
        std::vector<Bend> bends;
        for (float s = 0; s < length; s += step) {
            const float turn = wrapPi(headingAt(track, s + 25) - headingAt(track, s - 25));
            const float sign = turn > kBendTurn ? 1.0f : turn < -kBendTurn ? -1.0f : 0.0f;
            if (sign == 0) continue;
            // A brief easing inside one corner does not split it.
            if (!bends.empty() && bends.back().sign == sign && s - bends.back().b <= kBendMerge) bends.back().b = s;
            else bends.push_back({s, s, s, sign, 0, 0, 0, false, false});
        }
        if (bends.size() > 1 && bends.front().sign == bends.back().sign &&
            bends.front().a + length - bends.back().b <= kBendMerge) {
            bends.front().a = bends.back().a - length; // The lap's last bend runs over the line.
            bends.pop_back();
        }
        // A kink that turns the road only a little is no bend to drift.
        bends.erase(std::remove_if(bends.begin(), bends.end(), [&](const Bend& bend) {
            return std::fabs(wrapPi(headingAt(track, bend.b + 25) - headingAt(track, bend.a - 25))) < kBendMinTurn;
        }), bends.end());
        if (bends.empty()) return;
        // The apex is where half of the bend's turning is done.
        auto localTurn = [&](float s) { return wrapPi(headingAt(track, s + step / 2) - headingAt(track, s - step / 2)); };
        for (auto& bend : bends) {
            float total = 0, done = 0;
            for (float s = bend.a; s <= bend.b; s += step) total += localTurn(s);
            bend.apex = 0.5f * (bend.a + bend.b);
            for (float s = bend.a; s <= bend.b; s += step) {
                done += localTurn(s);
                if (std::fabs(done) >= 0.5f * std::fabs(total)) { bend.apex = s; break; }
            }
        }
        // How far each bend's line swings, and where it is widest before and
        // after the apex. Sharper bends swing across more of the road.
        const float approach = kApproach * track.halfWidth;
        for (auto& bend : bends) {
            const float turn = std::fabs(wrapPi(headingAt(track, bend.b + 25) - headingAt(track, bend.a - 25)));
            bend.swing = limit * std::clamp(turn / kFullTurn, kMinSwing, 1.0f);
            bend.in = std::fmax(bend.apex - (bend.a - kLineLead), approach);
            bend.out = std::fmax(bend.b + kLineTail - bend.apex, 1.3f * approach);
        }
        // Bends too close to swing wide between share one line, apex to apex.
        // Turns the same way join more readily: the drift simply carries on.
        const std::size_t count = bends.size();
        for (std::size_t i = 0; i < count && count > 1; ++i) {
            Bend& next = bends[(i + 1) % count];
            const float wrap = i + 1 == count ? length : 0.0f;
            // Too little room to swing out and back in again between them.
            const bool overlap = next.apex - next.in + wrap <= bends[i].apex + bends[i].out + approach;
            const bool carryOn = next.sign == bends[i].sign && next.a + wrap - bends[i].b <= kSameJoin * track.halfWidth;
            if (overlap || carryOn) { bends[i].linkNext = true; next.linkPrev = true; }
        }

        // The line: wide before each turn-in, tight at the apex, wide after.
        for (const auto& bend : bends) {
            if (!bend.linkPrev) keys_.push_back({bend.apex - bend.in, -bend.sign * kOutside * bend.swing});
            keys_.push_back({bend.apex, bend.sign * bend.swing});
            if (!bend.linkNext) keys_.push_back({bend.apex + bend.out, -bend.sign * kOutside * bend.swing});
        }
        for (auto& key : keys_) key.s = wrapS(key.s, length);
        std::sort(keys_.begin(), keys_.end(), [](const Key& a, const Key& b) { return a.s < b.s; });
        line.resize(track.samples.size());
        for (std::size_t i = 0; i < track.samples.size(); ++i) line[i] = lineAt(track, track.samples[i].s);

        // Cells along each bend's line, from its turn-in (or the middle of the
        // link from the bend before) to its exit (or the middle of the next link).
        for (std::size_t i = 0; i < count; ++i) {
            const Bend& bend = bends[i];
            const Bend& prev = bends[(i + count - 1) % count];
            const Bend& next = bends[(i + 1) % count];
            const float prevApex = prev.apex - (i == 0 ? length : 0.0f);
            const float nextApex = next.apex + (i + 1 == count ? length : 0.0f);
            const float from = bend.linkPrev ? 0.5f * (prevApex + bend.apex) : bend.apex - bend.in;
            const float to = bend.linkNext ? 0.5f * (bend.apex + nextApex) : bend.apex + bend.out;
            const float span = to - from;
            const int cells = std::clamp(int(std::lround(span / kLineSpacing)) + 1, kLineMin, kLineMax);
            for (int k = 0; k < cells; ++k) {
                if (bend.linkNext && k == cells - 1) break; // The next bend starts there.
                const float s = from + span * float(k) / float(cells - 1);
                place(track, s, lineAt(track, s), Kind::Line);
            }
        }

        // Long straights: rows that switch sides, starting on the side the bend
        // before leaves the car, so the first row continues the trail.
        const float side = kWeaveSide * limit;
        const float rowLength = kWeaveSpacing * float(kWeaveRow - 1);
        for (std::size_t i = 0; i < count; ++i) {
            if (bends[i].linkNext) continue;
            const Bend& next = bends[(i + 1) % count];
            const float start = bends[i].apex + bends[i].out + kWeaveClear;
            const float end = next.apex - next.in + (i + 1 == count ? length : 0.0f) - kWeaveClear;
            if (end - start < kWeavePitch) continue; // Short straights stay busy enough between bends.
            float sign = -bends[i].sign;
            for (float r = start; r + rowLength <= end; r += kWeavePitch) {
                for (int k = 0; k < kWeaveRow; ++k) place(track, r + kWeaveSpacing * float(k), sign * side, Kind::Weave);
                sign = -sign;
            }
        }
        std::sort(items.begin(), items.end(), [](const Item& a, const Item& b) { return a.s < b.s; });
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
            chainTimer_ = kChainWindow;
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

    // The drifting line's offset at a distance along the lap, right positive.
    float lineAt(const Track& track, float s) const {
        if (keys_.empty() || !(track.length > 0)) return 0;
        s = wrapS(s, track.length);
        const auto after = std::upper_bound(keys_.begin(), keys_.end(), s, [](float v, const Key& k) { return v < k.s; });
        const Key& b = after == keys_.end() ? keys_.front() : *after;
        const Key& a = after == keys_.begin() ? keys_.back() : *(after - 1);
        float span = b.s - a.s, into = s - a.s;
        if (span <= 0) span += track.length;
        if (into < 0) into += track.length;
        const float t = span > 0 ? std::clamp(into / span, 0.0f, 1.0f) : 0.0f;
        return a.lateral + (b.lateral - a.lateral) * t * t * (3 - 2 * t);
    }

private:
    struct Key { float s; float lateral; }; // A point the drifting line passes through.
    std::vector<Key> keys_;                  // In lap order.
    int chain_ = 0;
    float chainTimer_ = 0;

    static float wrapS(float s, float length) {
        s = std::fmod(s, length);
        return s < 0 ? s + length : s;
    }

    // A point on the centre line: the samples either side and the blend between.
    struct Frame { int i = 0, j = 0; float t = 0; float x = 0, z = 0, y = 0, cross = 0, rightX = 1, rightZ = 0; };

    static Frame frameAt(const Track& track, float s) {
        const auto& samples = track.samples;
        const int n = int(samples.size());
        s = std::fmod(s, track.length);
        if (s < 0) s += track.length;
        const auto it = std::upper_bound(samples.begin(), samples.end(), s,
            [](float value, const TrackSample& sample) { return value < sample.s; });
        Frame f;
        f.i = it == samples.begin() ? n - 1 : int(it - samples.begin()) - 1;
        f.j = (f.i + 1) % n;
        const TrackSample& a = samples[std::size_t(f.i)];
        const TrackSample& b = samples[std::size_t(f.j)];
        float gap = b.s - a.s;
        if (gap <= 0) gap += track.length;
        float into = s - a.s;
        if (into < 0) into += track.length;
        f.t = gap > 0 ? std::clamp(into / gap, 0.0f, 1.0f) : 0.0f;
        f.x = a.x + (b.x - a.x) * f.t;
        f.z = a.z + (b.z - a.z) * f.t;
        f.y = a.y + (b.y - a.y) * f.t;
        f.cross = a.cross + (b.cross - a.cross) * f.t;
        const float tx = b.x - a.x, tz = b.z - a.z, len = std::sqrt(tx * tx + tz * tz);
        if (len > 1e-4f) { f.rightX = tz / len; f.rightZ = -tx / len; }
        return f;
    }

    static float headingAt(const Track& track, float s) {
        const Frame f = frameAt(track, s);
        return std::atan2(-f.rightZ, f.rightX); // The travel direction, as atan2(dx, dz).
    }

    void place(const Track& track, float s, float lateral, Kind kind) {
        s = std::fmod(s, track.length);
        if (s < 0) s += track.length;
        const Frame f = frameAt(track, s);
        Item item;
        item.position = {f.x + f.rightX * lateral, f.y + f.cross * lateral + kHover, f.z + f.rightZ * lateral};
        item.spin = s * 0.37f;
        item.s = s;
        item.lateral = lateral;
        item.kind = kind;
        items.push_back(item);
    }
};
