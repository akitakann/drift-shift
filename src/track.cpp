#include "track.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

namespace {

Vec3 horizontalRight(const Track& track, int index) {
    const int n = static_cast<int>(track.samples.size());
    const TrackSample& prev = track.samples[track.closed ? (index + n - 1) % n : std::max(0, index - 1)];
    const TrackSample& next = track.samples[track.closed ? (index + 1) % n : std::min(n - 1, index + 1)];
    float tx = next.x - prev.x;
    float tz = next.z - prev.z;
    float len = std::sqrt(tx * tx + tz * tz);
    if (len < 1e-4f) {
        return {1, 0, 0};
    }
    tx /= len;
    tz /= len;
    return {tz, 0, -tx};
}

Vec3 roadNormal(const Track& track, int index) {
    const int n = static_cast<int>(track.samples.size());
    const TrackSample& prev = track.samples[track.closed ? (index + n - 1) % n : std::max(0, index - 1)];
    const TrackSample& next = track.samples[track.closed ? (index + 1) % n : std::min(n - 1, index + 1)];
    Vec3 tangent = normalize(Vec3{next.x - prev.x, next.y - prev.y, next.z - prev.z});
    Vec3 right = horizontalRight(track, index);
    right.y = track.samples[index].cross;
    right = normalize(right);
    Vec3 normal = normalize(cross(tangent, right));
    if (normal.y < 0) {
        normal = normal * -1.0f;
    }
    return normal;
}

Vec3 onRoad(const Track& track, int index, float lateral) {
    const TrackSample& sample = track.samples[index];
    Vec3 right = horizontalRight(track, index);
    return {
        sample.x + right.x * lateral,
        sample.y + lateral * sample.cross,
        sample.z + right.z * lateral,
    };
}

struct ReferencePose {
    Vec3 center, right, forward;
    float cross = 0;
};

ReferencePose referencePose(const Track& track, float distance) {
    const int count = static_cast<int>(track.samples.size());
    if (track.closed) {
        distance = std::fmod(distance, track.length);
        if (distance < 0) distance += track.length;
    } else distance = clampf(distance, 0, track.length);
    const auto after = std::upper_bound(track.samples.begin(), track.samples.end(), distance,
        [](float value, const TrackSample& sample) { return value < sample.s; });
    int index = static_cast<int>(after - track.samples.begin()) - 1;
    index = std::max(0, std::min(track.closed ? count - 1 : count - 2, index));
    const int next = (index + 1) % count;
    const auto& a = track.samples[index];
    const auto& b = track.samples[next];
    const float segment = next == 0 ? track.length - a.s : b.s - a.s;
    const float t = segment > .0001f ? clampf((distance - a.s) / segment, 0, 1) : 0;
    const Vec3 forward = normalize(Vec3{b.x - a.x, 0, b.z - a.z});
    return {{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t},
            {forward.z, 0, -forward.x}, forward, a.cross + (b.cross - a.cross) * t};
}

Vec3 referencePoint(const Track& track, float distance, float lateral, float lift) {
    const auto pose = referencePose(track, distance);
    return pose.center + pose.right * lateral + Vec3{0, lateral * pose.cross + lift, 0};
}

void addReflectorPost(CpuMesh& mesh, const ReferencePose& pose, float lateral, float edgeLateral) {
    // White upright bodies and a small warm reflector give a near-field rhythm.
    // They belong only to the static visual mesh, never to the driving query.
    const Vec3 base = pose.center + pose.right * lateral + Vec3{0, edgeLateral * pose.cross, 0};
    const Vec3 right = pose.right * .055f, forward = pose.forward * .055f;
    const Vec3 up{0, .70f, 0}, white{.86f, .88f, .84f}, amber{1, .58f, .10f};
    const Vec3 a = base - right - forward, b = base + right - forward;
    const Vec3 c = base + right + forward, d = base - right + forward;
    mesh.addQuad(a, b, b + up, a + up, pose.forward * -1, white);
    mesh.addQuad(b, c, c + up, b + up, pose.right, white * .92f);
    mesh.addQuad(c, d, d + up, c + up, pose.forward, white);
    mesh.addQuad(d, a, a + up, d + up, pose.right * -1, white * .92f);
    mesh.addQuad(a + up, b + up, c + up, d + up, {0, 1, 0}, white);
    for (float side : {-1.0f, 1.0f}) {
        const Vec3 face = base + pose.forward * (side * .056f);
        const Vec3 narrow = pose.right * .043f;
        mesh.addQuad(face - narrow + Vec3{0, .48f, 0}, face + narrow + Vec3{0, .48f, 0},
                     face + narrow + Vec3{0, .61f, 0}, face - narrow + Vec3{0, .61f, 0},
                     pose.forward * side, amber);
    }
}

void addRoadSpeedReferences(CpuMesh& mesh, const Track& track) {
    if (track.samples.size() < 2 || !(track.length > 0) || track.halfWidth < 1) return;
    const Vec3 white{.80f, .82f, .78f};
    // At the default stage scale these six-metre intervals become8.4m. Stagger
    // the sides so a pair of posts never flashes across the whole view at once.
    for (int side : {-1, 1}) {
        const float sign = static_cast<float>(side);
        const float edge = sign * (track.halfWidth + track.curbWidth);
        for (float s = side < 0 ? 0 : 3; s < track.length; s += 6) {
            const float lateral = sign * (track.halfWidth - .30f);
            const float start = track.closed ? s - .8f : std::fmax(0, s - .8f);
            const float end = track.closed ? s + .8f : std::fmin(track.length, s + .8f);
            const Vec3 a = referencePoint(track, start, lateral - .09f, .025f);
            const Vec3 b = referencePoint(track, start, lateral + .09f, .025f);
            const Vec3 c = referencePoint(track, end, lateral + .09f, .025f);
            const Vec3 d = referencePoint(track, end, lateral - .09f, .025f);
            Vec3 normal = normalize(cross(d - a, b - a));
            if (normal.y < 0) normal = normal * -1;
            if (end != start) mesh.addQuad(a, b, c, d, normal, white);
            if (track.runoff > .5f)
                addReflectorPost(mesh, referencePose(track, s), edge + sign * .45f, edge);
        }
    }
}

void addPracticeSpeedReferences(CpuMesh& mesh) {
    const Vec3 white{.58f, .61f, .57f};
    // Small local paint patches, not a full-map grid or a prescribed course.
    // The central launch strip stays open; no object can obstruct free driving.
    constexpr float rows[] = {-156, -108, -60, -28, -12, 12, 28, 60, 108, 156};
    for (float row : rows) for (int step = -20; step <= 20; ++step) {
        const float along = step * 8.0f;
        mesh.addQuad({row - .10f, .025f, along - .70f}, {row + .10f, .025f, along - .70f},
                     {row + .10f, .025f, along + .70f}, {row - .10f, .025f, along + .70f}, {0, 1, 0}, white);
        if (std::fabs(along) > 8)
            mesh.addQuad({along - .70f, .025f, row - .10f}, {along + .70f, .025f, row - .10f},
                         {along + .70f, .025f, row + .10f}, {along - .70f, .025f, row + .10f}, {0, 1, 0}, white);
    }
}

void addRibbon(CpuMesh& mesh, const Track& track, float lat0, float lat1, Vec3 color, bool flatOuter) {
    const int n = static_cast<int>(track.samples.size());
    for (int i = 0; i < n - (track.closed ? 0 : 1); ++i) {
        int j = (i + 1) % n;
        Vec3 a = onRoad(track, i, lat0);
        Vec3 b = onRoad(track, i, lat1);
        Vec3 c = onRoad(track, j, lat1);
        Vec3 d = onRoad(track, j, lat0);
        if (flatOuter) {
            // 芝は縁石の外側の高さで水平にする。
            float edge = (std::fabs(lat0) > std::fabs(lat1) ? lat1 : lat0);
            float sign = edge >= 0 ? 1.0f : -1.0f;
            float edgeLat = sign * (track.halfWidth + track.curbWidth);
            float y0 = track.samples[i].y + edgeLat * track.samples[i].cross;
            float y1 = track.samples[j].y + edgeLat * track.samples[j].cross;
            a.y = y0;
            b.y = y0;
            c.y = y1;
            d.y = y1;
        }
        Vec3 normal = normalize(roadNormal(track, i) + roadNormal(track, j));
        Vec3 tint = color;
        if (!flatOuter) {
            float shade = 0.94f + 0.08f * float((i * 13) % 5) / 5.0f;
            tint = color * shade;
        }
        mesh.addQuad(a, b, c, d, normal, tint);
    }
}

void addWalls(CpuMesh& mesh, const Track& track) {
    const int n = static_cast<int>(track.samples.size());
    const float inner = track.wallLateral();
    const float outer = inner + 0.4f;
    const float height = 0.9f;
    const Vec3 concrete{0.62f, 0.63f, 0.66f};
    for (int side = -1; side <= 1; side += 2) {
        float sign = float(side);
        for (int i = 0; i < n - (track.closed ? 0 : 1); ++i) {
            int j = (i + 1) % n;
            Vec3 a = onRoad(track, i, sign * inner);
            Vec3 b = onRoad(track, j, sign * inner);
            Vec3 c = onRoad(track, j, sign * outer);
            Vec3 d = onRoad(track, i, sign * outer);
            // 芝の高さに合わせる
            float edge = sign * (track.halfWidth + track.curbWidth);
            a.y = track.samples[i].y + edge * track.samples[i].cross;
            b.y = track.samples[j].y + edge * track.samples[j].cross;
            c.y = b.y;
            d.y = a.y;
            Vec3 topA = a + Vec3{0, height, 0};
            Vec3 topB = b + Vec3{0, height, 0};
            Vec3 topC = c + Vec3{0, height, 0};
            Vec3 topD = d + Vec3{0, height, 0};
            Vec3 inward{-sign * horizontalRight(track, i).x, 0, -sign * horizontalRight(track, i).z};
            mesh.addQuad(a, b, topB, topA, inward, concrete);
            mesh.addQuad(d, topD, topC, c, inward * -1.0f, concrete * 0.85f);
            mesh.addQuad(topA, topB, topC, topD, {0, 1, 0}, concrete * 1.05f);
        }
    }
}

void addCurbs(CpuMesh& mesh, const Track& track) {
    const int n = static_cast<int>(track.samples.size());
    const Vec3 red{0.75f, 0.09f, 0.08f};
    const Vec3 white{0.90f, 0.90f, 0.92f};
    for (int side = -1; side <= 1; side += 2) {
        float sign = float(side);
        float inner = sign * track.halfWidth;
        float outer = sign * (track.halfWidth + track.curbWidth);
        for (int i = 0; i < n - (track.closed ? 0 : 1); ++i) {
            int j = (i + 1) % n;
            bool paint = (static_cast<int>(track.samples[i].s / 3.0f) % 2) == 0;
            Vec3 color = paint ? red : white;
            Vec3 a = onRoad(track, i, inner);
            Vec3 b = onRoad(track, i, outer);
            Vec3 c = onRoad(track, j, outer);
            Vec3 d = onRoad(track, j, inner);
            Vec3 normal = normalize(roadNormal(track, i) + roadNormal(track, j));
            mesh.addQuad(a, b, c, d, normal, color);
        }
    }
}

void addStartLine(CpuMesh& mesh, const Track& track) {
    if (track.samples.size() < 2) {
        return;
    }
    Vec3 right = horizontalRight(track, 0);
    float yaw = track.spawnYaw;
    // スポーンはラインの手前なので、ライン自体はサンプル 0 の接線を使う。
    const TrackSample& a = track.samples[0];
    const TrackSample& b = track.samples[1];
    float tx = b.x - a.x;
    float tz = b.z - a.z;
    float len = std::sqrt(tx * tx + tz * tz);
    if (len > 1e-4f) {
        tx /= len;
        tz /= len;
        yaw = std::atan2(tx, tz);
    }
    Vec3 forward{std::sin(yaw), 0, std::cos(yaw)};
    Vec3 center{a.x, a.y, a.z};
    const int cells = 8;
    const float depth = 0.8f;
    for (int i = 0; i < cells; ++i) {
        float lat0 = -track.halfWidth + track.halfWidth * 2.0f * (float(i) / cells);
        float lat1 = -track.halfWidth + track.halfWidth * 2.0f * (float(i + 1) / cells);
        Vec3 color = (i % 2 == 0) ? Vec3{0.92f, 0.92f, 0.94f} : Vec3{0.05f, 0.05f, 0.06f};
        Vec3 p0 = center + right * lat0;
        Vec3 p1 = center + right * lat1;
        Vec3 p2 = p1 + forward * depth;
        Vec3 p3 = p0 + forward * depth;
        p0.y = a.y + lat0 * a.cross + 0.04f;
        p1.y = a.y + lat1 * a.cross + 0.04f;
        p2.y = p1.y;
        p3.y = p0.y;
        mesh.addQuad(p0, p1, p2, p3, {0, 1, 0}, color);
    }

    // スタートの門。位置が分かるようにする。
    const float pillar = track.halfWidth + 1.1f;
    const Vec3 steel{0.22f, 0.24f, 0.28f};
    for (int side = -1; side <= 1; side += 2) {
        Vec3 base = center + right * (float(side) * pillar);
        base.y = a.y;
        Vec3 axisX = right * 0.28f;
        Vec3 axisZ = forward * 0.28f;
        Vec3 y0 = base;
        Vec3 y1 = base + Vec3{0, 6.4f, 0};
        mesh.addQuad(y0 - axisX - axisZ, y0 + axisX - axisZ, y1 + axisX - axisZ, y1 - axisX - axisZ, forward * -1.0f, steel);
        mesh.addQuad(y0 - axisX + axisZ, y1 - axisX + axisZ, y1 + axisX + axisZ, y0 + axisX + axisZ, forward, steel);
        mesh.addQuad(y0 - axisX - axisZ, y1 - axisX - axisZ, y1 - axisX + axisZ, y0 - axisX + axisZ, right * -1.0f, steel);
        mesh.addQuad(y0 + axisX - axisZ, y0 + axisX + axisZ, y1 + axisX + axisZ, y1 + axisX - axisZ, right, steel);
    }
    Vec3 left = center + right * -pillar + Vec3{0, 6.2f, 0};
    Vec3 rightTop = center + right * pillar + Vec3{0, 6.2f, 0};
    mesh.addQuad(left, rightTop, rightTop + Vec3{0, 0.4f, 0}, left + Vec3{0, 0.4f, 0}, forward, steel);
}

void addPit(CpuMesh& mesh, const std::vector<Vec3>& pit) {
    if (pit.size() < 2) {
        return;
    }
    const float half = 4.0f;
    const Vec3 color{0.24f, 0.24f, 0.26f};
    for (size_t i = 0; i + 1 < pit.size(); ++i) {
        Vec3 a = pit[i];
        Vec3 b = pit[i + 1];
        float tx = b.x - a.x;
        float tz = b.z - a.z;
        float len = std::sqrt(tx * tx + tz * tz);
        if (len < 0.05f) {
            continue;
        }
        tx /= len;
        tz /= len;
        Vec3 right{tz, 0, -tx};
        Vec3 a0 = a + right * -half;
        Vec3 a1 = a + right * half;
        Vec3 b1 = b + right * half;
        Vec3 b0 = b + right * -half;
        a0.y = a.y + 0.03f;
        a1.y = a.y + 0.03f;
        b0.y = b.y + 0.03f;
        b1.y = b.y + 0.03f;
        mesh.addQuad(a0, a1, b1, b0, {0, 1, 0}, color);
    }
}

void addGround(CpuMesh& mesh, const Track& track) {
    float minX = track.samples[0].x;
    float maxX = minX;
    float minZ = track.samples[0].z;
    float maxZ = minZ;
    float minY = track.samples[0].y;
    for (const TrackSample& sample : track.samples) {
        minX = std::min(minX, sample.x);
        maxX = std::max(maxX, sample.x);
        minZ = std::min(minZ, sample.z);
        maxZ = std::max(maxZ, sample.z);
        minY = std::min(minY, sample.y);
    }
    const float pad = 120.0f;
    float y = minY - 4.0f;
    Vec3 color{0.12f, 0.27f, 0.11f};
    Vec3 a{minX - pad, y, minZ - pad};
    Vec3 b{maxX + pad, y, minZ - pad};
    Vec3 c{maxX + pad, y, maxZ + pad};
    Vec3 d{minX - pad, y, maxZ + pad};
    mesh.addQuad(a, d, c, b, {0, 1, 0}, color);
}

void addPracticeCone(CpuMesh& mesh, Vec3 position) {
    const Vec3 orange{1.0f, 0.30f, 0.035f};
    const Vec3 white{0.95f, 0.96f, 0.94f};
    mesh.addBox(position + Vec3{-0.40f, 0.0f, -0.40f},
                position + Vec3{0.40f, 0.12f, 0.40f}, {0.09f, 0.10f, 0.11f});
    constexpr int sides = 12;
    constexpr float tau = 6.28318530718f;
    constexpr float bottom = 0.12f, top = 1.20f;
    constexpr float bottomRadius = 0.34f, topRadius = 0.035f;
    constexpr float slope = (bottomRadius - topRadius) / (top - bottom);
    const float bands[] = {bottom, 0.42f, 0.60f, 0.86f, 1.02f, top};
    auto ringPoint = [&](float y, float angle) {
        const float radius = bottomRadius - (y - bottom) * slope;
        return position + Vec3{std::cos(angle) * radius, y, std::sin(angle) * radius};
    };
    for (int i = 0; i < sides; ++i) {
        const float a0 = tau * float(i) / sides;
        const float a1 = tau * float(i + 1) / sides;
        const float middle = (a0 + a1) * 0.5f;
        const Vec3 normal = normalize(Vec3{std::cos(middle), slope * std::cos((a1 - a0) * 0.5f),
                                           std::sin(middle)});
        for (int band = 0; band < 5; ++band) {
            mesh.addQuad(ringPoint(bands[band], a0), ringPoint(bands[band], a1),
                         ringPoint(bands[band + 1], a1), ringPoint(bands[band + 1], a0),
                         normal, band % 2 == 0 ? orange : white);
        }
        mesh.addTriangle(position + Vec3{0, top, 0}, ringPoint(top, a0), ringPoint(top, a1),
                         {0, 1, 0}, orange);
    }
}

}  // namespace

bool Track::load(const std::string& path) {
    std::ifstream in(path);
    if (!in) {
        return false;
    }
    practiceArea = false;
    closed = true;

    enum class Mode { Top, Names, Samples, Pit };
    Mode mode = Mode::Top;
    int expectNames = 0;
    int expectSamples = 0;
    int expectPit = 0;
    std::vector<Vec3> pit;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::istringstream ss(line);
        if (mode == Mode::Names) {
            int id = 0;
            ss >> id;
            std::string name;
            std::getline(ss, name);
            if (!name.empty() && name[0] == ' ') {
                name.erase(0, 1);
            }
            names.push_back(name);
            if (static_cast<int>(names.size()) >= expectNames) {
                mode = Mode::Top;
            }
            continue;
        }
        if (mode == Mode::Samples) {
            TrackSample sample;
            int bridge = 0;
            ss >> sample.x >> sample.z >> sample.y >> sample.cross >> bridge >> sample.corner >> sample.nameId;
            if (ss.fail()) {
                return false;
            }
            sample.bridge = bridge != 0;
            samples.push_back(sample);
            if (static_cast<int>(samples.size()) >= expectSamples) {
                mode = Mode::Top;
            }
            continue;
        }
        if (mode == Mode::Pit) {
            Vec3 point;
            ss >> point.x >> point.z >> point.y;
            if (ss.fail()) {
                return false;
            }
            pit.push_back(point);
            if (static_cast<int>(pit.size()) >= expectPit) {
                mode = Mode::Top;
            }
            continue;
        }

        std::string key;
        ss >> key;
        if (key == "origin") {
            ss >> originLat >> originLon;
        } else if (key == "half_width") {
            ss >> halfWidth;
        } else if (key == "curb_width") {
            ss >> curbWidth;
        } else if (key == "length") {
            ss >> length;
        } else if (key == "spawn") {
            ss >> spawnX >> spawnY >> spawnZ >> spawnYaw;
        } else if (key == "names") {
            ss >> expectNames;
            mode = Mode::Names;
        } else if (key == "samples") {
            ss >> expectSamples;
            mode = Mode::Samples;
        } else if (key == "pit") {
            ss >> expectPit;
            mode = expectPit > 0 ? Mode::Pit : Mode::Top;
        }
    }

    if (samples.size() < 16) {
        return false;
    }

    length = 0;
    for (size_t i = 0; i < samples.size(); ++i) {
        samples[i].s = length;
        const TrackSample& a = samples[i];
        const TrackSample& b = samples[(i + 1) % samples.size()];
        length += std::sqrt((b.x - a.x) * (b.x - a.x) + (b.z - a.z) * (b.z - a.z));
    }

    visual = CpuMesh{};
    addGround(visual, *this);
    const float curbOuter = halfWidth + curbWidth;
    const float grassOuter = curbOuter + runoff;
    addRibbon(visual, *this, -grassOuter, -curbOuter, {0.15f, 0.36f, 0.14f}, true);
    addRibbon(visual, *this, curbOuter, grassOuter, {0.15f, 0.36f, 0.14f}, true);
    addRibbon(visual, *this, -halfWidth, halfWidth, {0.17f, 0.17f, 0.19f}, false);
    addCurbs(visual, *this);
    addRoadSpeedReferences(visual, *this);
    addWalls(visual, *this);
    addPit(visual, pit);
    addStartLine(visual, *this);
    return true;
}

void Track::makeTestCorner() {
    *this = Track{};
    closed = false;
    halfWidth = 6.5f;
    curbWidth = 0.6f;
    runoff = 4.0f;
    spawnX = 0;
    spawnY = 0;
    spawnZ = -20;
    spawnYaw = 0;
    names = {"APPROACH", "R50 CORNER", "EXIT"};

    auto sample = [&](float x, float z, float s, int section) {
        TrackSample point;
        point.x = x;
        point.z = z;
        point.s = s;
        point.nameId = section;
        point.corner = section == 1 ? 1 : 0;
        samples.push_back(point);
    };
    // Analytic s values keep gate positions identical across sampling densities.
    for (int i = 0; i <= 35; ++i) sample(0, -35.0f + i, float(i), 0);
    constexpr float halfPi = 1.5707963267948966f;
    constexpr int arcSteps = 100;
    for (int i = 1; i <= arcSteps; ++i) {
        const float t = halfPi * float(i) / float(arcSteps);
        sample(50.0f - 50.0f * std::cos(t), 50.0f * std::sin(t), 35.0f + 50.0f * t, 1);
    }
    const float arcEnd = 35.0f + 50.0f * halfPi;
    for (int i = 1; i <= 45; ++i) sample(50.0f + i, 50.0f, arcEnd + i, 2);
    length = arcEnd + 45.0f;

    // A single open ribbon makes the test geometry easy to read and avoids a
    // fictional return road across the comparison corner.
    visual.addQuad({-120, -0.04f, -140}, {-120, -0.04f, 170},
                   {220, -0.04f, 170}, {220, -0.04f, -140}, {0, 1, 0}, {0.14f, 0.29f, 0.13f});
    const float outer = halfWidth + curbWidth + runoff;
    addRibbon(visual, *this, -outer, -halfWidth - curbWidth, {0.20f, 0.36f, 0.17f}, false);
    addRibbon(visual, *this, halfWidth + curbWidth, outer, {0.20f, 0.36f, 0.17f}, false);
    addRibbon(visual, *this, -halfWidth, halfWidth, {0.19f, 0.20f, 0.22f}, false);
    addCurbs(visual, *this);
    addRoadSpeedReferences(visual, *this);
    addWalls(visual, *this);

    // Painted entry and finish gates identify the measured section without
    // obscuring the camera or changing the driving surface.
    visual.addQuad({-halfWidth, 0.018f, -0.25f}, {halfWidth, 0.018f, -0.25f},
                   {halfWidth, 0.018f, 0.25f}, {-halfWidth, 0.018f, 0.25f},
                   {0, 1, 0}, {0.92f, 0.75f, 0.24f});
    visual.addQuad({61.75f, 0.018f, 50-halfWidth}, {62.25f, 0.018f, 50-halfWidth},
                   {62.25f, 0.018f, 50+halfWidth}, {61.75f, 0.018f, 50+halfWidth},
                   {0, 1, 0}, {0.85f, 0.90f, 0.92f});
}

void Track::makePracticeArea() {
    *this = Track{};
    practiceArea = true;
    closed = false;
    halfWidth = 1000.0f;
    curbWidth = runoff = 0;
    length = 2000.0f;
    spawnX = spawnY = spawnZ = spawnYaw = 0;
    names = {"FREE PRACTICE"};
    samples = {{0, -halfWidth, 0, 0, 0, 0, 0},
               {0, halfWidth, 0, 0, length, 0, 0}};

    const float outer = halfWidth + 250.0f;
    visual.addQuad({-outer, -0.04f, -outer}, {-outer, -0.04f, outer},
                   {outer, -0.04f, outer}, {outer, -0.04f, -outer},
                   {0, 1, 0}, {0.14f, 0.29f, 0.13f});
    visual.addQuad({-halfWidth, 0, -halfWidth}, {-halfWidth, 0, halfWidth},
                   {halfWidth, 0, halfWidth}, {halfWidth, 0, -halfWidth},
                   {0, 1, 0}, {0.19f, 0.20f, 0.22f});
    // Subtle ground references show direction and distance without prescribing a route.
    for (int i = -49; i <= 49; ++i) {
        const float p = i * 20.0f;
        const bool major = i % 5 == 0;
        const float width = major ? 0.12f : 0.055f;
        const Vec3 color = major ? Vec3{0.32f, 0.34f, 0.35f} : Vec3{0.24f, 0.25f, 0.27f};
        visual.addQuad({p - width, 0.015f, -halfWidth}, {p + width, 0.015f, -halfWidth},
                       {p + width, 0.015f, halfWidth}, {p - width, 0.015f, halfWidth}, {0, 1, 0}, color);
        visual.addQuad({-halfWidth, 0.016f, p - width}, {halfWidth, 0.016f, p - width},
                       {halfWidth, 0.016f, p + width}, {-halfWidth, 0.016f, p + width}, {0, 1, 0}, color);
    }
    addPracticeSpeedReferences(visual);
    // Optional visual landmarks only: free circles, a wide gap, and an offset slalom.
    // They are not added to the driving surface or collision query.
    const Vec3 cones[] = {
        {-36, 0, 18}, {36, 0, 18},
        {-10, 0, 70}, {10, 0, 70},
        {48, 0, 100}, {62, 0, 128}, {48, 0, 156}, {62, 0, 184}, {48, 0, 212},
    };
    for (const Vec3& position : cones) addPracticeCone(visual, position);
    const Vec3 concrete{0.55f, 0.57f, 0.59f};
    const float edge = halfWidth;
    visual.addBox({-edge - 0.4f, 0, -edge - 0.4f}, {-edge, 0.9f, edge + 0.4f}, concrete);
    visual.addBox({edge, 0, -edge - 0.4f}, {edge + 0.4f, 0.9f, edge + 0.4f}, concrete);
    visual.addBox({-edge, 0, -edge - 0.4f}, {edge, 0.9f, -edge}, concrete);
    visual.addBox({-edge, 0, edge}, {edge, 0.9f, edge + 0.4f}, concrete);
}

void Track::scaleBy(float ratio) {
    if (!std::isfinite(ratio) || ratio <= 0 || ratio == 1) return;
    spawnX *= ratio;
    spawnY *= ratio;
    spawnZ *= ratio;
    halfWidth *= ratio;
    curbWidth *= ratio;
    runoff *= ratio;
    length *= ratio;
    for (auto& sample : samples) {
        sample.x *= ratio;
        sample.y *= ratio;
        sample.z *= ratio;
        sample.s *= ratio;
    }
    for (size_t vertex = 0; vertex + 2 < visual.vertices.size(); vertex += 9) {
        visual.vertices[vertex] *= ratio;
        visual.vertices[vertex + 1] *= ratio;
        visual.vertices[vertex + 2] *= ratio;
    }
}

TrackPose Track::query(float x, float z, float yHint, float preferS) const {
    if (practiceArea) {
        TrackPose pose;
        if (!std::isfinite(x) || !std::isfinite(z) || !std::isfinite(yHint)) return pose;
        pose.found = true;
        pose.nameId = 0;
        pose.s = z + halfWidth;
        pose.xzDist = std::fmax(std::fabs(x), std::fabs(z));
        // Report the nearest square perimeter normal to the existing collision response.
        // Inside, every point belongs to the same flat asphalt surface.
        if (std::fabs(x) >= std::fabs(z)) {
            pose.lateral = x;
            pose.rightX = 1;
            pose.rightZ = 0;
            pose.yaw = 0;
        } else {
            pose.lateral = z;
            pose.rightX = 0;
            pose.rightZ = 1;
            pose.yaw = -1.5707963267948966f;
        }
        return pose;
    }
    TrackPose best;
    float bestScore = 1.0e30f;
    const int n = static_cast<int>(samples.size());
    for (int i = 0; i < n - (closed ? 0 : 1); ++i) {
        const TrackSample& a = samples[i];
        const TrackSample& b = samples[(i + 1) % n];
        float abx = b.x - a.x;
        float abz = b.z - a.z;
        float len2 = abx * abx + abz * abz;
        if (len2 < 1e-4f) {
            continue;
        }
        float t = ((x - a.x) * abx + (z - a.z) * abz) / len2;
        t = clampf(t, 0.0f, 1.0f);
        float cx = a.x + abx * t;
        float cz = a.z + abz * t;
        float cy = a.y + (b.y - a.y) * t;
        float dx = x - cx;
        float dz = z - cz;
        float dy = yHint - cy;
        // 立体交差では平面距離より高さを優先する。6m 差なら、壁際にいても下の路面へ落ちない。
        float score = dx * dx + dz * dz + dy * dy * 12.0f;
        if (preferS >= 0.0f && length > 1.0f) {
            float along = a.s + std::sqrt(len2) * t;
            float ds = std::fabs(along - preferS);
            if (closed && ds > length * 0.5f) {
                ds = length - ds;
            }
            // 今走っている周回位置から遠い路面は、交差の真下でも選ばない。
            score += ds * 0.2f;
        }
        if (score >= bestScore) {
            continue;
        }
        bestScore = score;
        float invLen = 1.0f / std::sqrt(len2);
        float tx = abx * invLen;
        float tz = abz * invLen;
        best.found = true;
        best.s = a.s + std::sqrt(len2) * t;
        if (closed && best.s >= length) {
            best.s -= length;
        }
        best.rightX = tz;
        best.rightZ = -tx;
        best.lateral = dx * best.rightX + dz * best.rightZ;
        best.centerY = cy;
        best.cross = a.cross + (b.cross - a.cross) * t;
        best.yaw = std::atan2(tx, tz);
        best.xzDist = std::sqrt(dx * dx + dz * dz);
        best.corner = t < 0.5f ? a.corner : b.corner;
        best.nameId = t < 0.5f ? a.nameId : b.nameId;
    }
    return best;
}

const char* Track::nameOf(int nameId) const {
    if (nameId < 0 || nameId >= static_cast<int>(names.size())) {
        return "";
    }
    return names[nameId].c_str();
}
