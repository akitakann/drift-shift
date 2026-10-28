#pragma once

#include "geom.hpp"

#include <cstdint>
#include <utility>
#include <vector>

// 頂点は xyz + 法線 + 色。インデックスは三角形。
struct CpuMesh {
    std::vector<float> vertices;
    std::vector<std::uint32_t> indices;

    void addVertex(Vec3 p, Vec3 n, Vec3 c) {
        vertices.insert(vertices.end(), {p.x, p.y, p.z, n.x, n.y, n.z, c.x, c.y, c.z});
    }

    void addRawTriangle(std::uint32_t a, std::uint32_t b, std::uint32_t c) {
        indices.insert(indices.end(), {a, b, c});
    }

    // a,b,c,d は四角の周囲。法線 n と逆向きなら巻きを直す。
    void addQuad(Vec3 a, Vec3 b, Vec3 c, Vec3 d, Vec3 n, Vec3 color) {
        if (dot(cross(b - a, c - a), n) < 0.0f) {
            std::swap(b, d);
        }
        std::uint32_t base = static_cast<std::uint32_t>(vertices.size() / 9);
        addVertex(a, n, color);
        addVertex(b, n, color);
        addVertex(c, n, color);
        addVertex(d, n, color);
        addRawTriangle(base, base + 1, base + 2);
        addRawTriangle(base, base + 2, base + 3);
    }

    void addTriangle(Vec3 a, Vec3 b, Vec3 c, Vec3 n, Vec3 color) {
        if (dot(cross(b - a, c - a), n) < 0.0f) {
            std::swap(b, c);
        }
        std::uint32_t base = static_cast<std::uint32_t>(vertices.size() / 9);
        addVertex(a, n, color);
        addVertex(b, n, color);
        addVertex(c, n, color);
        addRawTriangle(base, base + 1, base + 2);
    }

    void addBox(Vec3 mn, Vec3 mx, Vec3 color) {
        Vec3 p[8] = {
            {mn.x, mn.y, mn.z}, {mx.x, mn.y, mn.z}, {mx.x, mx.y, mn.z}, {mn.x, mx.y, mn.z},
            {mn.x, mn.y, mx.z}, {mx.x, mn.y, mx.z}, {mx.x, mx.y, mx.z}, {mn.x, mx.y, mx.z},
        };
        addQuad(p[3], p[2], p[6], p[7], {0, 1, 0}, color);
        addQuad(p[0], p[1], p[5], p[4], {0, -1, 0}, color);
        addQuad(p[1], p[2], p[6], p[5], {1, 0, 0}, color);
        addQuad(p[0], p[4], p[7], p[3], {-1, 0, 0}, color);
        addQuad(p[4], p[5], p[6], p[7], {0, 0, 1}, color);
        addQuad(p[0], p[3], p[2], p[1], {0, 0, -1}, color);
    }
};
