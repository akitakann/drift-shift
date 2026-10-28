#pragma once

#include "meshbuild.hpp"

#include <string>
#include <vector>

struct TrackPose {
    bool found = false;
    float s = 0;
    float lateral = 0;  // 右が正
    float centerY = 0;
    float cross = 0;  // 右方向 1m あたりの高さ
    float yaw = 0;
    float rightX = 1;
    float rightZ = 0;
    float xzDist = 0;
    int corner = 0;
    int nameId = -1;
};

struct TrackSample {
    float x = 0;
    float z = 0;
    float y = 0;
    float cross = 0;
    float s = 0;
    int corner = 0;
    int nameId = -1;
    bool bridge = false;
};

// 鈴鹿サーキット。中心線は OSM、高さは国土地理院の標高。
struct Track {
    bool closed = true;
    bool practiceArea = false;
    float originLat = 0;
    float originLon = 0;
    float halfWidth = 6.5f;
    float curbWidth = 0.8f;
    float runoff = 8.0f;
    float length = 0;
    float spawnX = 0;
    float spawnY = 0;
    float spawnZ = 0;
    float spawnYaw = 0;
    std::vector<TrackSample> samples;
    std::vector<std::string> names;
    // Static scenery and near-field speed references share the road's one draw.
    // Reflector posts/paint do not add obstacles or alter the surface query.
    CpuMesh visual;

    bool load(const std::string& path);
    // Flat, open R50 right-angle comparison corner. Coordinates are generated here,
    // independently of the OSM/GSI data used by the Suzuka course.
    void makeTestCorner();
    // Flat, broad paved area with no prescribed driving line or gates.
    void makePracticeArea();
    // Uniform world-space scaling; vehicle and camera dimensions are independent.
    // Invalid/nonpositive ratios leave the loaded/generated course unchanged.
    void scaleBy(float ratio);
    // preferS が 0 以上のときは、その周回位置に近い路面を優先する。立体交差で上下を取り違えない。
    TrackPose query(float x, float z, float yHint, float preferS = -1.0f) const;
    float wallLateral() const { return halfWidth + curbWidth + runoff; }
    const char* nameOf(int nameId) const;
};
