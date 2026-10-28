#pragma once

#include "geom.hpp"
#include "meshbuild.hpp"

#include <vector>

struct GpuMesh {
    unsigned vao = 0;
    unsigned vbo = 0;
    unsigned ibo = 0;
    int indexCount = 0;
};

struct Renderer {
    int width = 1280;
    int height = 720;

    bool init();
    void resize(int width, int height);
    void clearFrame();
    void drawSky();
    void beginWorld(const Mat4& viewProj, Vec3 cameraPos);
    void draw(const GpuMesh& mesh, const Mat4& model);
    void beginHud();
    void addQuad(float x0, float y0, float x1, float y1, float r, float g, float b, float a);
    void drawLine(float x0, float y0, float x1, float y1, float thickness, float r, float g, float b, float a);
    void drawText(float x, float y, float scale, const char* text, float r, float g, float b, float a);
    void flushHud();
    GpuMesh upload(const CpuMesh& mesh);

    unsigned worldProgram = 0;
    unsigned skyProgram = 0;
    unsigned hudProgram = 0;
    unsigned dummyVao = 0;
    unsigned hudVao = 0;
    unsigned hudVbo = 0;
    unsigned hudIbo = 0;
    int uViewProj = -1;
    int uModel = -1;
    int uCamPos = -1;
    int uOrtho = -1;
    std::vector<float> hudVerts;
    std::vector<unsigned> hudIndices;
};
