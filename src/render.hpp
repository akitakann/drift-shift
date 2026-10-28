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
    void clearFrame(float blurAmount = 0);
    void finishWorld(); // Composite before drawing the HUD; safe to call when blur is off.
    void shutdown(); // Call while the GL context still exists.
    void drawSky();
    void beginWorld(const Mat4& viewProj, Vec3 cameraPos);
    void draw(const GpuMesh& mesh, const Mat4& model);
    void drawSmoke(const GpuMesh& mesh, const Mat4& model, float opacity);
    void drawGlow(const GpuMesh& mesh, const Mat4& model, float opacity, bool radial = false);
    void beginHud();
    void addQuad(float x0, float y0, float x1, float y1, float r, float g, float b, float a);
    void drawLine(float x0, float y0, float x1, float y1, float thickness, float r, float g, float b, float a);
    void drawText(float x, float y, float scale, const char* text, float r, float g, float b, float a);
    void flushHud();
    GpuMesh upload(const CpuMesh& mesh);
    void updateMesh(GpuMesh& gpu, const CpuMesh& mesh);

    unsigned worldProgram = 0;
    unsigned skyProgram = 0;
    unsigned hudProgram = 0;
    unsigned dummyVao = 0;
    unsigned hudVao = 0;
    unsigned hudVbo = 0;
    unsigned hudIbo = 0;
    bool postAvailable = false; // Optional post shader/target support; failure keeps direct rendering usable.
    float appliedBlur = 0; // Actual 0..2 strength for this frame, retained after finishWorld().
    int uViewProj = -1;
    int uModel = -1;
    int uCamPos = -1;
    int uOrtho = -1;
    int uOpacity = -1;
    int uSmoke = -1;
    std::vector<float> hudVerts;
    std::vector<unsigned> hudIndices;

private:
    unsigned postProgram_ = 0, postFbo_ = 0, postColor_ = 0, postDepth_ = 0;
    int uPostTexture_ = -1, uPostAmount_ = -1;
    int postWidth_ = 0, postHeight_ = 0;
    int attemptedPostWidth_ = 0, attemptedPostHeight_ = 0;
    bool drawable_ = true, postFrame_ = false;
    bool ensurePostTarget();
    void releasePostTarget();
};
