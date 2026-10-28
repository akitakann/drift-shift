#include "render.hpp"

#include <SDL.h>

#include <cmath>
#include <cstdint>
#include <cstring>

#ifdef __EMSCRIPTEN__
#include <GLES3/gl3.h>
#else
#define GL_GLEXT_PROTOTYPES 1
#include <GL/gl.h>
#include <GL/glext.h>
#endif

namespace {

const char* kVersion =
#ifdef __EMSCRIPTEN__
    "#version 300 es\nprecision highp float;\n";
#else
    "#version 330 core\n";
#endif

const char* kWorldVs = R"GLSL(
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec3 aColor;
uniform mat4 uViewProj;
uniform mat4 uModel;
out vec3 vNormal;
out vec3 vColor;
out vec3 vWorld;
out vec2 vParticle;
void main() {
    vec4 world = uModel * vec4(aPos, 1.0);
    vWorld = world.xyz;
    vNormal = mat3(uModel) * aNormal;
    vColor = aColor;
    vParticle = aPos.xy;
    gl_Position = uViewProj * world;
}
)GLSL";

const char* kWorldFs = R"GLSL(
in vec3 vNormal;
in vec3 vColor;
in vec3 vWorld;
in vec2 vParticle;
uniform vec3 uCamPos;
uniform float uOpacity;
uniform int uSmoke;
out vec4 fragColor;
void main() {
    vec3 n = normalize(vNormal);
    if (!gl_FrontFacing) n = -n;
    vec3 sun = normalize(vec3(0.35, 0.86, 0.28));
    float light = 0.36 + 0.64 * max(dot(n, sun), 0.0);
    vec3 color = vColor * light;
    float dist = distance(vWorld, uCamPos);
    float fog = clamp((dist - 90.0) / 1500.0, 0.0, 1.0);
    vec3 haze = vec3(0.62, 0.74, 0.84);
    float alpha = uOpacity;
    if (uSmoke != 0) {
        if (uSmoke == 1 || uSmoke == 3) alpha *= 1.0 - smoothstep(0.12, 1.0, length(vParticle));
        color = vColor;
    }
    fragColor = vec4(mix(color, haze, fog), alpha);
}
)GLSL";

const char* kSkyVs = R"GLSL(
out float vY;
void main() {
    vec2 pos = vec2((gl_VertexID == 2) ? 3.0 : -1.0, (gl_VertexID == 1) ? 3.0 : -1.0);
    vY = pos.y;
    gl_Position = vec4(pos, 1.0, 1.0);
}
)GLSL";

const char* kSkyFs = R"GLSL(
in float vY;
out vec4 fragColor;
void main() {
    float t = clamp(vY * 0.5 + 0.5, 0.0, 1.0);
    vec3 horizon = vec3(0.62, 0.74, 0.84);
    vec3 zenith = vec3(0.28, 0.48, 0.78);
    fragColor = vec4(mix(horizon, zenith, smoothstep(0.0, 1.0, t)), 1.0);
}
)GLSL";

const char* kHudVs = R"GLSL(
layout(location = 0) in vec2 aPos;
layout(location = 1) in vec4 aColor;
uniform mat4 uOrtho;
out vec4 vColor;
void main() {
    vColor = aColor;
    gl_Position = uOrtho * vec4(aPos, 0.0, 1.0);
}
)GLSL";

const char* kHudFs = R"GLSL(
in vec4 vColor;
out vec4 fragColor;
void main() {
    fragColor = vColor;
}
)GLSL";

const char* kPostVs = R"GLSL(
out vec2 vUv;
void main() {
    vec2 pos = vec2((gl_VertexID == 2) ? 3.0 : -1.0, (gl_VertexID == 1) ? 3.0 : -1.0);
    vUv = pos * 0.5 + 0.5;
    gl_Position = vec4(pos, 0.0, 1.0);
}
)GLSL";

const char* kPostFs = R"GLSL(
in vec2 vUv;
uniform sampler2D uWorld;
uniform float uBlurAmount;
out vec4 fragColor;
void main() {
    vec4 original = texture(uWorld, vUv);
    // Texture coordinates have their origin at the bottom. Preserve the next
    // stretch of road and the broad lower-middle region occupied by the car.
    float gaze = length((vUv - vec2(0.5, 0.54)) / vec2(0.25, 0.23));
    float car = length((vUv - vec2(0.5, 0.30)) / vec2(0.32, 0.30));
    float peripheral = smoothstep(0.90, 1.60, gaze) * smoothstep(0.90, 1.40, car);
    float strength = clamp(uBlurAmount, 0.0, 2.0) * peripheral;
    if (strength < 0.0001) {
        fragColor = original;
        return;
    }
    vec2 radial = vUv - vec2(0.5, 0.54);
    vec2 stepUv = radial / max(length(radial), 0.0001) * (0.018 * strength / 8.0);
    vec3 sum = original.rgb * 2.0;
    float total = 2.0;
    for (int i = 1; i <= 8; ++i) {
        float weight = 1.0 - float(i) * 0.075;
        sum += texture(uWorld, clamp(vUv - stepUv * float(i), vec2(0.0), vec2(1.0))).rgb * weight;
        total += weight;
    }
    fragColor = vec4(mix(original.rgb, sum / total, peripheral), original.a);
}
)GLSL";

// 5x7。bit4 が左端。並びは 0-9 A-Z . : - / %
constexpr char kGlyphOrder[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ.:-/%";
const std::uint8_t kFont[][7] = {
    {0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E}, {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E},
    {0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F}, {0x0E, 0x11, 0x01, 0x06, 0x01, 0x11, 0x0E},
    {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02}, {0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E},
    {0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E}, {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08},
    {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E}, {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C},
    {0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11}, {0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E},
    {0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E}, {0x1E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1E},
    {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F}, {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10},
    {0x0E, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0E}, {0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11},
    {0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E}, {0x01, 0x01, 0x01, 0x01, 0x11, 0x11, 0x0E},
    {0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11}, {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F},
    {0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11}, {0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11},
    {0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E}, {0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10},
    {0x0E, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0D}, {0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11},
    {0x0E, 0x11, 0x10, 0x0E, 0x01, 0x11, 0x0E}, {0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04},
    {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E}, {0x11, 0x11, 0x11, 0x11, 0x11, 0x0A, 0x04},
    {0x11, 0x11, 0x11, 0x15, 0x15, 0x1B, 0x11}, {0x11, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x11},
    {0x11, 0x11, 0x0A, 0x04, 0x04, 0x04, 0x04}, {0x1F, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1F},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C}, {0x00, 0x0C, 0x0C, 0x00, 0x0C, 0x0C, 0x00},
    {0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00}, {0x01, 0x02, 0x04, 0x04, 0x08, 0x10, 0x10},
    {0x19, 0x1A, 0x02, 0x04, 0x08, 0x0B, 0x13},
};
static_assert(sizeof(kFont) / sizeof(kFont[0]) == sizeof(kGlyphOrder) - 1, "glyph count");

GLuint compileShader(GLenum type, const char* source) {
    const char* parts[] = {kVersion, source};
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 2, parts, nullptr);
    glCompileShader(shader);
    GLint ok = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[1024];
        glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
        SDL_Log("shader compile failed: %s", log);
        glDeleteShader(shader);
        return 0;
    }
    return shader;
}

GLuint linkProgram(const char* vs, const char* fs) {
    GLuint vertex = compileShader(GL_VERTEX_SHADER, vs);
    GLuint fragment = compileShader(GL_FRAGMENT_SHADER, fs);
    if (!vertex || !fragment) {
        if (vertex) glDeleteShader(vertex);
        if (fragment) glDeleteShader(fragment);
        return 0;
    }
    GLuint program = glCreateProgram();
    glAttachShader(program, vertex);
    glAttachShader(program, fragment);
    glLinkProgram(program);
    glDeleteShader(vertex);
    glDeleteShader(fragment);
    GLint ok = 0;
    glGetProgramiv(program, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[1024];
        glGetProgramInfoLog(program, sizeof(log), nullptr, log);
        SDL_Log("shader link failed: %s", log);
        glDeleteProgram(program);
        return 0;
    }
    return program;
}

int glyphIndex(char c) {
    if (c >= 'a' && c <= 'z') {
        c = static_cast<char>(c - 'a' + 'A');
    }
    const char* found = std::strchr(kGlyphOrder, c);
    if (!found) {
        return -1;
    }
    return static_cast<int>(found - kGlyphOrder);
}

}  // namespace

bool Renderer::init() {
    glGenVertexArrays(1, &dummyVao);
    worldProgram = linkProgram(kWorldVs, kWorldFs);
    skyProgram = linkProgram(kSkyVs, kSkyFs);
    hudProgram = linkProgram(kHudVs, kHudFs);
    if (!worldProgram || !skyProgram || !hudProgram) {
        shutdown();
        return false;
    }
    uViewProj = glGetUniformLocation(worldProgram, "uViewProj");
    uModel = glGetUniformLocation(worldProgram, "uModel");
    uCamPos = glGetUniformLocation(worldProgram, "uCamPos");
    uOrtho = glGetUniformLocation(hudProgram, "uOrtho");
    uOpacity = glGetUniformLocation(worldProgram, "uOpacity");
    uSmoke = glGetUniformLocation(worldProgram, "uSmoke");
    // Post processing is optional. A failed shader must not prevent the game
    // from using its original direct world/HUD drawing path.
    postProgram_ = linkProgram(kPostVs, kPostFs);
    postAvailable = postProgram_ != 0;
    if (postProgram_) {
        uPostTexture_ = glGetUniformLocation(postProgram_, "uWorld");
        uPostAmount_ = glGetUniformLocation(postProgram_, "uBlurAmount");
    }

    glGenVertexArrays(1, &hudVao);
    glGenBuffers(1, &hudVbo);
    glGenBuffers(1, &hudIbo);
    glBindVertexArray(hudVao);
    glBindBuffer(GL_ARRAY_BUFFER, hudVbo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, hudIbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(0));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(2 * sizeof(float)));
    glBindVertexArray(dummyVao);

    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    // lookAt は左右を直すために鏡映になっている。巻き方向が反転するので、表の判定も合わせる。
    glFrontFace(GL_CW);
    resize(width, height);
    return true;
}

void Renderer::clearFrame(float blurAmount) {
    if (postFrame_) finishWorld();
    const float requested = std::isfinite(blurAmount) ? clampf(blurAmount, 0, 2) : 0;
    appliedBlur = 0;
    postFrame_ = drawable_ && requested > 0 && ensurePostTarget();
    if (postFrame_) appliedBlur = requested;
    glBindFramebuffer(GL_FRAMEBUFFER, postFrame_ ? postFbo_ : 0);
    glViewport(0, 0, drawable_ ? width : 0, drawable_ ? height : 0);
    glDepthMask(GL_TRUE);
    glClearColor(0.62f, 0.74f, 0.84f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Renderer::resize(int newWidth, int newHeight) {
    drawable_ = newWidth > 0 && newHeight > 0;
    width = newWidth > 0 ? newWidth : 1;
    height = newHeight > 0 ? newHeight : 1;
    glViewport(0, 0, drawable_ ? width : 0, drawable_ ? height : 0);
}

void Renderer::releasePostTarget() {
    if (postFbo_) glDeleteFramebuffers(1, &postFbo_);
    if (postColor_) glDeleteTextures(1, &postColor_);
    if (postDepth_) glDeleteRenderbuffers(1, &postDepth_);
    postFbo_ = postColor_ = postDepth_ = 0;
    postWidth_ = postHeight_ = 0;
}

bool Renderer::ensurePostTarget() {
    if (!postProgram_ || !drawable_) return false;
    if (postFbo_ && postWidth_ == width && postHeight_ == height) {
        postAvailable = true;
        return true;
    }
    // Do not repeatedly allocate a failed size every frame. A later resize can
    // try again, while normal frames continue directly on the default target.
    if (attemptedPostWidth_ == width && attemptedPostHeight_ == height) return false;
    attemptedPostWidth_ = width;
    attemptedPostHeight_ = height;
    GLint maxTexture = 0, maxRenderbuffer = 0;
    glGetIntegerv(GL_MAX_TEXTURE_SIZE, &maxTexture);
    glGetIntegerv(GL_MAX_RENDERBUFFER_SIZE, &maxRenderbuffer);
    if (width > maxTexture || height > maxTexture || width > maxRenderbuffer || height > maxRenderbuffer) {
        postAvailable = false;
        return false;
    }
    GLint drawFbo = 0, readFbo = 0, renderbuffer = 0, activeTexture = 0, texture = 0, unpackBuffer = 0;
    glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &drawFbo);
    glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &readFbo);
    glGetIntegerv(GL_RENDERBUFFER_BINDING, &renderbuffer);
    glGetIntegerv(GL_ACTIVE_TEXTURE, &activeTexture);
    glActiveTexture(GL_TEXTURE0);
    glGetIntegerv(GL_TEXTURE_BINDING_2D, &texture);
    glGetIntegerv(GL_PIXEL_UNPACK_BUFFER_BINDING, &unpackBuffer);
    glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
    releasePostTarget();
    glGenFramebuffers(1, &postFbo_);
    glGenTextures(1, &postColor_);
    glGenRenderbuffers(1, &postDepth_);
    bool complete = false;
    if (postFbo_ && postColor_ && postDepth_) {
        glBindTexture(GL_TEXTURE_2D, postColor_);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
        glBindRenderbuffer(GL_RENDERBUFFER, postDepth_);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width, height);
        glBindFramebuffer(GL_FRAMEBUFFER, postFbo_);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, postColor_, 0);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, postDepth_);
        complete = glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
    }
    if (complete) {
        postWidth_ = width;
        postHeight_ = height;
    } else {
        SDL_Log("radial blur target unavailable at %dx%d; using direct rendering", width, height);
        releasePostTarget();
    }
    glBindBuffer(GL_PIXEL_UNPACK_BUFFER, static_cast<GLuint>(unpackBuffer));
    glBindTexture(GL_TEXTURE_2D, static_cast<GLuint>(texture));
    glActiveTexture(static_cast<GLenum>(activeTexture));
    glBindRenderbuffer(GL_RENDERBUFFER, static_cast<GLuint>(renderbuffer));
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, static_cast<GLuint>(drawFbo));
    glBindFramebuffer(GL_READ_FRAMEBUFFER, static_cast<GLuint>(readFbo));
    postAvailable = complete;
    return complete;
}

void Renderer::finishWorld() {
    if (!postFrame_) return;
    postFrame_ = false;
    GLint program = 0, vao = 0, activeTexture = 0, texture = 0, sampler = 0;
    GLboolean depthMask = GL_TRUE, colorMask[4] = {GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE};
    glGetIntegerv(GL_CURRENT_PROGRAM, &program);
    glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &vao);
    glGetIntegerv(GL_ACTIVE_TEXTURE, &activeTexture);
    glActiveTexture(GL_TEXTURE0);
    glGetIntegerv(GL_TEXTURE_BINDING_2D, &texture);
    glGetIntegerv(GL_SAMPLER_BINDING, &sampler);
    glGetBooleanv(GL_DEPTH_WRITEMASK, &depthMask);
    glGetBooleanv(GL_COLOR_WRITEMASK, colorMask);
    const GLenum capabilities[] = {GL_DEPTH_TEST, GL_BLEND, GL_CULL_FACE, GL_SCISSOR_TEST, GL_STENCIL_TEST, GL_DITHER};
    GLboolean enabled[sizeof(capabilities) / sizeof(capabilities[0])];
    for (unsigned i = 0; i < sizeof(capabilities) / sizeof(capabilities[0]); ++i) {
        enabled[i] = glIsEnabled(capabilities[i]);
        glDisable(capabilities[i]);
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, drawable_ ? width : 0, drawable_ ? height : 0);
    glDepthMask(GL_FALSE);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glUseProgram(postProgram_);
    glBindVertexArray(dummyVao);
    glBindSampler(0, 0);
    glBindTexture(GL_TEXTURE_2D, postColor_);
    glUniform1i(uPostTexture_, 0);
    glUniform1f(uPostAmount_, appliedBlur);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glBindTexture(GL_TEXTURE_2D, static_cast<GLuint>(texture));
    glBindSampler(0, static_cast<GLuint>(sampler));
    glActiveTexture(static_cast<GLenum>(activeTexture));
    glUseProgram(static_cast<GLuint>(program));
    glBindVertexArray(static_cast<GLuint>(vao));
    glDepthMask(depthMask);
    glColorMask(colorMask[0], colorMask[1], colorMask[2], colorMask[3]);
    for (unsigned i = 0; i < sizeof(capabilities) / sizeof(capabilities[0]); ++i) {
        if (enabled[i]) glEnable(capabilities[i]);
        else glDisable(capabilities[i]);
    }
}

void Renderer::shutdown() {
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    releasePostTarget();
    if (postProgram_) glDeleteProgram(postProgram_);
    if (worldProgram) glDeleteProgram(worldProgram);
    if (skyProgram) glDeleteProgram(skyProgram);
    if (hudProgram) glDeleteProgram(hudProgram);
    if (hudVbo) glDeleteBuffers(1, &hudVbo);
    if (hudIbo) glDeleteBuffers(1, &hudIbo);
    if (hudVao) glDeleteVertexArrays(1, &hudVao);
    if (dummyVao) glDeleteVertexArrays(1, &dummyVao);
    postProgram_ = worldProgram = skyProgram = hudProgram = 0;
    hudVbo = hudIbo = hudVao = dummyVao = 0;
    uPostTexture_ = uPostAmount_ = -1;
    uViewProj = uModel = uCamPos = uOrtho = uOpacity = uSmoke = -1;
    attemptedPostWidth_ = attemptedPostHeight_ = 0;
    postAvailable = postFrame_ = false;
    appliedBlur = 0;
    hudVerts.clear();
    hudIndices.clear();
}

void Renderer::drawSky() {
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    glUseProgram(skyProgram);
    glBindVertexArray(dummyVao);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);
}

void Renderer::beginWorld(const Mat4& viewProj, Vec3 cameraPos) {
    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    glUseProgram(worldProgram);
    glUniformMatrix4fv(uViewProj, 1, GL_FALSE, viewProj.m);
    glUniform3f(uCamPos, cameraPos.x, cameraPos.y, cameraPos.z);
    glUniform1f(uOpacity, 1);
    glUniform1i(uSmoke, 0);
}

void Renderer::draw(const GpuMesh& mesh, const Mat4& model) {
    if (mesh.indexCount <= 0) {
        return;
    }
    glBindVertexArray(mesh.vao);
    glUniformMatrix4fv(uModel, 1, GL_FALSE, model.m);
    glDrawElements(GL_TRIANGLES, mesh.indexCount, GL_UNSIGNED_INT, nullptr);
}

void Renderer::drawSmoke(const GpuMesh& mesh, const Mat4& model, float opacity) {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    glUniform1f(uOpacity, clampf(opacity, 0, 1));
    glUniform1i(uSmoke, 1);
    draw(mesh, model);
    glUniform1f(uOpacity, 1);
    glUniform1i(uSmoke, 0);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
}

void Renderer::drawGlow(const GpuMesh& mesh, const Mat4& model, float opacity, bool radial) {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE);
    glDepthMask(GL_FALSE);
    glUniform1f(uOpacity, clampf(opacity, 0, 1));
    glUniform1i(uSmoke, radial ? 3 : 2);
    draw(mesh, model);
    glUniform1f(uOpacity, 1);
    glUniform1i(uSmoke, 0);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
}

void Renderer::beginHud() {
    hudVerts.clear();
    hudIndices.clear();
}

void Renderer::addQuad(float x0, float y0, float x1, float y1, float r, float g, float b, float a) {
    unsigned base = static_cast<unsigned>(hudVerts.size() / 6);
    float verts[] = {
        x0, y0, r, g, b, a, x1, y0, r, g, b, a, x1, y1, r, g, b, a, x0, y1, r, g, b, a,
    };
    hudVerts.insert(hudVerts.end(), verts, verts + 24);
    unsigned idx[] = {base, base + 1, base + 2, base, base + 2, base + 3};
    hudIndices.insert(hudIndices.end(), idx, idx + 6);
}

void Renderer::drawLine(float x0, float y0, float x1, float y1, float thickness, float r, float g, float b, float a) {
    float dx = x1 - x0;
    float dy = y1 - y0;
    float len = std::sqrt(dx * dx + dy * dy);
    if (len < 0.001f) {
        return;
    }
    dx /= len;
    dy /= len;
    float px = -dy * thickness * 0.5f;
    float py = dx * thickness * 0.5f;
    unsigned base = static_cast<unsigned>(hudVerts.size() / 6);
    float verts[] = {
        x0 + px, y0 + py, r, g, b, a, x0 - px, y0 - py, r, g, b, a,
        x1 - px, y1 - py, r, g, b, a, x1 + px, y1 + py, r, g, b, a,
    };
    hudVerts.insert(hudVerts.end(), verts, verts + 24);
    unsigned idx[] = {base, base + 1, base + 2, base, base + 2, base + 3};
    hudIndices.insert(hudIndices.end(), idx, idx + 6);
}

void Renderer::drawText(float x, float y, float scale, const char* text, float r, float g, float b, float a) {
    float pen = x;
    for (const char* cursor = text; *cursor; ++cursor) {
        char c = *cursor;
        if (c >= 'a' && c <= 'z') {
            c = static_cast<char>(c - 'a' + 'A');
        }
        if (c == ' ') {
            pen += 4.0f * scale;
            continue;
        }
        int index = glyphIndex(c);
        if (index < 0) {
            pen += 6.0f * scale;
            continue;
        }
        const std::uint8_t* rows = kFont[index];
        for (int row = 0; row < 7; ++row) {
            for (int col = 0; col < 5; ++col) {
                if ((rows[row] & (0x10 >> col)) == 0) {
                    continue;
                }
                float x0 = pen + float(col) * scale;
                float y0 = y + float(row) * scale;
                addQuad(x0, y0, x0 + scale, y0 + scale, r, g, b, a);
            }
        }
        pen += 6.0f * scale;
    }
}

void Renderer::flushHud() {
    if (hudIndices.empty()) {
        return;
    }
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glUseProgram(hudProgram);
    Mat4 ortho = orthoTopLeft(float(width), float(height));
    glUniformMatrix4fv(uOrtho, 1, GL_FALSE, ortho.m);
    glBindVertexArray(hudVao);
    glBindBuffer(GL_ARRAY_BUFFER, hudVbo);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(hudVerts.size() * sizeof(float)), hudVerts.data(), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, hudIbo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(hudIndices.size() * sizeof(unsigned)), hudIndices.data(), GL_DYNAMIC_DRAW);
    glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(hudIndices.size()), GL_UNSIGNED_INT, nullptr);
    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
}

GpuMesh Renderer::upload(const CpuMesh& mesh) {
    GpuMesh gpu;
    if (mesh.indices.empty()) {
        return gpu;
    }
    glGenVertexArrays(1, &gpu.vao);
    glGenBuffers(1, &gpu.vbo);
    glGenBuffers(1, &gpu.ibo);
    glBindVertexArray(gpu.vao);
    glBindBuffer(GL_ARRAY_BUFFER, gpu.vbo);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(mesh.vertices.size() * sizeof(float)), mesh.vertices.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, gpu.ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(mesh.indices.size() * sizeof(std::uint32_t)), mesh.indices.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), reinterpret_cast<void*>(0));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), reinterpret_cast<void*>(3 * sizeof(float)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), reinterpret_cast<void*>(6 * sizeof(float)));
    gpu.indexCount = static_cast<int>(mesh.indices.size());
    glBindVertexArray(dummyVao);
    return gpu;
}

void Renderer::updateMesh(GpuMesh& gpu, const CpuMesh& mesh) {
    if (!gpu.vao && !mesh.indices.empty()) {
        gpu = upload(mesh);
        return;
    }
    gpu.indexCount = static_cast<int>(mesh.indices.size());
    if (!gpu.vao || mesh.indices.empty()) return;
    glBindVertexArray(gpu.vao);
    glBindBuffer(GL_ARRAY_BUFFER, gpu.vbo);
    glBufferData(GL_ARRAY_BUFFER, mesh.vertices.size() * sizeof(float), mesh.vertices.data(), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, gpu.ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, mesh.indices.size() * sizeof(std::uint32_t), mesh.indices.data(), GL_DYNAMIC_DRAW);
    glBindVertexArray(dummyVao);
}
