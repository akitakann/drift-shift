#include "car.hpp"

#include <cmath>

namespace {

constexpr float kMaxSpeed = 82.0f;      // 約 295 km/h
constexpr float kMaxReverse = 9.0f;
constexpr float kAccel = 13.0f;
constexpr float kBrake = 30.0f;
constexpr float kReverseAccel = 8.0f;
constexpr float kDrag = 0.00045f;
constexpr float kRolling = 0.9f;
constexpr float kWheelbase = 3.05f;
constexpr float kSteerRate = 2.6f;
constexpr float kWheelRadius = 0.34f;

float maxSteerAt(float speed) {
    return 0.46f / (1.0f + std::fabs(speed) * 0.085f);
}

}  // namespace

void Car::reset(const Track& track) {
    x = track.spawnX;
    y = track.spawnY;
    z = track.spawnZ;
    yaw = track.spawnYaw;
    speed = 0;
    steerAngle = 0;
    wheelSpin = 0;
    brakeHold = 0;
    onAsphalt = true;
    ground = track.query(x, z, y);
}

void Car::stepGrip(float dt) {
    // ------------------------------------------------------------------------
    // グリップ走行。ドリフトはここには書いていない。
    //
    // 速度は車体の前方だけを向く。横滑りはしない。
    // 走っているときの曲がりは、滑らない自転車モデル（ホイールベースと前輪の切れ角）。
    // ほぼ止まっているときは、据え切りで少しだけ向きが変わる。壁に向かって止まっても戻れる。
    //
    // ドリフトの課題では、この関数を差し替える。
    // たとえば横方向の速度、スリップ角、前後で違うグリップをここに足す。
    // アクセル、ブレーキ、壁との接触は update() 側にある。
    // ------------------------------------------------------------------------
    float yawRate = 0;
    if (std::fabs(speed) > 0.05f) {
        yawRate = (speed / kWheelbase) * std::tan(steerAngle);
    }
    float slow = clampf(1.0f - std::fabs(speed) / 8.0f, 0.0f, 1.0f);
    yawRate += steerAngle * 0.9f * slow;
    yaw += yawRate * dt;

    x += std::sin(yaw) * speed * dt;
    z += std::cos(yaw) * speed * dt;
}

void Car::update(float dt, const DriveInput& input, const Track& track) {
    float target = clampf(input.steer, -1.0f, 1.0f) * maxSteerAt(speed);
    float step = kSteerRate * dt;
    steerAngle += clampf(target - steerAngle, -step, step);
    steerAngle = clampf(steerAngle, -0.6f, 0.6f);

    if (std::fabs(speed) < 0.45f && input.brake > 0.5f && input.throttle < 0.1f) {
        brakeHold += dt;
    } else {
        brakeHold = 0;
    }
    bool reversing = brakeHold > 0.28f;

    float force = 0;
    float speedBefore = speed;
    if (input.brake > 0.05f && !reversing) {
        force -= input.brake * kBrake * (speed >= 0 ? 1.0f : -1.0f);
    } else if (reversing) {
        force -= input.brake * kReverseAccel;
    } else if (input.throttle > 0.0f && speed > -0.4f) {
        float ratio = clampf(speed / kMaxSpeed, 0.0f, 1.0f);
        force += input.throttle * kAccel * (1.0f - ratio * ratio);
    }
    force -= kDrag * speed * std::fabs(speed);
    if (std::fabs(speed) > 0.05f) {
        force -= kRolling * (speed > 0 ? 1.0f : -1.0f);
    }
    // 芝は加速が鈍く、速度も伸びない。横には滑らない。
    if (!onAsphalt) {
        if (force > 0) {
            force *= 0.55f;
        }
        if (std::fabs(speed) > 0.2f) {
            force -= 3.2f * (speed > 0 ? 1.0f : -1.0f);
        }
    }

    speed += force * dt;
    if (!reversing && input.brake > 0.05f) {
        if (speedBefore >= 0 && speed < 0) speed = 0;
        if (speedBefore <= 0 && speed > 0) speed = 0;
    }
    speed = clampf(speed, -kMaxReverse, kMaxSpeed);
    wheelSpin += speed / kWheelRadius * dt;

    stepGrip(dt);

    float preferS = ground.found ? ground.s : -1.0f;
    ground = track.query(x, z, y, preferS);
    if (!ground.found) {
        return;
    }

    const float wall = track.wallLateral();
    if (std::fabs(ground.lateral) > wall) {
        float sign = ground.lateral > 0 ? 1.0f : -1.0f;
        float overflow = std::fabs(ground.lateral) - wall;
        x -= ground.rightX * sign * overflow;
        z -= ground.rightZ * sign * overflow;
        // 壁の外へ向いている分だけ速度を落とす。コースに沿っているときは壁を擦るだけ。
        float facingOut = sign * (std::sin(yaw) * ground.rightX + std::cos(yaw) * ground.rightZ);
        if (facingOut > 0.15f) {
            float cap = 2.0f + 8.0f * (1.0f - facingOut);
            if (speed > cap) {
                speed = cap;
            }
        }
        ground.lateral = sign * wall;
    }

    float bankLimit = track.halfWidth + track.curbWidth;
    float bankLat = clampf(ground.lateral, -bankLimit, bankLimit);
    y = ground.centerY + bankLat * ground.cross;
    onAsphalt = std::fabs(ground.lateral) <= track.halfWidth + 0.15f;
}

void buildCarBody(CpuMesh& mesh) {
    const Vec3 red{0.72f, 0.08f, 0.09f};
    const Vec3 dark{0.06f, 0.06f, 0.07f};
    const Vec3 carbon{0.12f, 0.12f, 0.13f};
    mesh.addBox({-0.88f, 0.32f, -2.05f}, {0.88f, 0.62f, 2.05f}, red);
    mesh.addBox({-0.72f, 0.62f, -0.55f}, {0.72f, 1.05f, 1.05f}, red);
    mesh.addBox({-0.62f, 0.78f, -0.35f}, {0.62f, 1.02f, 0.9f}, dark);
    mesh.addBox({-0.7f, 0.32f, 1.7f}, {0.7f, 0.5f, 2.25f}, red);
    mesh.addBox({-0.95f, 0.26f, 1.95f}, {0.95f, 0.36f, 2.35f}, carbon);
    mesh.addBox({-0.86f, 0.85f, -2.2f}, {0.86f, 1.0f, -1.85f}, carbon);
    mesh.addBox({-0.15f, 0.95f, -2.05f}, {0.15f, 1.15f, -1.9f}, carbon);
}

void buildCarWheel(CpuMesh& mesh) {
    const int seg = 14;
    const float radius = 0.34f;
    const float halfW = 0.15f;
    const Vec3 tire{0.07f, 0.07f, 0.08f};
    const Vec3 rim{0.72f, 0.74f, 0.76f};
    for (int i = 0; i < seg; ++i) {
        float a0 = float(i) / float(seg) * 6.2831853f;
        float a1 = float(i + 1) / float(seg) * 6.2831853f;
        float c0 = std::cos(a0);
        float s0 = std::sin(a0);
        float c1 = std::cos(a1);
        float s1 = std::sin(a1);
        Vec3 normal = normalize(Vec3{0, c0 + c1, s0 + s1});
        Vec3 a{-halfW, c0 * radius, s0 * radius};
        Vec3 b{halfW, c0 * radius, s0 * radius};
        Vec3 c{halfW, c1 * radius, s1 * radius};
        Vec3 d{-halfW, c1 * radius, s1 * radius};
        mesh.addQuad(a, b, c, d, normal, tire);
        for (int side = -1; side <= 1; side += 2) {
            float x = float(side) * halfW;
            Vec3 nrm{float(side), 0, 0};
            Vec3 center{x, 0, 0};
            Vec3 p0{x, c0 * radius * 0.62f, s0 * radius * 0.62f};
            Vec3 p1{x, c1 * radius * 0.62f, s1 * radius * 0.62f};
            Vec3 o0{x, c0 * radius, s0 * radius};
            Vec3 o1{x, c1 * radius, s1 * radius};
            mesh.addQuad(o0, o1, p1, p0, nrm, tire);
            mesh.addTriangle(center, p0, p1, nrm, rim);
        }
    }
}

void buildCarShadow(CpuMesh& mesh) {
    const Vec3 ink{0.02f, 0.02f, 0.025f};
    mesh.addQuad({-1.05f, 0, -2.1f}, {1.05f, 0, -2.1f}, {1.05f, 0, 2.15f}, {-1.05f, 0, 2.15f}, {0, 1, 0}, ink);
}

Mat4 carModelMatrix(const Car& car, float pitch, float roll) {
    Vec3 forward = normalize(Vec3{
        std::sin(car.yaw) * std::cos(pitch),
        std::sin(pitch),
        std::cos(car.yaw) * std::cos(pitch),
    });
    Vec3 right0{std::cos(car.yaw), 0, -std::sin(car.yaw)};
    Vec3 up0 = normalize(cross(forward, right0));
    Vec3 right = normalize(right0 * std::cos(roll) + up0 * std::sin(roll));
    Vec3 up = normalize(cross(forward, right));
    return basis({car.x, car.y, car.z}, right, up, forward);
}
