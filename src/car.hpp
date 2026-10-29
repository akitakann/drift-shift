#pragma once

#include "meshbuild.hpp"
#include "track.hpp"

struct DriveInput {
    float throttle = 0;  // 0..1
    float brake = 0;     // 0..1  停止中は後退
    float steer = 0;     // -1..1  左が負、右が正
    float handbrake = 0; // 0..1  長押しでドリフト、離すとチャージを解放
    float boostHeading = 0; // Optical camera yaw captured once on a charged release; radians.
    bool hasBoostHeading = false; // Replays retain the captured direction rather than sampling their camera.
};

enum class DriftModel { Inertia, LineControl };
enum class DriftPhase { None, Entry, Held };

struct FeelSettings;

struct Car {
    DriftModel driftModel = DriftModel::Inertia; // 設定。resetでも選択を保つ
    float x = 0;
    float y = 0;
    float z = 0;
    float yaw = 0;
    float travelYaw = 0;    // 速度の向き。後退時は前進側の向きを保持
    float yawRate = 0;      // 車体の旋回速度（rad/s）
    float slipAngle = 0;    // 車体 yaw - travelYaw。右ドリフトが正
    float driftIntensity = 0; // 音・煙・HUD 用の実際の横滑り量（0..1）
    float impact = 0;       // このステップで壁が吸収した外向き速度（m/s）
    float speed = 0;        // 進行方向の速度 m/s。正が前進
    float throttleLevel = 0; // なめらかな駆動入力。エンジン音にも同じ値を使う
    float recoveryTime = 0.8f; // 衝突・失効時の横滑りを90%戻す時間の目安。明示解除は別の短い減衰
    float driftCharge = 0;   // 路面上で実際に滑ったチャージ時間（秒）
    int driftChargeLevel = 0; // 0..3
    float boostRemaining = 0;
    float boostSpeedBonus = 0; // speedに含まれる一時的な上乗せ。通常の駆動速度とは分ける
    float itemKickTime = -1;   // Elapsed time of an item push; negative when idle.
    float itemKickBonus = 0;   // Push speed currently included in speed, m/s.
    float itemKickPeak = 30.0f / 3.6f; // Tuning: peak push of one item, m/s. reset preserves it.
    int boostLevel = 0;       // ブースト継続中の段階（0..3）
    unsigned boostSerial = 0; // 有効な解放ごとに一度だけ増える演出用番号
    float boostSpeedScale = 1; // 調整値。resetで保持
    float boostDurationScale = 1;
    float chargeRateScale = 1;
    float driftSteerRate = 1.8f; // Full-input drift line turn rate, rad/s. reset preserves tuning.
    float driftSteerResponse = 0.12f; // B-model time to remove 90% of line-turn error.
    float topSpeedKmh = 200; // Normal component only; earned boosts may exceed this cap.
    float accelerationTime = 1.0f; // Full-throttle zero-to-cap time, seconds, plus input smoothing.
    float boostSpeedAttack = 0.10f;
    float boostSpeedHold = 0.20f;
    float boostSpeedReturn = 0.80f;
    float boostTotalDuration = 0; // 有効なブースト開始時に確定する時間
    float boostAttackDuration = 0, boostHoldDuration = 0, boostReturnDuration = 0;
    bool driftHeld = false;  // ドリフトボタンの現在の入力
    float driftEntrySpeed = 0; // 進入時の通常速度。長押し時の緩い減速の基準（m/s）
    float steerAngle = 0;   // 前輪の切れ角（ラジアン、右が正）
    float wheelSpin = 0;
    float brakeHold = 0;    // 止まってから後退に入るまでの待ち時間
    bool onAsphalt = true;
    TrackPose ground;
    bool grounded = true;
    float verticalSpeed = 0; // World-space vertical momentum, m/s.
    float groundHeight = 0; // Selected road/grass floor below the vehicle, including banking.
    float airTime = 0;
    float terrainPitch = 0, terrainRoll = 0; // Contact-plane attitude, radians.
    float landingImpact = 0; // Relative downward speed absorbed by the latest landing, m/s.
    unsigned landingSerial = 0;
    float gravity = 9.81f;
    float suspensionTravel = .20f; // Tyres keep contact through this much body extension, m.
    float maxLaunchSpeed = 4.5f; // Bound upward momentum inherited from raw terrain, m/s.
    float airSteerControl = .35f; // Fraction of deliberate line steering retained in flight.

    // ドリフトは明示的に始める。グリップ時には元の自転車モデルを使用する。
    float driftBlend = 0;
    float driftTravelRate = 0; // 実際の進路の旋回速度。解除時にも連続して引き継ぐ
    float driftAge = 0;       // 入りからの経過時間。開始時の回頭に使用
    float driftKick = 0;      // Entry中の車体相対回頭速度。Heldでは再始動しない
    float driftDirection = 0;
    bool drifting = false;
    bool handbrakeHeld = false;
    float handbrakeArm = 0;
    bool driftHoldActive = false;
    bool driftHoldBlocked = false;
    DriftPhase driftPhase = DriftPhase::None;
    float driftAngleCommand = 0; // 維持する横滑り角の大きさ（rad）。Held中45..60度
    bool releaseEaseActive = false;
    float releaseEaseSlip = 0;    // 明示解除の瞬間の横滑り角。符号も保持
    float releaseEaseElapsed = 0; // 進路を鼻先へ寄せる短いイージングの経過時間
    bool releaseUsesCameraHeading = false;
    float releaseHeading = 0; // A captured launch target; normal steering can still bend it.
    float releaseTravelOffset = 0; // Incoming travel yaw relative to the captured target.
    float releaseDirectionTime = 0.25f;

    void reset(const Track& track);
    void applyTuning(const FeelSettings& settings);
    void rescaleWorld(float ratio, const Track& track);
    TrackPose querySupport(const Track& track, float worldX, float worldZ) const;
    float boostEnvelopeAmount() const;
    // Item pickup: a short push on top of the drive speed. A new pickup restarts
    // it rather than stacking; braking, a barrier or leaving the asphalt ends it.
    void addKick();
    void update(float dt, const DriveInput& input, const Track& track);

    void stepGrip(float dt);
    void stepDrift(float dt, const DriveInput& input);

private:
    bool supportReady_ = false;
    float landingSpeedExcess_ = 0;
    float airTurnRate_ = 0;
    void updateSupport(float dt, const Track& track);
    void updateAirborne(float dt, const DriveInput& input, const Track& track);
};

void buildCarBody(CpuMesh& mesh);
void buildCarWheel(CpuMesh& mesh);
void buildCarShadow(CpuMesh& mesh);
Mat4 carModelMatrix(const Car& car, float pitch, float roll);
