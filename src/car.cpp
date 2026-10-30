#include "car.hpp"
#include "tuning.hpp"

#include <cmath>

namespace {

constexpr float kMaxReverse = 9.0f;
constexpr float kBrake = 30.0f;
constexpr float kReverseAccel = 8.0f;
constexpr float kDrag = 0.00045f;
constexpr float kRolling = 0.9f;
constexpr float kGrassCruiseSpeed = 80.0f/3.6f;
constexpr float kGrassDeceleration = 13.0f;
constexpr float kGrassAcceleration = 8.0f;
constexpr float kWheelbase = 3.05f;
constexpr float kSteerRate = 2.6f;
constexpr float kWheelRadius = 0.34f;
constexpr float kChargeThresholds[] = {0.0f, 0.65f, 1.30f, 2.20f};
constexpr float kBoostPeaks[] = {0.0f, 10.0f, 14.0f, 18.0f};
constexpr float kMinChargeAngle = 8.0f * 3.14159265f / 180.0f;
constexpr float kMinDriftSpeed = 11.0f;
constexpr float kDriftMinAngle = 45.0f * 3.14159265f / 180.0f;
constexpr float kDriftMaxAngle = 60.0f * 3.14159265f / 180.0f;
constexpr float kEntryTargetAngle = 46.0f * 3.14159265f / 180.0f;
constexpr float kAngleResponse = 10.0f;
constexpr float kAngleCommandRate = 45.0f * 3.14159265f / 180.0f;
constexpr float kHeldSlowdown = 1.0f;
constexpr float kHeldSpeedRetention = 0.90f;
constexpr float kReleaseEaseDuration = 0.15f;
constexpr float kKickAttack = 0.06f, kKickHold = 0.20f, kKickReturn = 0.60f; // Item push envelope, s.
constexpr float kWallKeep = 0.55f;   // Share of the pre-impact speed a wall glance keeps.
constexpr float kWallAlign = 14.0f;  // Rate at which a nose-in body swings along the wall, 1/s.
constexpr float kWallMaxAngle = 25.0f * 3.14159265f / 180.0f; // Sharpest nose-in pose kept on contact.

int chargeLevelOf(float charge) {
    return charge + 0.00001f >= kChargeThresholds[3] ? 3 :
           (charge + 0.00001f >= kChargeThresholds[2] ? 2 :
            (charge + 0.00001f >= kChargeThresholds[1] ? 1 : 0));
}

float maxSteerAt(float speed) {
    return 0.46f / (1.0f + std::fabs(speed) * 0.085f);
}

float gripYawRate(float speed, float steerAngle) {
    float rate = 0;
    if (std::fabs(speed) > 0.05f) rate = (speed / kWheelbase) * std::tan(steerAngle);
    const float slow = clampf(1.0f - std::fabs(speed) / 8.0f, 0.0f, 1.0f);
    return rate + steerAngle * 0.9f * slow;
}

float smoothBoost(float t) {
    t=clampf(t,0,1);
    return t*t*t*(t*(t*6-15)+10);
}

struct BoostProfile {
    float attack, hold, returning;
    float total() const { return attack+hold+returning; }
    float amount(float elapsed) const {
        if (elapsed<attack) return smoothBoost(elapsed/attack);
        if (elapsed<attack+hold) return 1;
        return 1-smoothBoost((elapsed-attack-hold)/returning);
    }
    float retime(float elapsed,const BoostProfile& after) const {
        if (elapsed<attack) return clampf(elapsed/attack,0,1)*after.attack;
        if (hold>0 && elapsed<attack+hold)
            return after.attack+clampf((elapsed-attack)/hold,0,1)*after.hold;
        return after.attack+after.hold+clampf((elapsed-attack-hold)/returning,0,1)*after.returning;
    }
};

BoostProfile configuredBoost(float attack,float hold,float returning,float scale) {
    return {attack*scale,hold*scale,returning*scale};
}

BoostProfile currentBoost(const Car& car) {
    BoostProfile profile=configuredBoost(car.boostSpeedAttack,car.boostSpeedHold,car.boostSpeedReturn,car.boostDurationScale);
    if (car.boostAttackDuration>0 && car.boostHoldDuration>=0 && car.boostReturnDuration>0)
        profile={car.boostAttackDuration,car.boostHoldDuration,car.boostReturnDuration};
    // Public state can also seed a valid in-flight reward for replay/test tools.
    // Honour its total while keeping the same stage proportions if no snapshot exists.
    if (car.boostTotalDuration>0 && std::fabs(profile.total()-car.boostTotalDuration)>.00001f) {
        const float scale=car.boostTotalDuration/profile.total();
        profile=configuredBoost(profile.attack,profile.hold,profile.returning,scale);
    }
    return profile;
}

void captureBoost(Car& car,const BoostProfile& profile,float elapsed=0) {
    car.boostAttackDuration=profile.attack;
    car.boostHoldDuration=profile.hold;
    car.boostReturnDuration=profile.returning;
    car.boostTotalDuration=profile.total();
    car.boostRemaining=std::fmax(car.boostTotalDuration-elapsed,0.0f);
}

void clearBoostClock(Car& car) {
    car.boostRemaining=car.boostTotalDuration=0;
    car.boostAttackDuration=car.boostHoldDuration=car.boostReturnDuration=0;
    car.boostLevel=0;
}

float surfaceHeight(const Track& track, const TrackPose& pose) {
    const float bankLimit=track.halfWidth+track.curbWidth;
    return pose.centerY+clampf(pose.lateral,-bankLimit,bankLimit)*pose.cross;
}

TrackPose supportQuery(const Track& track,float x,float z,float hint,float preferS) {
    if (!std::isfinite(x) || !std::isfinite(z) || !std::isfinite(hint)) return {};
    const TrackPose fallback=track.query(x,z,hint,preferS);
    if (!fallback.found || track.practiceArea) return fallback;
    // Track::query's height-weighted closest point is useful for route selection,
    // but biases uphill wheel probes toward a lower segment behind them. Within
    // the already selected route, use the actual XZ projection for contact.
    const float route=preferS>=0?preferS:fallback.s;
    TrackPose best;
    float bestDistance=1.0e30f;
    for (size_t i=0;i<track.samples.size()-(track.closed?0:1);++i) {
        const auto& a=track.samples[i];
        const auto& b=track.samples[(i+1)%track.samples.size()];
        const float abx=b.x-a.x,abz=b.z-a.z;
        const float len2=abx*abx+abz*abz;
        if (len2<.0001f) continue;
        const float len=std::sqrt(len2);
        const float t=clampf(((x-a.x)*abx+(z-a.z)*abz)/len2,0,1);
        const float along=a.s+len*t;
        float gap=std::fabs(along-route);
        if (track.closed && gap>track.length*.5f) gap=std::fabs(track.length-gap);
        if (gap>16) continue;
        const float dx=x-(a.x+abx*t),dz=z-(a.z+abz*t);
        const float distance=dx*dx+dz*dz;
        if (distance>=bestDistance) continue;
        bestDistance=distance;
        best.found=true;
        best.s=track.closed&&along>=track.length?along-track.length:along;
        best.rightX=abz/len;best.rightZ=-abx/len;
        best.lateral=dx*best.rightX+dz*best.rightZ;
        best.centerY=a.y+(b.y-a.y)*t;
        best.cross=a.cross+(b.cross-a.cross)*t;
        best.yaw=std::atan2(abx,abz);
        best.xzDist=std::sqrt(distance);
        best.corner=t<.5f?a.corner:b.corner;
        best.nameId=t<.5f?a.nameId:b.nameId;
    }
    if (!best.found || (best.centerY==fallback.centerY && best.cross==fallback.cross)) return fallback;
    return best;
}

float probeHeight(const Track& track, float x, float z, float heightHint, float preferS) {
    const auto pose=supportQuery(track,x,z,heightHint,preferS);
    return pose.found?surfaceHeight(track,pose):heightHint;
}

}  // namespace

void Car::reset(const Track& track) {
    x = track.spawnX;
    y = track.spawnY;
    z = track.spawnZ;
    yaw = track.spawnYaw;
    travelYaw = yaw;
    yawRate = 0;
    slipAngle = 0;
    driftIntensity = 0;
    impact = 0;
    driftBlend = 0;
    driftTravelRate = 0;
    driftAge = 0;
    driftKick = 0;
    driftDirection = 0;
    drifting = false;
    handbrakeHeld = false;
    handbrakeArm = 0;
    driftHeld = false;
    driftHoldActive = false;
    driftHoldBlocked = false;
    driftPhase = DriftPhase::None;
    driftAngleCommand = 0;
    releaseEaseActive = false;
    releaseEaseSlip = 0;
    releaseEaseElapsed = 0;
    releaseUsesCameraHeading = false;
    releaseHeading = releaseTravelOffset = 0;
    driftEntrySpeed = 0;
    driftCharge = 0;
    driftChargeLevel = 0;
    boostRemaining = 0;
    boostLevel = 0;
    boostSerial = 0;
    boostSpeedBonus = 0;
    itemKickTime = -1;
    itemKickBonus = 0;
    boostTotalDuration = 0;
    boostAttackDuration = boostHoldDuration = boostReturnDuration = 0;
    speed = 0;
    throttleLevel = 0;
    steerAngle = 0;
    wheelSpin = 0;
    brakeHold = 0;
    onAsphalt = true;
    ground = track.query(x, z, y);
    grounded = true;
    verticalSpeed = airTime = landingImpact = 0;
    terrainPitch = terrainRoll = 0;
    landingSerial = 0;
    groundHeight = ground.found?surfaceHeight(track,ground):y;
    y = groundHeight;
    supportReady_ = false;
    landingSpeedExcess_ = 0;
    airTurnRate_ = 0;
}

void Car::stepGrip(float dt) {
    driftTravelRate = 0;
    yawRate = gripYawRate(speed, steerAngle) * turnScale;
    yaw = wrapPi(yaw + yawRate * dt);
    travelYaw = yaw;
    slipAngle = 0;
    driftIntensity = 0;

    x += std::sin(yaw) * speed * moveScale * dt;
    z += std::cos(yaw) * speed * moveScale * dt;
}

void Car::stepDrift(float dt, const DriveInput& input) {
    const float steer = clampf(input.steer, -1.0f, 1.0f);
    const bool canSlide = speed > kMinDriftSpeed && onAsphalt;
    if (releaseEaseActive) {
        if (releaseUsesCameraHeading) {
            const float previousYaw = yaw;
            const float steeringTurn = gripYawRate(speed, steerAngle) * turnScale * dt;
            releaseHeading = wrapPi(releaseHeading + steeringTurn);
            releaseEaseElapsed = std::fmin(releaseEaseElapsed + dt, releaseDirectionTime);
            const float remaining = std::fmax(1.0f - releaseEaseElapsed / releaseDirectionTime, 0.0f);
            driftBlend = remaining * remaining * remaining;
            // Both the velocity and body keep their incoming pose, then converge
            // to the captured view direction. The camera is never sampled here.
            travelYaw = wrapPi(releaseHeading + releaseTravelOffset * driftBlend);
            yaw = wrapPi(releaseHeading + releaseEaseSlip * driftBlend);
            yawRate = wrapPi(yaw - previousYaw) / dt;
            slipAngle = wrapPi(yaw - travelYaw);
            driftIntensity = slipFeedback(speed, slipAngle, onAsphalt);
            x += std::sin(travelYaw) * speed * moveScale * dt;
            z += std::cos(travelYaw) * speed * moveScale * dt;
            if (remaining < 0.00001f) {
                releaseEaseActive = false;
                releaseEaseSlip = releaseEaseElapsed = releaseTravelOffset = 0;
                drifting = false;
                driftBlend = 0;
                slipAngle = 0;
                travelYaw = yaw;
                driftIntensity = 0;
            }
            return;
        }
        // 車体は通常の操舵で進める。古い慣性だけを短く減衰させ、車体を巻き戻さない。
        yawRate = gripYawRate(speed, steerAngle) * turnScale;
        yaw = wrapPi(yaw + yawRate * dt);
        releaseEaseElapsed = std::fmin(releaseEaseElapsed + dt, kReleaseEaseDuration);
        const float remaining = std::fmax(1.0f - releaseEaseElapsed / kReleaseEaseDuration, 0.0f);
        driftBlend = remaining * remaining * remaining;
        slipAngle = releaseEaseSlip * driftBlend;
        travelYaw = wrapPi(yaw - slipAngle);
        driftIntensity = slipFeedback(speed, slipAngle, onAsphalt);
        x += std::sin(travelYaw) * speed * moveScale * dt;
        z += std::cos(travelYaw) * speed * moveScale * dt;
        if (remaining < 0.00001f) {
            releaseEaseActive = false;
            releaseEaseSlip = releaseEaseElapsed = 0;
            drifting = false;
            driftBlend = 0;
            slipAngle = 0;
            travelYaw = yaw;
            driftIntensity = 0;
        }
        return;
    }
    if (!drifting) {
        stepGrip(dt);
        return;
    }
    const bool powered = driftHeld && driftHoldActive && canSlide && input.brake <= 0.05f;
    const float recover = clampf(recoveryTime, 0.3f, 1.5f);
    const float recoveryRate = 2.302585f / recover;
    const float blendRate = powered ? 10.0f : recoveryRate;
    driftBlend += ((powered ? 1.0f : 0.0f) - driftBlend) * (1.0f - std::exp(-blendRate * dt));

    // 左右入力は両モデルで滑るラインそのものを曲げる。Aは応答だけに
    // 慣性を残し、車体の横滑り角はラインと独立した相対運動で制御する。
    const float previousTravelRate = driftTravelRate;
    float travelTurn;
    float targetYawRate = 0, targetRelativeRate = 0;
    const bool entering = powered && driftPhase == DriftPhase::Entry;
    if (powered) {
        const float inside = steer * driftDirection;
        const float responseTime = driftSteerResponse * (driftModel == DriftModel::Inertia ? 1.6f : 1.0f);
        const float lineResponse = 1 - std::exp(-2.302585093f * dt / responseTime);
        travelTurn = previousTravelRate + (steer * driftSteerRate * turnScale - previousTravelRate) * lineResponse;
        travelTurn = clampf(travelTurn, -3.0f, 3.0f);
        const float signedSlip = slipAngle * driftDirection;
        float relativeTarget;
        if (entering) {
            // 導入中だけは0→45度を作る。途中で逆ハンを入れても正面のまま
            // 維持状態へ入らず、ボタンを離せば現在の角度から解除できる。
            driftAngleCommand = kEntryTargetAngle;
            relativeTarget = clampf((driftAngleCommand - signedSlip) * kAngleResponse, -4.8f, 4.8f);
            driftKick = driftDirection * relativeTarget;
        } else {
            driftKick = 0;
            driftAngleCommand = clampf(driftAngleCommand + inside * kAngleCommandRate * dt,
                                       kDriftMinAngle, kDriftMaxAngle);
            relativeTarget = clampf((driftAngleCommand - signedSlip) * kAngleResponse, -1.8f, 1.8f);
        }
        targetRelativeRate = relativeTarget;
        driftAge = std::fmin(driftAge + dt, 5.0f);
    } else {
        driftKick = 0;
        // 衝突・失効後は速度が車の向きへ追いつく。車体だけを古い進路へ戻さない。
        // 維持中の旋回は短く引き継ぎ、入力があれば通常の操舵へ移る。
        const float gripTurn = (speed / kWheelbase) * std::tan(steerAngle) * turnScale;
        driftTravelRate += (gripTurn - driftTravelRate) * (1.0f - std::exp(-15.0f * dt));
        const float catchTurn = slipAngle * recoveryRate;
        travelTurn = driftTravelRate + 0.85f * catchTurn;
        targetYawRate = driftTravelRate - 0.15f * catchTurn;
    }
    if (powered) {
        const float signedSlip = slipAngle * driftDirection;
        // Filter only the slip correction. Filtering/clamping world yaw first
        // would fight a quick line reversal and unwind the player's drift pose.
        const float previousRelativeRate = (yawRate - previousTravelRate) * driftDirection;
        float relativeRate = previousRelativeRate + (targetRelativeRate - previousRelativeRate) *
                             (1 - std::exp(-24.0f * dt));
        const float relativeLimit = entering ? 4.8f : 1.8f;
        relativeRate = clampf(relativeRate, -relativeLimit, relativeLimit);
        const float outwardLimit = std::fmax(kDriftMaxAngle - signedSlip, 0.0f) * 6.0f;
        relativeRate = std::fmin(relativeRate, outwardLimit);
        if (driftPhase == DriftPhase::Held) {
            const float inwardLimit = std::fmax(signedSlip - kDriftMinAngle, 0.0f) * 6.0f;
            relativeRate = std::fmax(relativeRate, -inwardLimit);
        }
        yawRate = travelTurn + driftDirection * relativeRate;
        driftTravelRate = travelTurn;
    } else {
        yawRate += (targetYawRate - yawRate) * (1 - std::exp(-28.0f * dt));
        yawRate = clampf(yawRate, -3.0f, 3.0f);
        travelTurn = clampf(travelTurn, -3.0f, 3.0f);
    }
    yaw = wrapPi(yaw + yawRate * dt);
    travelYaw = wrapPi(travelYaw + travelTurn * dt);
    slipAngle = wrapPi(yaw - travelYaw);
    if (entering && slipAngle * driftDirection >= kDriftMinAngle &&
        slipAngle * driftDirection <= kDriftMaxAngle) {
        driftPhase = DriftPhase::Held;
        driftAngleCommand = clampf(slipAngle * driftDirection, kDriftMinAngle, kDriftMaxAngle);
        driftKick = 0;
    }
    driftIntensity = slipFeedback(speed, slipAngle, onAsphalt);
    x += std::sin(travelYaw) * speed * moveScale * dt;
    z += std::cos(travelYaw) * speed * moveScale * dt;

    if ((!powered && std::fabs(slipAngle) < 0.01f && driftBlend < 0.035f) || speed < 3.0f) {
        drifting = false;
        driftBlend = 0;
        driftTravelRate = 0;
        driftAge = 0;
        driftKick = 0;
        driftDirection = 0;
        driftPhase = DriftPhase::None;
        driftAngleCommand = 0;
    }
}

void Car::updateSupport(float dt, const Track& track) {
    const bool wasGrounded=grounded;
    const float oldY=y;
    const float hint=groundHeight;
    const float preferS=ground.found?ground.s:-1;
    // The selected floor, rather than the flying body, is the height reference.
    // A jump below Suzuka's overpass must not attract the car to its upper deck.
    ground=supportQuery(track,x,z,hint,preferS);
    if (!ground.found) {
        grounded=false;
        y=oldY+verticalSpeed*dt-.5f*gravity*dt*dt;
        verticalSpeed-=gravity*dt;
        airTime+=dt;
        return;
    }
    const float wall=track.wallLateral();
    if (std::fabs(ground.lateral)>wall) {
        const float sign=ground.lateral>0?1.0f:-1.0f;
        const float overflow=std::fabs(ground.lateral)-wall;
        x-=ground.rightX*sign*overflow;
        z-=ground.rightZ*sign*overflow;
        const float nx=ground.rightX*sign,nz=ground.rightZ*sign;
        float vx=std::sin(travelYaw)*speed,vz=std::cos(travelYaw)*speed;
        const float outward=vx*nx+vz*nz;
        if (outward>0) {
            const float beforeImpactSpeed=std::fabs(speed);
            impact=outward;
            vx-=nx*outward;vz-=nz*outward;
            const float direction=speed>=0?1.0f:-1.0f;
            // Arcade glance: forward impacts deflect along the wall and keep
            // part of their pace instead of stopping dead against it.
            float tangentX=nz,tangentZ=-nx;
            const float along=vx*tangentX+vz*tangentZ;
            const float bodyAlong=std::sin(yaw)*tangentX+std::cos(yaw)*tangentZ;
            if (along<0 || (std::fabs(along)<.5f && bodyAlong<0)) {tangentX=-tangentX;tangentZ=-tangentZ;}
            if (direction>0 && beforeImpactSpeed>kMinDriftSpeed) {
                const float kept=std::fmax(std::fabs(along),beforeImpactSpeed*kWallKeep);
                vx=tangentX*kept;vz=tangentZ*kept;
            }
            speed=std::sqrt(vx*vx+vz*vz)*direction;
            if (direction>0 && grounded) {
                // Swing the body to follow the wall so it cannot stay pinned
                // nose-in, losing speed to the wall on every step.
                const float wallYaw=std::atan2(tangentX,tangentZ);
                const float into=std::sin(yaw)*nx+std::cos(yaw)*nz;
                if (into>0) {
                    float offset=wrapPi(yaw-wallYaw);
                    offset=clampf(offset,-kWallMaxAngle,kWallMaxAngle);
                    yaw=wrapPi(wallYaw+offset*std::exp(-kWallAlign*dt));
                }
            }
            if (boostSpeedBonus>0 && beforeImpactSpeed>0)
                boostSpeedBonus*=std::fabs(speed)/beforeImpactSpeed;
            if (std::fabs(speed)>.01f) {
                travelYaw=std::atan2(vx*direction,vz*direction);
                slipAngle=wrapPi(yaw-travelYaw);
            }
            if (impact>2) driftBlend*=std::exp(-impact*.12f);
        }
        ground=supportQuery(track,x,z,hint,ground.s);
    }
    groundHeight=surfaceHeight(track,ground);
    onAsphalt=std::fabs(ground.lateral)<=track.halfWidth+.15f;
    // Average the support slope across the wheelbase. A short kerb/step must
    // compress/extend suspension rather than supply a huge launch impulse.
    constexpr float probe=kWheelbase*.5f;
    const float dx=std::sin(travelYaw)*probe,dz=std::cos(travelYaw)*probe;
    const float ahead=probeHeight(track,x+dx,z+dz,groundHeight,ground.s);
    const float behind=probeHeight(track,x-dx,z-dz,groundHeight,ground.s);
    const float supportVelocity=(ahead-behind)/(2*probe)*speed*moveScale;
    const float contactVelocity=std::fmin(supportVelocity,maxLaunchSpeed);
    const float freeY=oldY+verticalSpeed*dt-.5f*gravity*dt*dt;
    const float freeVelocity=verticalSpeed-gravity*dt;
    // The body may extend above the floor while the tyres still reach it.
    // Keep its continuous vertical motion within that suspension travel so a
    // small downstep does not disable steering/charge for a one-frame flight.
    constexpr float contactSlop=.0005f;
    if (wasGrounded && (!supportReady_ || groundHeight+suspensionTravel>=freeY-contactSlop)) {
        const bool extending=supportReady_ && freeY>groundHeight+contactSlop;
        y=extending?freeY:groundHeight;
        verticalSpeed=extending?std::fmin(freeVelocity,maxLaunchSpeed):contactVelocity;
        grounded=true;
        airTime=0;
    } else if (!wasGrounded && groundHeight>=freeY-contactSlop) {
        y=groundHeight;
        landingImpact=std::fmax(supportVelocity-freeVelocity,0.0f);
        ++landingSerial;
        verticalSpeed=contactVelocity;
        grounded=true;
        airTime=0;
        landingSpeedExcess_=std::fmax(speed-boostSpeedBonus-topSpeedKmh/3.6f,0.0f);
    } else {
        y=freeY;
        verticalSpeed=freeVelocity;
        grounded=false;
        airTime+=dt;
    }
    supportReady_=true;
    if (grounded) {
        airTurnRate_=0;
        const float fx=std::sin(yaw),fz=std::cos(yaw);
        const float rx=std::cos(yaw),rz=-std::sin(yaw);
        const float front=probeHeight(track,x+fx*1.42f,z+fz*1.42f,groundHeight,ground.s);
        const float rear=probeHeight(track,x-fx*1.38f,z-fz*1.38f,groundHeight,ground.s);
        const float right=probeHeight(track,x+rx*.82f,z+rz*.82f,groundHeight,ground.s);
        const float left=probeHeight(track,x-rx*.82f,z-rz*.82f,groundHeight,ground.s);
        terrainPitch=std::atan2(front-rear,2.8f);
        terrainRoll=std::atan2(right-left,1.64f);
    }
}

void Car::updateAirborne(float dt, const DriveInput& input, const Track& track) {
    const bool previousHold=handbrakeHeld;
    driftHeld=handbrakeHeld=input.handbrake>.2f;
    auto cancelAirReward=[&](bool blockHold=true) {
        driftCharge=0;driftChargeLevel=0;
        clearBoostClock(*this);boostSpeedBonus=0;
        releaseEaseActive=false;
        releaseEaseSlip=releaseEaseElapsed=releaseTravelOffset=releaseHeading=0;
        releaseUsesCameraHeading=false;
        driftHoldActive=false;driftHoldBlocked=blockHold;
        driftPhase=DriftPhase::None;driftAngleCommand=0;
        driftDirection=driftKick=driftAge=0;
    };
    if (driftHeld&&!previousHold) cancelAirReward(false);
    if (input.brake>.05f) cancelAirReward();
    if (!driftHeld && previousHold) {
        // Releasing in air consumes the edge without inventing tyre traction.
        driftCharge=0;driftChargeLevel=0;driftHoldActive=false;
        releaseEaseActive=false;
        releaseEaseSlip=releaseEaseElapsed=releaseTravelOffset=releaseHeading=0;
        releaseUsesCameraHeading=false;
        driftPhase=DriftPhase::None;driftAngleCommand=0;
    }
    itemKickTime=-1;itemKickBonus=0; // In flight the push simply stays as momentum.
    const float steer=clampf(input.steer,-1,1);
    const float targetTurn=steer*driftSteerRate*turnScale*airSteerControl*(speed>=0?1.0f:-1.0f);
    airTurnRate_+=(targetTurn-airTurnRate_)*(1-std::exp(-8*dt));
    const float turn=airTurnRate_*dt;
    yaw=wrapPi(yaw+turn);
    travelYaw=wrapPi(travelYaw+turn);
    if (releaseEaseActive && releaseUsesCameraHeading) releaseHeading=wrapPi(releaseHeading+turn);
    yawRate=driftTravelRate=airTurnRate_;
    driftKick=0;
    const float rawThrottle=clampf(input.throttle,0,1);
    throttleLevel+=(rawThrottle-throttleLevel)*(1-std::exp(-(rawThrottle>throttleLevel?18.0f:10.0f)*dt));
    if (boostRemaining>0) {
        boostRemaining=std::fmax(boostRemaining-dt,0.0f);
        if (boostRemaining<.00001f) clearBoostClock(*this);
    }
    // Reward time continues, but ending a motor force cannot remove momentum
    // from a flying car. The excess joins the normal cap gradually on landing.
    boostSpeedBonus=boostLevel>0&&boostLevel<=3?
        std::fmin(kBoostPeaks[boostLevel]*boostSpeedScale*boostEnvelopeAmount(),std::fmax(speed,0.0f)):0;
    x+=std::sin(travelYaw)*speed*moveScale*dt;
    z+=std::cos(travelYaw)*speed*moveScale*dt;
    wheelSpin+=speed*moveScale/kWheelRadius*dt;
    updateSupport(dt,track);
    if (impact>.05f || (grounded&&(!onAsphalt || input.brake>.05f))) cancelAirReward();
    if (grounded)
        landingSpeedExcess_=std::fmax(landingSpeedExcess_,std::fmax(speed-boostSpeedBonus-topSpeedKmh/3.6f,0.0f));
    driftIntensity=grounded?slipFeedback(speed,slipAngle,onAsphalt):0;
}

void Car::rescaleWorld(float ratio, const Track& track) {
    if (!std::isfinite(ratio) || ratio<=0) return;
    const float preferS=ground.found?ground.s*ratio:-1;
    x*=ratio;y*=ratio;z*=ratio;
    groundHeight*=ratio;
    verticalSpeed*=ratio;
    landingImpact*=ratio;
    ground=supportQuery(track,x,z,groundHeight,preferS);
    if (ground.found) {
        groundHeight=surfaceHeight(track,ground);
        onAsphalt=std::fabs(ground.lateral)<=track.halfWidth+.15f;
        if (grounded) y=groundHeight;
    }
    // A paused geometry edit is not a crest or a landing impulse.
    if (grounded) supportReady_=false;
}

TrackPose Car::querySupport(const Track& track,float worldX,float worldZ) const {
    return supportQuery(track,worldX,worldZ,groundHeight,ground.found?ground.s:-1);
}

void Car::applyTuning(const FeelSettings& rawSettings) {
    const FeelSettings settings = sanitizeFeelSettings(rawSettings);
    const float maxSpeed = settings.topSpeedKmh / 3.6f;
    if (grounded && settings.topSpeedKmh!=topSpeedKmh) landingSpeedExcess_=0;
    const bool activeBoost = boostLevel > 0 && boostLevel <= 3 &&
                             std::isfinite(boostRemaining) && boostRemaining > 0 &&
                             std::isfinite(boostTotalDuration) && boostTotalDuration > 0 &&
                             std::isfinite(speed) && speed > 0;
    if (activeBoost) {
        // Re-time the existing reward at its current progress. Changing a setting
        // never starts another reward or changes its release event number.
        const BoostProfile before=currentBoost(*this);
        const BoostProfile after=configuredBoost(settings.boostSpeedAttack,settings.boostSpeedHold,
                                                  settings.boostSpeedReturn,settings.boostDurationScale);
        captureBoost(*this,after,before.retime(before.total()-boostRemaining,after));

        // Keep speed and its removable reward component in sync immediately,
        // including while paused and when strength changes from zero to nonzero.
        const float baseSpeed = std::fmin(std::fmax(speed - boostSpeedBonus, 0.0f), maxSpeed+landingSpeedExcess_);
        boostSpeedBonus = kBoostPeaks[boostLevel] * settings.boostSpeedScale * boostEnvelopeAmount();
        if (!grounded) boostSpeedBonus=std::fmin(boostSpeedBonus,std::fmax(speed,0.0f));
        if (grounded) speed = baseSpeed + boostSpeedBonus;
    } else if (grounded) {
        // A lower practice cap takes effect even while paused. Raising it never
        // creates speed: only a later accelerator input can reach the new cap.
        speed = std::fmin(speed, maxSpeed+landingSpeedExcess_);
    }
    driftIntensity = grounded?slipFeedback(speed, slipAngle, onAsphalt):0;
    gravity = settings.gravity;
    suspensionTravel = settings.suspensionTravel;
    maxLaunchSpeed = settings.maxLaunchSpeed;
    airSteerControl = settings.airSteerControl;
    recoveryTime = settings.recoveryTime;
    boostSpeedScale = settings.boostSpeedScale;
    boostDurationScale = settings.boostDurationScale;
    chargeRateScale = settings.chargeRateScale;
    driftSteerRate = settings.driftSteerRate;
    driftSteerResponse = settings.driftSteerResponse;
    topSpeedKmh = settings.topSpeedKmh;
    accelerationTime = settings.accelerationTime;
    // Preserve normalized progress when this transition is retimed in flight.
    const float nextReleaseTime = settings.releaseDirectionTime;
    if (releaseEaseActive && releaseUsesCameraHeading && releaseDirectionTime > 0)
        releaseEaseElapsed = releaseEaseElapsed / releaseDirectionTime * nextReleaseTime;
    releaseDirectionTime = nextReleaseTime;
    boostSpeedAttack = settings.boostSpeedAttack;
    boostSpeedHold = settings.boostSpeedHold;
    boostSpeedReturn = settings.boostSpeedReturn;
    itemKickPeak = settings.itemKickKmh / 3.6f;
    itemChargeSeconds = settings.itemChargeSeconds;
    moveScale = settings.moveSpeedScale;
    turnScale = settings.turnRateScale;
}

void Car::addKick() {
    if (driftHeld && driftHoldActive && itemChargeSeconds > 0) {
        driftCharge = std::fmin(driftCharge + itemChargeSeconds, kChargeThresholds[3]);
        driftChargeLevel = chargeLevelOf(driftCharge);
    }
    if (speed < 0 || !(itemKickPeak > 0)) return;
    itemKickTime = 0;
}

float Car::boostEnvelopeAmount() const {
    if (boostLevel<=0 || boostLevel>3 || !(boostRemaining>0)) return 0;
    const BoostProfile profile=currentBoost(*this);
    return clampf(profile.amount(profile.total()-boostRemaining),0,1);
}

void Car::update(float dt, const DriveInput& input, const Track& track) {
    if (!(dt > 0) || !std::isfinite(dt)) return;
    if (dt>.1f) {
        // The live loop supplies1/60s. Bound exceptional external catch-up calls
        // and subdivide them so a single update cannot tunnel an entire jump.
        const float span=std::fmin(dt,1.0f);
        const int steps=int(std::ceil(span/(1.0f/60.0f)));
        for (int i=0;i<steps;++i) update(span/steps,input,track);
        return;
    }
    gravity = finiteFeelValue(gravity,9.81f,6,22);
    suspensionTravel = finiteFeelValue(suspensionTravel,.20f,.05f,.45f);
    maxLaunchSpeed = finiteFeelValue(maxLaunchSpeed,4.5f,2,8);
    airSteerControl = finiteFeelValue(airSteerControl,.35f,0,.8f);
    boostSpeedScale = finiteFeelValue(boostSpeedScale, 1, 0, 2);
    boostDurationScale = finiteFeelValue(boostDurationScale, 1, 0.25f, 3);
    chargeRateScale = finiteFeelValue(chargeRateScale, 1, 0.25f, 3);
    itemChargeSeconds = finiteFeelValue(itemChargeSeconds, .3f, 0, 1);
    moveScale = finiteFeelValue(moveScale, 1, .5f, 2);
    turnScale = finiteFeelValue(turnScale, 1, .5f, 2);
    driftSteerRate = finiteFeelValue(driftSteerRate, 1.8f, 0.3f, 3);
    driftSteerResponse = finiteFeelValue(driftSteerResponse, .12f, .05f, .5f);
    topSpeedKmh = finiteFeelValue(topSpeedKmh, 200, 60, 200);
    releaseDirectionTime = finiteFeelValue(releaseDirectionTime, .25f, .10f, .60f);
    accelerationTime = finiteFeelValue(accelerationTime, 1, .2f, 3);
    boostSpeedAttack = finiteFeelValue(boostSpeedAttack, .1f, .03f, .3f);
    boostSpeedHold = finiteFeelValue(boostSpeedHold, .2f, 0, .6f);
    boostSpeedReturn = finiteFeelValue(boostSpeedReturn, .8f, .2f, 2);
    const float maxSpeed = topSpeedKmh / 3.6f;
    impact = 0;
    landingImpact = 0;
    if (!grounded) {
        updateAirborne(dt,input,track);
        return;
    }
    landingSpeedExcess_*=std::exp(-dt/.35f);
    if (landingSpeedExcess_<.001f) landingSpeedExcess_=0;
    const float steeringSpeed = speed;
    auto removeBoostSpeed = [&] {
        // 毎ステップ通常の駆動速度へ戻して計算する。ブーストの速度を加速や
        // 次のドリフトへ持ち越さず、減衰後の上乗せだけを後で足し直す。
        if (boostSpeedBonus > 0 && speed > 0) speed = std::fmax(speed - boostSpeedBonus, 0.0f);
        boostSpeedBonus = 0;
        if (itemKickBonus > 0 && speed > 0) speed = std::fmax(speed - itemKickBonus, 0.0f);
        itemKickBonus = 0;
    };
    removeBoostSpeed();
    auto cancelBoost = [&] {
        removeBoostSpeed();
        clearBoostClock(*this);
    };
    auto cancelReleaseEase = [&] {
        releaseEaseActive = false;
        releaseEaseSlip = releaseEaseElapsed = 0;
        releaseUsesCameraHeading = false;
        releaseHeading = releaseTravelOffset = 0;
    };
    auto clearHeldAngle = [&] {
        driftPhase = DriftPhase::None;
        driftAngleCommand = 0;
    };
    const bool previousHold = handbrakeHeld;
    driftHeld = input.handbrake > 0.2f;
    handbrakeHeld = driftHeld;
    if (driftHeld && !previousHold) {
        cancelBoost();
        cancelReleaseEase();
        clearHeldAngle();
        handbrakeArm = 0.18f;
        driftHoldBlocked = false;
    }
    if (!driftHeld) handbrakeArm = 0;

    auto cancelReward = [&](bool blockHold = true) {
        driftCharge = 0;
        driftChargeLevel = 0;
        cancelBoost();
        cancelReleaseEase();
        clearHeldAngle();
        driftHoldActive = false;
        driftAge = 0;
        driftKick = 0;
        driftDirection = 0;
        if (blockHold) driftHoldBlocked = true;
    };
    const bool invalidHold = input.brake > 0.05f || !onAsphalt || speed < 0;
    if (invalidHold) {
        cancelReward();
        handbrakeArm = 0;
    }

    // 開始時だけ車体に回頭のきっかけを与える。速度ベクトルは向け直さない。
    // ボタンを長押ししても、このキックは繰り返さない。
    if (!driftHoldActive && driftHeld && !driftHoldBlocked && speed > kMinDriftSpeed && onAsphalt &&
        input.brake <= 0.05f && std::fabs(input.steer) > 0.12f) {
        driftDirection = input.steer > 0 ? 1.0f : -1.0f;
        drifting = true;
        driftHoldActive = true;
        driftPhase = DriftPhase::Entry;
        driftAngleCommand = kEntryTargetAngle;
        driftEntrySpeed = speed;
        driftBlend = std::fmax(driftBlend, 0.15f);
        driftAge = 0;
        driftKick = driftDirection * 4.8f;
        driftCharge = 0;
        driftChargeLevel = 0;
        cancelBoost();
        cancelReleaseEase();
        handbrakeArm = 0;
    }
    handbrakeArm = clampf(handbrakeArm - dt, 0.0f, 0.18f);

    const bool manualRelease = previousHold && !driftHeld && driftHoldActive &&
                               speed > kMinDriftSpeed && onAsphalt && input.brake <= 0.05f;
    const int releaseLevel = manualRelease ? driftChargeLevel : 0;
    if (manualRelease) {
        // Charged launches converge smoothly toward the recorded optical ray.
        // Uncharged or direction-less releases retain the earlier nose recovery.
        releaseEaseActive = true;
        releaseEaseSlip = slipAngle;
        releaseEaseElapsed = 0;
        releaseUsesCameraHeading = releaseLevel > 0 && boostSpeedScale > 0 &&
                                   input.hasBoostHeading && std::isfinite(input.boostHeading);
        if (releaseUsesCameraHeading) {
            releaseHeading = std::remainder(input.boostHeading, 6.283185307f);
            releaseTravelOffset = wrapPi(travelYaw - releaseHeading);
            releaseEaseSlip = wrapPi(yaw - releaseHeading);
        }
        yawRate = 0;
        drifting = true;
        driftTravelRate = 0;
        driftAge = 0;
        driftKick = 0;
        driftDirection = 0;
        clearHeldAngle();
    }
    if (!driftHeld) {
        driftHoldActive = false;
        driftCharge = 0;
        driftChargeLevel = 0;
    }
    if (boostRemaining > 0) {
        boostRemaining = std::fmax(boostRemaining - dt, 0.0f);
        if (boostRemaining < 0.00001f) {
            clearBoostClock(*this);
        }
    }
    const float rawThrottle = clampf(input.throttle, 0.0f, 1.0f);
    const float throttleResponse = rawThrottle > throttleLevel ? 18.0f : 10.0f;
    throttleLevel += (rawThrottle - throttleLevel) * (1.0f - std::exp(-throttleResponse * dt));
    float target = clampf(input.steer, -1.0f, 1.0f) * maxSteerAt(steeringSpeed);
    float step = kSteerRate * dt;
    steerAngle += clampf(target - steerAngle, -step, step);
    steerAngle = clampf(steerAngle, -0.6f, 0.6f);

    if (speed < 0.45f && input.brake > 0.5f && input.throttle < 0.1f) {
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
    } else if (input.throttle > 0.0f && speed < -0.4f) {
        force += throttleLevel * kBrake;
    } else if (input.throttle > 0.0f) {
        // Reach a finite, low practice cap promptly instead of approaching a
        // distant equilibrium. Compensate rolling/aero resistance only while
        // driving forward; releasing W retains the existing coasting forces.
        const float resistance = kDrag * speed * std::fabs(speed) + (speed > 0.05f ? kRolling : 0.0f);
        force += throttleLevel * (maxSpeed / accelerationTime + resistance);
    }
    force -= kDrag * speed * std::fabs(speed);
    if (std::fabs(speed) > 0.05f) {
        force -= kRolling * (speed > 0 ? 1.0f : -1.0f);
    }
    // Only the green beyond the painted curb supplies grass resistance.
    // This branch is grounded-only; flying over it never changes momentum.
    const bool grassContact=ground.found && std::fabs(ground.lateral)>track.halfWidth+track.curbWidth;
    if (grassContact) {
        const float grassLimit=std::fmin(maxSpeed,kGrassCruiseSpeed);
        if (input.throttle<=0 && input.brake<=.05f && std::fabs(speed)>.2f)
            force-=3.2f*(speed>0?1.0f:-1.0f);
        if (speed>grassLimit) {
            // Bound each step's loss, rather than clamping entry speed to80.
            const float slowing=std::fmin(kGrassDeceleration,(speed-grassLimit)/dt);
            force=std::fmin(force,-slowing);
        } else if (speed>=0 && input.throttle>0 && input.brake<=.05f && !reversing) {
            const float room=std::fmax((grassLimit-speed)/dt,0.0f);
            force=std::fmin(force,std::fmin(kGrassAcceleration,room));
        }
    }

    // 有効な長押し中は1m/s²で少しだけ減速する。進入速度の約90%で止め、
    // 低速で時間切れにならないようにするが、失った速度を増やして戻すことはしない。
    if (driftHeld && driftHoldActive && onAsphalt && input.brake <= 0.05f) {
        const float minimum = std::fmin(driftEntrySpeed,
            std::fmax(driftEntrySpeed * kHeldSpeedRetention, kMinDriftSpeed + 0.05f));
        const float loss = std::fmax(speed - minimum, 0.0f);
        speed -= std::fmin(kHeldSlowdown * dt, loss);
    } else {
        speed += force * dt;
    }
    if (!reversing && input.brake > 0.05f) {
        if (speedBefore >= 0 && speed < 0) speed = 0;
        if (speedBefore <= 0 && speed > 0) speed = 0;
    }
    if (speedBefore < 0 && speed > 0 && input.throttle > 0.05f) speed = 0;
    speed = clampf(speed, -kMaxReverse, maxSpeed+landingSpeedExcess_);
    if (boostRemaining > 0 && boostLevel > 0 && boostLevel <= 3 && speed > 0) {
        boostSpeedBonus = kBoostPeaks[boostLevel] * boostSpeedScale * boostEnvelopeAmount();
        speed = clampf(speed + boostSpeedBonus, -kMaxReverse, maxSpeed+landingSpeedExcess_+boostSpeedBonus);
    }
    if (itemKickTime >= 0 && (input.brake > 0.05f || !onAsphalt || speed <= 0)) itemKickTime = -1;
    if (itemKickTime >= 0) {
        // Quick rise, brief hold, ease out; smaller than any boost.
        itemKickTime += dt;
        if (itemKickTime >= kKickAttack + kKickHold + kKickReturn) itemKickTime = -1;
        else {
            const float amount = itemKickTime < kKickAttack ? smoothBoost(itemKickTime / kKickAttack) :
                itemKickTime < kKickAttack + kKickHold ? 1.0f :
                1 - smoothBoost((itemKickTime - kKickAttack - kKickHold) / kKickReturn);
            itemKickBonus = itemKickPeak * amount;
            speed += itemKickBonus;
        }
    }
    wheelSpin += speed * moveScale / kWheelRadius * dt;

    stepDrift(dt, input);

    updateSupport(dt,track);
    if (impact > 0.05f) itemKickTime = -1;
    if (!ground.found) {
        cancelReward();
        return;
    }

    if (!grounded) {
        driftIntensity=0;
        // A release that has already left the road cannot award a new boost.
        if (manualRelease) {
            driftCharge=0;driftChargeLevel=0;
            cancelReleaseEase();
        }
        return;
    }
    driftIntensity = slipFeedback(speed, slipAngle, onAsphalt);
    if (!onAsphalt || impact > 0.05f || input.brake > 0.05f) {
        cancelReward();
    } else if (speed <= kMinDriftSpeed) {
        cancelReward(driftHoldActive || drifting);
    } else if (releaseLevel > 0) {
        // 報酬は路面・衝突の確認後に一度だけ成立する。新しい travelYaw に沿う
        // 速度へ足すだけで、ブースト自体は方向や位置を変えない。
        boostLevel = releaseLevel;
        captureBoost(*this,configuredBoost(boostSpeedAttack,boostSpeedHold,boostSpeedReturn,boostDurationScale));
        boostSpeedBonus = 0; // The release event starts the attack before any gain is added.
        ++boostSerial;
    } else if (driftHeld && driftHoldActive && std::fabs(slipAngle) >= kMinChargeAngle) {
        driftCharge = std::fmin(driftCharge + dt * chargeRateScale, kChargeThresholds[3]);
        driftChargeLevel = chargeLevelOf(driftCharge);
    }
    driftIntensity = slipFeedback(speed, slipAngle, onAsphalt);
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
