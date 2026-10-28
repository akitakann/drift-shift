#include "car.hpp"
#include "render.hpp"
#include "track.hpp"
#include "audio.hpp"
#include "trial.hpp"
#include "tuning.hpp"
#include "presentation.hpp"
#include "vehicle_presentation.hpp"
#include "effects.hpp"
#include "items.hpp"
#include <SDL.h>
#include <SDL_image.h>
#include <SDL_mixer.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>
#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#include <emscripten/html5.h>
#endif

namespace {
constexpr double kFixedStepSec = 1.0 / 60.0;
constexpr float kDegrees = 57.295779513f;
using Orbit = CameraPresentation;
struct RuntimeTuningField { const char* name; float FeelSettings::* member; };
const RuntimeTuningField kRuntimeTuningFields[] = {
    {"recoveryTime", &FeelSettings::recoveryTime},
    {"topSpeedKmh", &FeelSettings::topSpeedKmh},
    {"accelerationTime", &FeelSettings::accelerationTime},
    {"stageScale", &FeelSettings::stageScale},
    {"releaseDirectionTime", &FeelSettings::releaseDirectionTime},
    {"driftSteerRate", &FeelSettings::driftSteerRate},
    {"driftSteerResponse", &FeelSettings::driftSteerResponse},
    {"cameraTravelWeight", &FeelSettings::cameraTravelWeight},
    {"cameraResponseTime", &FeelSettings::cameraResponseTime},
    {"fovGain", &FeelSettings::fovGain},
    {"smokeScale", &FeelSettings::smokeScale},
    {"audioScale", &FeelSettings::audioScale},
    {"engineGain", &FeelSettings::engineGain},
    {"tireGain", &FeelSettings::tireGain},
    {"cameraDistance", &FeelSettings::cameraDistance},
    {"cameraHeight", &FeelSettings::cameraHeight},
    {"cameraLookAhead", &FeelSettings::cameraLookAhead},
    {"cameraEntryTime", &FeelSettings::cameraEntryTime},
    {"cameraExitTime", &FeelSettings::cameraExitTime},
    {"cameraExitPower", &FeelSettings::cameraExitPower},
    {"cameraPreviewWeight", &FeelSettings::cameraPreviewWeight},
    {"cameraSteerLookAhead", &FeelSettings::cameraSteerLookAhead},
    {"cameraSteerLookMax", &FeelSettings::cameraSteerLookMax},
    {"baseFov", &FeelSettings::baseFov},
    {"boostFovPeak", &FeelSettings::boostFovPeak},
    {"boostFovAttack", &FeelSettings::boostFovAttack},
    {"boostFovHold", &FeelSettings::boostFovHold},
    {"boostFovReturn", &FeelSettings::boostFovReturn},
    {"chargeFovPeak", &FeelSettings::chargeFovPeak},
    {"chargeShakeScale", &FeelSettings::chargeShakeScale},
    {"boostSetbackPeak", &FeelSettings::boostSetbackPeak},
    {"boostSetbackAttack", &FeelSettings::boostSetbackAttack},
    {"boostSetbackHold", &FeelSettings::boostSetbackHold},
    {"boostSetbackReturn", &FeelSettings::boostSetbackReturn},
    {"boostKickReturn", &FeelSettings::boostKickReturn},
    {"boostSpeedScale", &FeelSettings::boostSpeedScale},
    {"boostDurationScale", &FeelSettings::boostDurationScale},
    {"boostSpeedAttack", &FeelSettings::boostSpeedAttack},
    {"boostSpeedHold", &FeelSettings::boostSpeedHold},
    {"boostSpeedReturn", &FeelSettings::boostSpeedReturn},
    {"chargeRateScale", &FeelSettings::chargeRateScale},
    {"bodyRollMax", &FeelSettings::bodyRollMax},
    {"bodyPitchMax", &FeelSettings::bodyPitchMax},
    {"bodyResponseTime", &FeelSettings::bodyResponseTime},
    {"bodyRecoveryTime", &FeelSettings::bodyRecoveryTime},
    {"bodyBounceHeight", &FeelSettings::bodyBounceHeight},
    {"bodyBounceReturn", &FeelSettings::bodyBounceReturn},
    {"boostBlurStrength", &FeelSettings::boostBlurStrength},
    {"gravity", &FeelSettings::gravity},
    {"landingImpactScale", &FeelSettings::landingImpactScale},
    {"speedCueScale", &FeelSettings::speedCueScale},
    {"windGain", &FeelSettings::windGain},
    {"suspensionTravel", &FeelSettings::suspensionTravel},
    {"maxLaunchSpeed", &FeelSettings::maxLaunchSpeed},
    {"airSteerControl", &FeelSettings::airSteerControl},
    {"engineRevScale", &FeelSettings::engineRevScale},
    {"engineRevReturn", &FeelSettings::engineRevReturn},
};
struct TireMark { Vec3 a,b; float age=0; };
struct Smoke { Vec3 position; float age=0, strength=0; };
struct App {
    SDL_Window* window=nullptr;
    SDL_GLContext gl=nullptr;
    SDL_GameController* pad=nullptr;
    Renderer renderer;
    Track track;
    Car car;
    GpuMesh course,body,wheel,shadow,tireMesh,smokeMesh,boostJet,fxMesh;
    DriftEffects fx;
    ChargeItems items;
    GpuMesh itemSolid,itemGlow;
    GpuMesh chargeWheel[3],chargeHalo[3];
    DriftAudio audio;
    TrialSession trial;
    FeelSettings settings;
    FeelSettings recordingSettings;
    DriftModel selectedModel=DriftModel::Inertia, recordingModel=DriftModel::Inertia;
    FeelGroup group=FeelGroup::Handling;
    int candidate=0, runId=0, frameNumber=0, recordingGroup=0, recordingCandidate=0;
    int recordingRunId=0;
    Orbit camera;
    VehiclePresentation vehicle;
    std::deque<TireMark> tireMarks;
    std::vector<Smoke> smoke;
    Vec3 lastTire[4];
    bool tiresReady=false, running=true, paused=true, started=false, muted=false;
    bool replay=false, ended=false, entryReached=false;
    bool recordingTruncated=false;
    bool circuit=false;
    float stageScale=1;
    bool liveTuningChanged=false, recordingTuningChanged=false;
    int tuningOverrideCount=0, configRevision=0;
    float smokeAccumulator=0, stateTime=0, finishedAge=0;
    float currentImpact=0;
    float currentLandingImpact=0;
    float maxAirHeight=0,totalAirTime=0,speedCueAmount=0;
    float takeoffY=0,maxTakeoffRise=0,currentAcceleration=0;
    std::vector<DriveInput> lastRecording,currentRecording;
    size_t replayFrame=0;
    double accumulator=0;
    Uint64 prevCounter=0;
    bool freeDriving() const { return track.practiceArea || circuit; }
    const char* courseId() const { return circuit ? "suzuka" : "practice"; }
};

std::string findTrackPath() {
    for (const char* p : {"assets/track/suzuka.txt","../assets/track/suzuka.txt","/assets/track/suzuka.txt"}) {
        std::ifstream in(p); if(in) return p;
    }
    return "assets/track/suzuka.txt";
}
void openPad(App& app) {
#ifdef __EMSCRIPTEN__
    // ブラウザのパッドは Gamepad API 側で読む。SDL で開くと入力が二重になる。
    (void)app;
    return;
#else
    if (app.pad) {
        return;
    }
    for (int i = 0; i < SDL_NumJoysticks(); ++i) {
        if (SDL_IsGameController(i)) {
            app.pad = SDL_GameControllerOpen(i);
            if (app.pad) {
                SDL_Log("controller: %s", SDL_GameControllerName(app.pad));
                break;
            }
        }
    }
#endif
}

struct PadPulse {
    bool reset = false;
    bool pause = false;
    bool camera = false;
};

float stickDead(float value) {
    const float dead = 0.14f;
    if (std::fabs(value) < dead) {
        return 0;
    }
    float sign = value > 0 ? 1.0f : -1.0f;
    return (value - sign * dead) / (1.0f - dead);
}

#ifndef __EMSCRIPTEN__
float stickAxis(SDL_GameController* pad, SDL_GameControllerAxis axis) {
    return stickDead(SDL_GameControllerGetAxis(pad, axis) / 32767.0f);
}
#endif

#ifdef __EMSCRIPTEN__
// ブラウザの Gamepad API は 0 が離した状態、1 が押し込み。古いパッドは -1..1 で、離すと -1。
float triggerLevel(float value) {
    if (value < 0) {
        value = (value + 1.0f) * 0.5f;
    }
    return value < 0.04f ? 0.0f : clampf(value, 0.0f, 1.0f);
}

PadPulse readBrowserPad(DriveInput& input) {
    PadPulse pulse;
    static bool prevCamera = false;
    static bool prevReset = false;
    static bool prevPause = false;
    bool camera = false;
    bool reset = false;
    bool pause = false;
    if (emscripten_sample_gamepad_data() != EMSCRIPTEN_RESULT_SUCCESS) {
        prevCamera = prevReset = prevPause = false;
        return pulse;
    }
    const int count = emscripten_get_num_gamepads();
    for (int i = 0; i < count; ++i) {
        EmscriptenGamepadEvent pad;
        if (emscripten_get_gamepad_status(i, &pad) != EMSCRIPTEN_RESULT_SUCCESS || !pad.connected) {
            continue;
        }
        auto axis = [&](int index) {
            return index < pad.numAxes ? static_cast<float>(pad.axis[index]) : 0.0f;
        };
        auto analog = [&](int index) {
            return index < pad.numButtons ? static_cast<float>(pad.analogButton[index]) : 0.0f;
        };
        auto held = [&](int index) {
            return index < pad.numButtons && pad.digitalButton[index];
        };
        const bool standard = std::strcmp(pad.mapping, "standard") == 0 || (pad.numButtons >= 16 && pad.numAxes >= 2);
        input.steer += stickDead(axis(0));
        if (standard) {
            input.steer += stickDead(axis(2));
            if (held(14)) input.steer -= 1;
            if (held(15)) input.steer += 1;
            if (held(12)) input.throttle = 1;
            if (held(13)) input.brake = 1;
            float throttle = analog(7);
            float brake = analog(6);
            if (throttle < 0.01f && pad.numAxes > 5) throttle = triggerLevel(axis(5));
            if (brake < 0.01f && pad.numAxes > 4) brake = triggerLevel(axis(4));
            input.throttle = std::max(input.throttle, throttle);
            input.brake = std::max(input.brake, brake);
            input.handbrake = std::max(input.handbrake, analog(0));
            camera = camera || held(3);
            reset = reset || held(8);
            pause = pause || held(9);
        } else if (pad.numAxes >= 6) {
            input.throttle = std::max(input.throttle, triggerLevel(axis(5)));
            input.brake = std::max(input.brake, triggerLevel(axis(4)));
        }
    }
    pulse.camera = camera && !prevCamera;
    pulse.reset = reset && !prevReset;
    pulse.pause = pause && !prevPause;
    prevCamera = camera;
    prevReset = reset;
    prevPause = pause;
    return pulse;
}
#endif

DriveInput readDrive(SDL_GameController* pad, PadPulse& pulse) {
    const Uint8* keys = SDL_GetKeyboardState(nullptr);
    DriveInput input;
    bool keyboardActive=true;
#ifdef __EMSCRIPTEN__
    keyboardActive=EM_ASM_INT({const focus=document.activeElement;return focus===document.getElementById('canvas')||!!(focus&&focus.closest('#tuning-panel'));});
#endif
    if(keyboardActive) {
    if (keys[SDL_SCANCODE_W] || keys[SDL_SCANCODE_UP]) input.throttle = 1;
    if (keys[SDL_SCANCODE_S] || keys[SDL_SCANCODE_DOWN]) input.brake = 1;
    if (keys[SDL_SCANCODE_A] || keys[SDL_SCANCODE_LEFT]) input.steer -= 1;
    if (keys[SDL_SCANCODE_D] || keys[SDL_SCANCODE_RIGHT]) input.steer += 1;
    if (keys[SDL_SCANCODE_SPACE]) input.handbrake = 1;
    }
#ifndef __EMSCRIPTEN__
    if (pad) {
        float throttle = stickAxis(pad, SDL_CONTROLLER_AXIS_TRIGGERRIGHT);
        float brake = stickAxis(pad, SDL_CONTROLLER_AXIS_TRIGGERLEFT);
        if (throttle < 0) throttle = 0;
        if (brake < 0) brake = 0;
        input.throttle = std::max(input.throttle, throttle);
        input.brake = std::max(input.brake, brake);
        if (SDL_GameControllerGetButton(pad, SDL_CONTROLLER_BUTTON_A)) input.handbrake = 1;
        input.steer += stickAxis(pad, SDL_CONTROLLER_AXIS_LEFTX);
        input.steer += stickAxis(pad, SDL_CONTROLLER_AXIS_RIGHTX);
        if (SDL_GameControllerGetButton(pad, SDL_CONTROLLER_BUTTON_DPAD_UP)) input.throttle = 1;
        if (SDL_GameControllerGetButton(pad, SDL_CONTROLLER_BUTTON_DPAD_DOWN)) input.brake = 1;
        if (SDL_GameControllerGetButton(pad, SDL_CONTROLLER_BUTTON_DPAD_LEFT)) input.steer -= 1;
        if (SDL_GameControllerGetButton(pad, SDL_CONTROLLER_BUTTON_DPAD_RIGHT)) input.steer += 1;
    }
#else
    (void)pad;
#endif
#ifdef __EMSCRIPTEN__
    // SDL はブラウザのパッドをゲームコントローラと認識しないことが多い。
    // 標準マッピングを Gamepad API から直接読む。
    PadPulse browser = readBrowserPad(input);
    pulse.camera = pulse.camera || browser.camera;
    pulse.reset = pulse.reset || browser.reset;
    pulse.pause = pulse.pause || browser.pause;
#else
    (void)pulse;
#endif
    input.throttle = clampf(input.throttle, 0, 1);
    input.brake = clampf(input.brake, 0, 1);
    input.steer = clampf(input.steer, -1, 1);
    return input;
}


void readSettings(App& app) {
    app.tuningOverrideCount=0;
#ifdef __EMSCRIPTEN__
    int group=EM_ASM_INT({return window.driftLab ? window.driftLab.group|0 : 0;});
    int candidate=EM_ASM_INT({return window.driftLab ? window.driftLab.candidate|0 : 1;});
    app.group=static_cast<FeelGroup>(std::max(0,std::min(5,group)));
    app.candidate=std::max(0,std::min(2,candidate));
    const int model=app.group==FeelGroup::Handling ? std::min(app.candidate,1) :
        EM_ASM_INT({return window.driftLab ? window.driftLab.handlingModel|0 : 0;});
    app.selectedModel=model==1?DriftModel::LineControl:DriftModel::Inertia;
#endif
    app.settings=feelSettings(app.group,app.candidate);
    if(app.group==FeelGroup::Handling) {
        app.candidate=app.selectedModel==DriftModel::LineControl?1:0;
        app.settings=FeelSettings{};
    }
#ifdef __EMSCRIPTEN__
    if(app.group==FeelGroup::Combined && app.candidate==1) {
        const int model=EM_ASM_INT({return (window.driftLab.combined||{}).handlingModel|0;});
        app.selectedModel=model==1?DriftModel::LineControl:DriftModel::Inertia;
        app.settings.cameraTravelWeight=clampf(EM_ASM_DOUBLE({return +(window.driftLab.combined||{}).cameraTravelWeight||0.72;}),0.5f,0.9f);
        app.settings.smokeScale=clampf(EM_ASM_DOUBLE({var v=(window.driftLab.combined||{}).smokeScale; return typeof v==='number'?v:1;}),0,2);
        app.settings.audioScale=clampf(EM_ASM_DOUBLE({var v=(window.driftLab.combined||{}).audioScale; return typeof v==='number'?v:1;}),0,1.5f);
        app.settings.fovGain=clampf(EM_ASM_DOUBLE({return +(window.driftLab.combined||{}).fovGain||0;}),0,6);
    }
    for(const auto& field:kRuntimeTuningFields) {
        const float value=float(EM_ASM_DOUBLE({
            const config=window.driftLab&&window.driftLab.tuningOverrides;
            const key=UTF8ToString($0);
            if(!config||typeof config!=='object'||!Object.prototype.hasOwnProperty.call(config,key))return NaN;
            const value=config[key];
            return typeof value==='number'&&Number.isFinite(value)?value:NaN;
        },field.name));
        if(std::isfinite(value)) {app.settings.*(field.member)=value;++app.tuningOverrideCount;}
    }
#else
    // Native builds read the same release preset as the browser page.
    for(const char* path:{"web/release-preset.json","../web/release-preset.json"}) {
        std::ifstream in(path);
        if(!in) continue;
        std::stringstream buffer;buffer<<in.rdbuf();
        const std::string text=buffer.str();
        if(text.find("\"handlingModel\": 1")!=std::string::npos) app.selectedModel=DriftModel::LineControl;
        for(const auto& field:kRuntimeTuningFields) {
            const std::string key=std::string("\"")+field.name+"\"";
            const size_t at=text.find(key);
            if(at==std::string::npos) continue;
            const size_t colon=text.find(':',at+key.size());
            if(colon==std::string::npos) continue;
            const float value=std::strtof(text.c_str()+colon+1,nullptr);
            if(std::isfinite(value)) {app.settings.*(field.member)=value;++app.tuningOverrideCount;}
        }
        break;
    }
#endif
    app.settings=sanitizeFeelSettings(app.settings);
}

int requestedTuningRevision() {
#ifdef __EMSCRIPTEN__
    return EM_ASM_INT({const revision=(window.driftLab||{}).tuningRevision;return Number.isInteger(revision)&&revision>=0?Math.min(revision,2147483647):0;});
#else
    return 0;
#endif
}

void applyStageScale(App& app,bool preservePose) {
    if(std::fabs(app.settings.stageScale-app.stageScale)<0.000001f) return;
    const float ratio=app.settings.stageScale/app.stageScale;
    const Vec3 previous{app.car.x,app.car.y,app.car.z};
    app.track.scaleBy(ratio);
    app.items.build(app.track);
    app.stageScale=app.settings.stageScale;
    app.renderer.updateMesh(app.course,app.track.visual);
    if(preservePose) {
        app.car.rescaleWorld(ratio,app.track);
        app.takeoffY*=ratio;app.maxTakeoffRise*=ratio;app.maxAirHeight*=ratio;
        const Vec3 shift=Vec3{app.car.x,app.car.y,app.car.z}-previous;
        // Translate the complete view without scaling its distance to the car.
        app.camera.anchor=app.camera.anchor+shift;
        app.camera.eye=app.camera.eye+shift;app.camera.look=app.camera.look+shift;
        app.vehicle.rebaseVelocity(app.car);
    }
    app.tireMarks.clear();app.smoke.clear();app.tiresReady=false;app.smokeAccumulator=0;app.fx.reset();
}

bool applyLiveTuning(App& app) {
    const int revision=requestedTuningRevision();
    if(revision==app.configRevision) return false;
    const FeelSettings previous=app.settings;
    readSettings(app);
    // Input replay must retain the source vehicle's physical trajectory.
    if(app.replay) {
        app.settings.recoveryTime=app.recordingSettings.recoveryTime;
        app.settings.topSpeedKmh=app.recordingSettings.topSpeedKmh;
        app.settings.accelerationTime=app.recordingSettings.accelerationTime;
        app.settings.driftSteerRate=app.recordingSettings.driftSteerRate;
        app.settings.driftSteerResponse=app.recordingSettings.driftSteerResponse;
        app.settings.boostSpeedScale=app.recordingSettings.boostSpeedScale;
        app.settings.boostDurationScale=app.recordingSettings.boostDurationScale;
        app.settings.boostSpeedAttack=app.recordingSettings.boostSpeedAttack;
        app.settings.boostSpeedHold=app.recordingSettings.boostSpeedHold;
        app.settings.boostSpeedReturn=app.recordingSettings.boostSpeedReturn;
        app.settings.chargeRateScale=app.recordingSettings.chargeRateScale;
        app.settings.stageScale=app.recordingSettings.stageScale;
        app.settings.releaseDirectionTime=app.recordingSettings.releaseDirectionTime;
        app.settings.gravity=app.recordingSettings.gravity;
        app.settings.suspensionTravel=app.recordingSettings.suspensionTravel;
        app.settings.maxLaunchSpeed=app.recordingSettings.maxLaunchSpeed;
        app.settings.airSteerControl=app.recordingSettings.airSteerControl;
    }
    bool changed=false;
    for(const auto& field:kRuntimeTuningFields)
        changed=changed||std::fabs(app.settings.*(field.member)-previous.*(field.member))>0.000001f;
    if(changed) {
        const float previousSpeed=app.car.speed;
        app.car.applyTuning(app.settings);
        applyStageScale(app,true);
        if(app.car.speed!=previousSpeed) app.vehicle.rebaseVelocity(app.car);
        app.camera.applySettings(app.car,previous,app.settings);
        if(app.started&&app.frameNumber>0) {
            app.liveTuningChanged=true;
            // An input-only recording cannot reconstruct changed settings.
            if(!app.replay) app.lastRecording.clear();
        }
    }
    app.configRevision=revision;
    return true;
}

void resetTrial(App& app,bool replay=false) {
    readSettings(app);
    if(app.group==FeelGroup::Combined && app.candidate==0) app.selectedModel=DriftModel::Inertia;
    if(replay) {
        app.settings.recoveryTime=app.recordingSettings.recoveryTime;
        app.settings.topSpeedKmh=app.recordingSettings.topSpeedKmh;
        app.settings.accelerationTime=app.recordingSettings.accelerationTime;
        app.settings.driftSteerRate=app.recordingSettings.driftSteerRate;
        app.settings.driftSteerResponse=app.recordingSettings.driftSteerResponse;
        app.settings.boostSpeedScale=app.recordingSettings.boostSpeedScale;
        app.settings.boostDurationScale=app.recordingSettings.boostDurationScale;
        app.settings.boostSpeedAttack=app.recordingSettings.boostSpeedAttack;
        app.settings.boostSpeedHold=app.recordingSettings.boostSpeedHold;
        app.settings.boostSpeedReturn=app.recordingSettings.boostSpeedReturn;
        app.settings.chargeRateScale=app.recordingSettings.chargeRateScale;
        app.settings.stageScale=app.recordingSettings.stageScale;
        app.settings.releaseDirectionTime=app.recordingSettings.releaseDirectionTime;
        app.settings.gravity=app.recordingSettings.gravity;
        app.settings.suspensionTravel=app.recordingSettings.suspensionTravel;
        app.settings.maxLaunchSpeed=app.recordingSettings.maxLaunchSpeed;
        app.settings.airSteerControl=app.recordingSettings.airSteerControl;
    }
    applyStageScale(app,false);
    app.car.reset(app.track);
    app.items.reset();
    app.car.speed=app.freeDriving()?0:TrialCourse::entrySpeedKmh/3.6f;
    app.car.driftModel=replay?app.recordingModel:app.selectedModel;
    app.car.applyTuning(app.settings);
    app.trial.reset();
    app.audio.reset();
    app.camera.ready=false;
    app.vehicle.reset();
    app.tireMarks.clear(); app.smoke.clear(); app.tiresReady=false; app.fx.reset();
    app.smokeAccumulator=0; app.currentRecording.clear();
    app.replay=replay; app.replayFrame=0; app.frameNumber=0;
    app.recordingTruncated=false;
    app.liveTuningChanged=false;
    app.configRevision=requestedTuningRevision();
    app.ended=false; app.entryReached=false; app.finishedAge=0;
    app.paused=!app.started;
    app.accumulator=0; app.currentImpact=app.currentLandingImpact=0; app.stateTime=1;
    app.maxAirHeight=app.totalAirTime=app.speedCueAmount=0;
    app.takeoffY=app.maxTakeoffRise=app.currentAcceleration=0;
    ++app.runId;
}

void publishState(App& app,bool samples=false) {
#ifdef __EMSCRIPTEN__
    const auto& m=app.trial.metrics;
    std::ostringstream out; out<<std::fixed<<std::setprecision(5);
    out<<"{\"ready\":true,\"running\":"<<(app.started&&!app.paused&&!app.ended?"true":"false")
       <<",\"freePractice\":"<<(app.freeDriving()?"true":"false")
       <<",\"courseId\":\""<<app.courseId()<<"\""
       <<",\"stageScale\":"<<app.stageScale<<",\"trackHalfWidth\":"<<app.track.halfWidth<<",\"trackLength\":"<<app.track.length
       <<",\"carX\":"<<app.car.x<<",\"carY\":"<<app.car.y<<",\"carZ\":"<<app.car.z
       <<",\"handlingModel\":"<<int(app.car.driftModel)
       <<",\"samplesTruncated\":"<<(app.recordingTruncated||app.trial.samplesTruncated?"true":"false")
       <<",\"paused\":"<<(app.paused?"true":"false")<<",\"replay\":"<<(app.replay?"true":"false")
       <<",\"group\":"<<int(app.group)<<",\"candidate\":"<<app.candidate<<",\"frame\":"<<app.frameNumber
       <<",\"configRevision\":"<<app.configRevision<<",\"liveTuningChanged\":"<<(app.liveTuningChanged?"true":"false")
       <<",\"speed\":"<<app.car.speed*3.6f<<",\"slip\":"<<app.car.slipAngle*kDegrees
       <<",\"slipSpeed\":"<<lateralSlipSpeed(app.car.speed,app.car.slipAngle)
       <<",\"driftCharge\":"<<app.car.driftCharge<<",\"chargeLevel\":"<<app.car.driftChargeLevel
       <<",\"boostRemaining\":"<<app.car.boostRemaining<<",\"boostLevel\":"<<app.car.boostLevel
       <<",\"boostTotalDuration\":"<<app.car.boostTotalDuration
       <<",\"boostAttackDuration\":"<<app.car.boostAttackDuration<<",\"boostHoldDuration\":"<<app.car.boostHoldDuration
       <<",\"boostReturnDuration\":"<<app.car.boostReturnDuration<<",\"boostEnvelope\":"<<app.car.boostEnvelopeAmount()
       <<",\"boostBonus\":"<<app.car.boostSpeedBonus*3.6f<<",\"normalSpeed\":"<<(app.car.speed-app.car.boostSpeedBonus)*3.6f
       <<",\"yawRate\":"<<app.car.yawRate*kDegrees
       <<",\"bodyRoll\":"<<app.vehicle.roll*kDegrees<<",\"bodyPitch\":"<<app.vehicle.pitch*kDegrees
       <<",\"bodyHeave\":"<<app.vehicle.heave
       <<",\"grounded\":"<<(app.car.grounded?"true":"false")
       <<",\"surface\":\""<<(!app.car.grounded?"air":std::fabs(app.car.ground.lateral)>app.track.halfWidth+app.track.curbWidth?"grass":std::fabs(app.car.ground.lateral)>app.track.halfWidth?"curb":"asphalt")<<"\""
       <<",\"groundHeight\":"<<app.car.groundHeight<<",\"verticalSpeed\":"<<app.car.verticalSpeed
       <<",\"airHeight\":"<<std::fmax(app.car.y-app.car.groundHeight,0.0f)<<",\"airTime\":"<<app.car.airTime
       <<",\"terrainPitch\":"<<app.car.terrainPitch*kDegrees<<",\"terrainRoll\":"<<app.car.terrainRoll*kDegrees
       <<",\"landingSerial\":"<<app.car.landingSerial<<",\"landingImpact\":"<<app.car.landingImpact
       <<",\"landingCompression\":"<<app.vehicle.landingCompression
       <<",\"maxAirHeight\":"<<app.maxAirHeight<<",\"totalAirTime\":"<<app.totalAirTime
       <<",\"maxTakeoffRise\":"<<app.maxTakeoffRise<<",\"cameraEyeY\":"<<app.camera.eye.y
       <<",\"acceleration\":"<<app.currentAcceleration
       <<",\"tireMarkCount\":"<<app.tireMarks.size()<<",\"smokeCount\":"<<app.smoke.size()
       <<",\"speedCueAmount\":"<<app.speedCueAmount
       <<",\"engineSampleLoaded\":"<<(app.audio.engineSampleLoaded()?"true":"false")
       <<",\"tireSampleLoaded\":"<<(app.audio.tireSampleLoaded()?"true":"false")
       <<",\"boostBlur\":"<<app.renderer.appliedBlur<<",\"postAvailable\":"<<(app.renderer.postAvailable?"true":"false")
       <<",\"cameraFov\":"<<app.camera.fovDegrees<<",\"boostVisual\":"<<app.camera.boostAmount
       <<",\"cameraHeading\":"<<app.camera.heading*kDegrees
       <<",\"cameraLookHeading\":"<<app.camera.lookHeading*kDegrees<<",\"cameraBoostPreview\":"<<app.camera.boostPreview
       <<",\"cameraSteerPreview\":"<<app.camera.steerLookOffset*kDegrees
       <<",\"cameraCharge\":"<<app.camera.chargeAmount<<",\"cameraBoostKick\":"<<app.camera.boostViewAmount
       <<",\"cameraFovTail\":"<<app.camera.boostFovAmount
       <<",\"cameraSetback\":"<<app.camera.boostSetback<<",\"cameraShake\":"<<length(app.camera.chargeShake)
       <<",\"cameraChargeFov\":"<<app.camera.chargeFovOffset<<",\"cameraBoostFov\":"<<app.camera.boostFovOffset
       <<",\"cameraPhase\":"<<int(app.camera.motionPhase)<<",\"cameraPhaseElapsed\":"<<app.camera.motionElapsed
       <<",\"carYaw\":"<<app.car.yaw*kDegrees<<",\"travelYaw\":"<<app.car.travelYaw*kDegrees
       <<",\"driftHeld\":"<<(app.car.driftHeld?"true":"false")
       <<",\"driftHoldActive\":"<<(app.car.driftHoldActive?"true":"false")
       <<",\"driftPhase\":"<<int(app.car.driftPhase)
       <<",\"releaseUsesCameraHeading\":"<<(app.car.releaseUsesCameraHeading?"true":"false")
       <<",\"releaseHeading\":"<<app.car.releaseHeading*kDegrees
       <<",\"releaseElapsed\":"<<app.car.releaseEaseElapsed
       <<",\"hold\":"<<m.holdSeconds<<",\"contacts\":"<<m.contacts<<",\"offroad\":"<<m.offroad
       <<",\"finished\":"<<(app.ended?"true":"false")<<",\"passed\":"<<(m.passed?"true":"false")
       <<",\"timedOut\":"<<(m.timedOut?"true":"false")<<",\"exitSpeed\":"<<(app.freeDriving()?app.car.speed*3.6f:m.exitSpeedKmh)
       <<",\"exitSlip\":"<<(app.freeDriving()?std::fabs(app.car.slipAngle)*kDegrees:m.exitSlipDeg)<<",\"entrySpeed\":"<<m.entrySpeedKmh<<",\"duration\":"<<m.duration
       <<",\"replayAvailable\":"<<(!app.lastRecording.empty()&&!app.recordingTuningChanged?"true":"false")<<",\"runId\":"<<app.runId
       <<",\"recordingGroup\":"<<app.recordingGroup<<",\"recordingCandidate\":"<<app.recordingCandidate
       <<",\"recordingRunId\":"<<app.recordingRunId
       <<",\"recordingHandlingModel\":"<<int(app.recordingModel)
       <<",\"recordingRecoveryTime\":"<<app.recordingSettings.recoveryTime
       <<",\"recordingTopSpeedKmh\":"<<app.recordingSettings.topSpeedKmh
       <<",\"recordingAccelerationTime\":"<<app.recordingSettings.accelerationTime
       <<",\"recordingDriftSteerRate\":"<<app.recordingSettings.driftSteerRate
       <<",\"recordingDriftSteerResponse\":"<<app.recordingSettings.driftSteerResponse
       <<",\"recordingBoostSpeedScale\":"<<app.recordingSettings.boostSpeedScale
       <<",\"recordingBoostDurationScale\":"<<app.recordingSettings.boostDurationScale
       <<",\"recordingBoostSpeedAttack\":"<<app.recordingSettings.boostSpeedAttack
       <<",\"recordingBoostSpeedHold\":"<<app.recordingSettings.boostSpeedHold
       <<",\"recordingBoostSpeedReturn\":"<<app.recordingSettings.boostSpeedReturn
       <<",\"recordingChargeRateScale\":"<<app.recordingSettings.chargeRateScale
       <<",\"recordingStageScale\":"<<app.recordingSettings.stageScale
       <<",\"recordingReleaseDirectionTime\":"<<app.recordingSettings.releaseDirectionTime
       <<",\"recordingGravity\":"<<app.recordingSettings.gravity
       <<",\"recordingSuspensionTravel\":"<<app.recordingSettings.suspensionTravel
       <<",\"recordingMaxLaunchSpeed\":"<<app.recordingSettings.maxLaunchSpeed
       <<",\"recordingAirSteerControl\":"<<app.recordingSettings.airSteerControl
       <<",\"settings\":{\"mechanicsVersion\":\"release-v1-pressure-grass\",\"audioProfile\":\"recorded-engine-c-tire-a-pressure-charge-rev\",\"cameraProfile\":\"camera-launch-relative-exit\",\"bodyProfile\":\"suspension-contact-bounded-launch\",\"postProfile\":\"boost-peripheral-radial-blur-speed-lines\",\"boostSpeedProfile\":\"attack-hold-return\",\"manualRelease\":\"ease-out-to-captured-camera\",\"releaseEaseTime\":"<<app.settings.releaseDirectionTime<<",\"unchargedReleaseEaseTime\":0.15,\"heldSlipMin\":45,\"heldSlipMax\":60,\"driftDeceleration\":1,\"driftMaxSpeedLoss\":0.1,\"handlingModel\":"<<int(app.car.driftModel)
       <<",\"freePractice\":"<<(app.freeDriving()?"true":"false")
       <<",\"courseId\":\""<<app.courseId()<<"\""
       <<",\"customTuning\":"<<(app.tuningOverrideCount>0?"true":"false")<<",\"tuningOverrideCount\":"<<app.tuningOverrideCount
       <<",\"cameraEntryDuration\":"<<app.settings.cameraEntryTime<<",\"cameraEntryDelay\":"<<CameraPresentation::entryDelay
       <<",\"cameraExitDuration\":"<<app.settings.cameraExitTime<<",\"cameraRecoveryDuration\":"<<CameraPresentation::recoveryDuration
       <<",\"cameraEntryEasing\":\"quintic-in-out\",\"cameraExitEasing\":\"power-out\""
       <<",\"cameraBoostLookWeight\":"<<app.settings.cameraPreviewWeight<<",\"cameraBoostPreviewResponseTime\":"<<CameraPresentation::boostPreviewResponseTime
       <<",\"chargeShakeHorizontal\":"<<CameraPresentation::chargeShakeHorizontal*app.settings.chargeShakeScale
       <<",\"chargeShakeVertical\":"<<CameraPresentation::chargeShakeVertical*app.settings.chargeShakeScale
       <<",\"chargeResponseTime\":"<<CameraPresentation::chargeResponseTime<<",\"chargeReleaseTime\":"<<CameraPresentation::chargeReleaseTime
       <<",\"boostCameraAttackTime\":"<<app.settings.boostSetbackAttack<<",\"boostCameraHoldTime\":"<<app.settings.boostSetbackHold<<",\"boostCameraReturnTime\":"<<app.settings.boostSetbackReturn
       <<",\"boostCameraEasing\":\"quintic-in-out\",\"boostJetAttackTime\":"<<CameraPresentation::boostAttackTime
       <<",\"boostJetReturnTime\":"<<app.settings.boostKickReturn<<",\"boostFovCancelTime\":"<<app.settings.boostKickReturn
       <<",\"boostFovAttackTime\":"<<app.settings.boostFovAttack<<",\"boostFovHoldTime\":"<<app.settings.boostFovHold<<",\"boostFovReturnTime\":"<<app.settings.boostFovReturn;
    for(const auto& field:kRuntimeTuningFields) out<<",\""<<field.name<<"\":"<<(app.settings.*(field.member));
    out<<"}";
    if(samples) {
        out<<",\"samples\":[";
        bool first=true;
        for(const auto& s:app.trial.samples) {
            if(!first) out<<','; first=false;
            out<<"{\"time\":"<<s.time<<",\"input\":{\"throttle\":"<<s.input.throttle<<",\"brake\":"<<s.input.brake
               <<",\"steer\":"<<s.input.steer<<",\"handbrake\":"<<s.input.handbrake
               <<",\"hasBoostHeading\":"<<(s.input.hasBoostHeading?"true":"false")<<",\"boostHeading\":"<<s.input.boostHeading<<"},\"speed\":"<<s.speed
               <<",\"slip\":"<<s.slip<<",\"slipSpeed\":"<<s.slipSpeed<<",\"s\":"<<s.s<<",\"lateral\":"<<s.lateral
               <<",\"asphalt\":"<<(s.asphalt?"true":"false")<<",\"impact\":"<<s.impact
               <<",\"height\":"<<s.height<<",\"verticalSpeed\":"<<s.verticalSpeed
               <<",\"grounded\":"<<(s.grounded?"true":"false")<<"}";
        }
        out<<']';
    }
    out<<'}';
    const std::string text=out.str();
    EM_ASM({
        const detail=JSON.parse(UTF8ToString($0));
        const canvas=document.getElementById('canvas');
        canvas.dataset.chargeLevel=String(detail.chargeLevel);
        canvas.dataset.boostRemaining=String(detail.boostRemaining);
        canvas.dataset.boostTotalDuration=String(detail.boostTotalDuration);
        canvas.dataset.boostAttackDuration=String(detail.boostAttackDuration);
        canvas.dataset.boostHoldDuration=String(detail.boostHoldDuration);
        canvas.dataset.boostReturnDuration=String(detail.boostReturnDuration);
        canvas.dataset.boostEnvelope=String(detail.boostEnvelope);
        canvas.dataset.boostBonus=String(detail.boostBonus);
        canvas.dataset.normalSpeed=String(detail.normalSpeed);
        canvas.dataset.yawRate=String(detail.yawRate);
        canvas.dataset.bodyRoll=String(detail.bodyRoll);
        canvas.dataset.bodyPitch=String(detail.bodyPitch);
        canvas.dataset.bodyHeave=String(detail.bodyHeave);
        canvas.dataset.grounded=String(detail.grounded);
        canvas.dataset.surface=detail.surface;
        canvas.dataset.groundHeight=String(detail.groundHeight);
        canvas.dataset.verticalSpeed=String(detail.verticalSpeed);
        canvas.dataset.airHeight=String(detail.airHeight);canvas.dataset.airTime=String(detail.airTime);
        canvas.dataset.landingSerial=String(detail.landingSerial);canvas.dataset.landingImpact=String(detail.landingImpact);
        canvas.dataset.landingCompression=String(detail.landingCompression);
        canvas.dataset.maxAirHeight=String(detail.maxAirHeight);canvas.dataset.totalAirTime=String(detail.totalAirTime);
        canvas.dataset.maxTakeoffRise=String(detail.maxTakeoffRise);canvas.dataset.cameraEyeY=String(detail.cameraEyeY);
        canvas.dataset.acceleration=String(detail.acceleration);
        canvas.dataset.tireMarkCount=String(detail.tireMarkCount);canvas.dataset.smokeCount=String(detail.smokeCount);
        canvas.dataset.speedCueAmount=String(detail.speedCueAmount);
        canvas.dataset.engineSampleLoaded=String(detail.engineSampleLoaded);
        canvas.dataset.tireSampleLoaded=String(detail.tireSampleLoaded);
        canvas.dataset.boostBlur=String(detail.boostBlur);
        canvas.dataset.postAvailable=String(detail.postAvailable);
        canvas.dataset.boostLevel=String(detail.boostLevel);
        canvas.dataset.handlingModel=String(detail.handlingModel);
        canvas.dataset.freePractice=String(detail.freePractice);
        canvas.dataset.courseId=detail.courseId;
        canvas.dataset.stageScale=String(detail.stageScale);
        canvas.dataset.trackHalfWidth=String(detail.trackHalfWidth);
        canvas.dataset.trackLength=String(detail.trackLength);
        canvas.dataset.carX=String(detail.carX);canvas.dataset.carY=String(detail.carY);canvas.dataset.carZ=String(detail.carZ);
        canvas.dataset.releaseUsesCameraHeading=String(detail.releaseUsesCameraHeading);
        canvas.dataset.releaseHeading=String(detail.releaseHeading);
        canvas.dataset.releaseElapsed=String(detail.releaseElapsed);
        canvas.dataset.cameraFov=String(detail.cameraFov);
        canvas.dataset.cameraHeading=String(detail.cameraHeading);
        canvas.dataset.cameraLookHeading=String(detail.cameraLookHeading);
        canvas.dataset.cameraBoostPreview=String(detail.cameraBoostPreview);
        canvas.dataset.cameraSteerPreview=String(detail.cameraSteerPreview);
        canvas.dataset.cameraCharge=String(detail.cameraCharge);
        canvas.dataset.cameraBoostKick=String(detail.cameraBoostKick);
        canvas.dataset.cameraFovTail=String(detail.cameraFovTail);
        canvas.dataset.cameraSetback=String(detail.cameraSetback);
        canvas.dataset.cameraShake=String(detail.cameraShake);
        canvas.dataset.cameraChargeFov=String(detail.cameraChargeFov);
        canvas.dataset.cameraBoostFov=String(detail.cameraBoostFov);
        canvas.dataset.cameraPhase=String(detail.cameraPhase);
        canvas.dataset.cameraPhaseElapsed=String(detail.cameraPhaseElapsed);
        canvas.dataset.carYaw=String(detail.carYaw);
        canvas.dataset.travelYaw=String(detail.travelYaw);
        canvas.dataset.boostVisual=String(detail.boostVisual);
        canvas.dataset.driftHeld=String(detail.driftHeld);
        canvas.dataset.driftHoldActive=String(detail.driftHoldActive);
        canvas.dataset.driftPhase=String(detail.driftPhase);
        canvas.dataset.appliedSettings=JSON.stringify(detail.settings);
        canvas.dataset.configRevision=String(detail.configRevision);
        canvas.dataset.liveTuningChanged=String(detail.liveTuningChanged);
        canvas.dataset.runId=String(detail.runId);
        canvas.dataset.frame=String(detail.frame);
        canvas.dataset.speed=String(detail.speed);
        window.dispatchEvent(new CustomEvent('driftlab-state',{detail}));
    },text.c_str());
#else
    (void)app; (void)samples;
#endif
}

Mat4 bodyMatrix(const App& app) {
    return mul(translation(0,0.55f+app.vehicle.heave+app.vehicle.landingCompression,0),
               mul(carModelMatrix(app.car,app.car.terrainPitch+app.vehicle.pitch,app.car.terrainRoll+app.vehicle.roll),translation(0,-0.55f,0)));
}

void tickEffects(App& app,float dt) {
    for(auto& mark:app.tireMarks) mark.age+=dt;
    while(!app.tireMarks.empty() && (app.tireMarks.front().age>12 || app.tireMarks.size()>3200)) app.tireMarks.pop_front();
    for(auto& puff:app.smoke) {puff.age+=dt; puff.position.y+=dt*0.55f;}
    app.smoke.erase(std::remove_if(app.smoke.begin(),app.smoke.end(),[](const Smoke& p){return p.age>=0.85f;}),app.smoke.end());
    if(!app.car.grounded) {app.tiresReady=false;app.smokeAccumulator=0;return;}
    float strength=slipFeedback(app.car.speed,app.car.slipAngle,app.car.onAsphalt);
    const bool marking=strength>0.02f;
    if(!marking && !app.tiresReady) {app.smokeAccumulator=0; return;}
    Vec3 forward{std::sin(app.car.yaw),0,std::cos(app.car.yaw)};
    Vec3 right{std::cos(app.car.yaw),0,-std::sin(app.car.yaw)};
    Vec3 tires[4];
    for(int axle=0;axle<2;++axle) for(int side=0;side<2;++side) {
        const int i=axle*2+side; // front pair first, rear pair second
        Vec3 p=Vec3{app.car.x,app.car.y,app.car.z}+
               forward*(axle==0?1.42f:-1.38f)+right*(side==0?-0.82f:0.82f);
        auto pose=app.car.querySupport(app.track,p.x,p.z);
        p.y=pose.centerY+clampf(pose.lateral,-app.track.halfWidth-app.track.curbWidth,app.track.halfWidth+app.track.curbWidth)*pose.cross+0.035f;
        tires[i]=p;
        if(app.tiresReady && length(p-app.lastTire[i])<2.5f) app.tireMarks.push_back({app.lastTire[i],p,0});
        app.lastTire[i]=p;
    }
    // Close the active tracks at the release-frame contacts before breaking the chain.
    app.tiresReady=marking;
    if(!marking) {app.smokeAccumulator=0; return;}
    app.smokeAccumulator+=dt*20.0f*strength*app.settings.smokeScale;
    while(app.smokeAccumulator>=1) {
        app.smokeAccumulator-=1;
        for(int i=2;i<4;++i) app.smoke.push_back({tires[i]+Vec3{0,0.16f,0},0,strength});
    }
}

void updateCamera(App& app,float dt) {
    app.camera.update(dt,app.car,app.settings,app.started&&!app.paused&&!app.ended);
    auto ground=app.car.querySupport(app.track,app.camera.eye.x,app.camera.eye.z);
    app.camera.eye.y=std::max(app.camera.eye.y,ground.centerY+0.8f);
}

void drawEffects(App& app) {
    CpuMesh marks;
    for(const auto& mark:app.tireMarks) {
        Vec3 d=mark.b-mark.a;
        float size=std::sqrt(d.x*d.x+d.z*d.z); if(size<0.015f) continue;
        Vec3 width{d.z/size*0.13f,0,-d.x/size*0.13f};
        float fade=clampf(mark.age/12,0,1);
        Vec3 color{0.045f+fade*0.14f,0.05f+fade*0.14f,0.055f+fade*0.14f};
        marks.addQuad(mark.a-width,mark.a+width,mark.b+width,mark.b-width,{0,1,0},color);
    }
    app.renderer.updateMesh(app.tireMesh,marks);
    app.renderer.draw(app.tireMesh,Mat4{});
}

void drawSmoke(App& app) {
    // Stable back-to-front order, soft radial alpha, depth-tested but no depth writes.
    std::vector<const Smoke*> order;
    for(const auto& puff:app.smoke) order.push_back(&puff);
    std::sort(order.begin(),order.end(),[&](const Smoke* a,const Smoke* b){return dot(a->position-app.camera.eye,a->position-app.camera.eye)>dot(b->position-app.camera.eye,b->position-app.camera.eye);});
    Vec3 right{std::cos(app.camera.heading),0,-std::sin(app.camera.heading)};
    for(const auto* puff:order) {
        float life=puff->age/0.85f;
        float radius=0.25f+1.1f*life;
        Mat4 model=basis(puff->position,right*radius,{0,radius,0},{0,0,1});
        float opacity=0.17f*puff->strength*(1-life)*(1-life*0.3f);
        app.renderer.drawSmoke(app.smokeMesh,model,opacity);
    }
}

void drawBoostJets(App& app,const Mat4& bodyModel) {
    if(app.ended || !app.started || app.car.boostRemaining<=0 || app.car.boostLevel<=0) return;
    const float strength=std::max(app.camera.boostAmount,clampf(app.car.boostSpeedBonus/18.0f,0,1));
    if(strength<=0.015f) return;
    const float flicker=0.94f+0.06f*std::sin(app.frameNumber*1.3f);
    const float width=0.12f+0.06f*strength;
    const float jetLength=(0.35f+0.65f*strength)*flicker;
    for(int side=-1;side<=1;side+=2) {
        const Mat4 model=mul(bodyModel,basis({side*0.53f,0.40f,-2.16f},{width,0,0},{0,width,0},{0,0,jetLength}));
        app.renderer.drawGlow(app.boostJet,model,strength*0.82f);
    }
}

void drawSpeedCues(App& app) {
    app.speedCueAmount=0;
    if(!app.started||app.paused||app.ended||app.settings.speedCueScale<=0) return;
    auto& h=app.renderer;
    const float amount=clampf((std::fabs(app.car.speed)*3.6f-60)/100,0,1);
    if(amount<=0) return;
    app.speedCueAmount=amount*app.settings.speedCueScale;
    const float u=clampf(h.width/1280.0f,0.6f,1.6f);
    const float tangent=std::tan(app.camera.fovDegrees/kDegrees*0.5f);
    const float yawDelta=clampf(wrapPi(app.car.travelYaw-app.camera.lookHeading),-1.1f,1.1f);
    const float focusX=clampf(0.5f+std::tan(yawDelta)/(2*tangent*float(h.width)/std::max(1,h.height)),0.2f,0.8f);
    const Vec3 ray=app.camera.look-app.camera.eye;
    const float planarRay=std::sqrt(ray.x*ray.x+ray.z*ray.z);
    const float focusY=clampf(0.5f+ray.y/std::fmax(planarRay,0.1f)/(2*tangent),0.18f,0.55f);
    const float cx=h.width*focusX,cy=h.height*focusY;
    for(int i=0;i<8;++i) {
        const float ex=h.width*(i<4?0.035f:0.965f);
        const float ey=h.height*(0.16f+0.23f*(i%4));
        const float dx=ex-cx,dy=ey-cy,length=std::sqrt(dx*dx+dy*dy);
        const float phase=std::fmod(app.trial.metrics.duration*(0.8f+1.6f*amount)+i*0.618034f,1.0f);
        const float radius=0.82f+phase*0.36f;
        const float px=cx+dx*radius,py=cy+dy*radius;
        const float streak=(16+58*amount)*u;
        const float envelope=std::sin(phase*3.14159265f);
        const float alpha=(0.11f*amount+0.10f*app.car.boostEnvelopeAmount())*envelope*app.settings.speedCueScale;
        h.drawLine(px-dx/length*streak,py-dy/length*streak,px,py,1.1f*u,0.87f,0.94f,1,clampf(alpha,0,0.35f));
    }
}

void drawItems(App& app,Vec3 eye) {
    CpuMesh solid,glow;
    app.items.buildMeshes(solid,glow,eye);
    app.renderer.updateMesh(app.itemSolid,solid);
    app.renderer.updateMesh(app.itemGlow,glow);
    app.renderer.draw(app.itemSolid,Mat4{});
    app.renderer.drawGlow(app.itemGlow,Mat4{},1);
}

void drawDriftEffects(App& app,Vec3 eye) {
    app.fx.forEachFlash([&](int level,const Mat4& model,float opacity){
        app.renderer.drawGlow(app.chargeHalo[std::clamp(level,1,3)-1],model,opacity,true);
    });
    CpuMesh glow;
    app.fx.buildGlow(glow,eye);
    app.renderer.updateMesh(app.fxMesh,glow);
    app.renderer.drawGlow(app.fxMesh,Mat4{},1);
}

void drawGroundShadow(App& app) {
    const Vec3 forward{std::sin(app.car.yaw),0,std::cos(app.car.yaw)};
    const Vec3 right{std::cos(app.car.yaw),0,-std::sin(app.car.yaw)};
    Vec3 corners[4];
    const float along[]={-2.15f,-2.15f,2.15f,2.15f};
    const float across[]={-1.0f,1.0f,1.0f,-1.0f};
    for(int i=0;i<4;++i) {
        Vec3 p=Vec3{app.car.x,0,app.car.z}+forward*along[i]+right*across[i];
        const auto floor=app.car.querySupport(app.track,p.x,p.z);
        p.y=floor.found?floor.centerY+clampf(floor.lateral,-app.track.halfWidth-app.track.curbWidth,app.track.halfWidth+app.track.curbWidth)*floor.cross:app.car.groundHeight;
        corners[i]=p+Vec3{0,0.035f,0};
    }
    const float fade=clampf((app.car.y-app.car.groundHeight)/10,0,1);
    const Vec3 color{0.03f+fade*0.09f,0.035f+fade*0.10f,0.04f+fade*0.11f};
    CpuMesh mesh;mesh.addQuad(corners[0],corners[1],corners[2],corners[3],{0,1,0},color);
    app.renderer.updateMesh(app.shadow,mesh);app.renderer.draw(app.shadow,Mat4{});
}

void drawHud(App& app) {
    auto& h=app.renderer; h.beginHud();
    drawSpeedCues(app);
    float u=clampf(h.width/1280.0f,0.6f,1.6f);
    char line[128];
    h.addQuad(16*u,16*u,270*u,83*u,0.015f,0.025f,0.04f,0.62f);
    std::snprintf(line,sizeof(line),"%03d KM/H",int(std::lround(app.car.speed*3.6f)));
    h.drawText(28*u,28*u,3.2f*u,line,1,1,1,1);
    const char* state="GRIP";
    if(app.car.driftHoldActive) state="DRIFT";
    if(app.car.boostRemaining>0&&app.car.boostLevel>0) state="BOOST";
    if(!app.car.grounded) state="AIRBORNE";
    if(app.paused) state="PAUSED";
    h.drawText(float(h.width)-245*u,25*u,2.6f*u,state,0.95f,0.95f,0.95f,1);
    if(app.started) {
        const float thresholds[]={0,0.65f,1.30f,2.20f};
        const Vec3 colors[]={{0.08f,0.48f,1},{1,0.34f,0.035f},{1,0.12f,0.66f}};
        const float left=h.width*0.5f-96*u,y=h.height-34*u;
        for(int i=0;i<3;++i) {
            const float x=left+i*66*u;
            const float fill=clampf((app.car.driftCharge-thresholds[i])/(thresholds[i+1]-thresholds[i]),0,1);
            h.addQuad(x-2*u,y-2*u,x+60*u,y+8*u,0.02f,0.03f,0.04f,0.7f);
            h.addQuad(x,y,x+58*u*fill,y+6*u,colors[i].x,colors[i].y,colors[i].z,0.9f);
        }
    }
    h.flushHud();
}

void rememberRun(App& app) {
    if(!app.replay) {
        if(app.recordingTruncated||app.trial.samplesTruncated||app.liveTuningChanged) app.lastRecording.clear();
        else app.lastRecording=app.currentRecording;
        app.recordingTuningChanged=app.liveTuningChanged;
        app.recordingSettings=app.settings;
        app.recordingModel=app.car.driftModel;
        app.recordingGroup=int(app.group); app.recordingCandidate=app.candidate; app.recordingRunId=app.runId;
    }
}

void finishRun(App& app) {
    app.ended=true;
    app.camera.clearBoostCue();
    rememberRun(app);
    publishState(app,true);
}

void frame(void* userdata) {
    App& app=*static_cast<App*>(userdata);
    SDL_Event event;
    int command=0, modelCommand=-1;
    bool pauseOnFirstStart=false;
    while(SDL_PollEvent(&event)) {
        if(event.type==SDL_QUIT) app.running=false;
        else if(event.type==SDL_KEYDOWN && !event.key.repeat) {
            auto code=event.key.keysym.scancode;
#ifdef __EMSCRIPTEN__
            if(!EM_ASM_INT({return document.activeElement===document.getElementById('canvas');})) continue;
#endif
            if(code==SDL_SCANCODE_R) command=1;
            else if(code==SDL_SCANCODE_P) command=3;
            else if(code==SDL_SCANCODE_1 || code==SDL_SCANCODE_2) modelCommand=code==SDL_SCANCODE_2?1:0;
            else if(code==SDL_SCANCODE_M) {
                app.muted=!app.muted;
#ifdef __EMSCRIPTEN__
                EM_ASM({window.driftLab.muted=!!$0;},app.muted);
#endif
            }
#ifndef __EMSCRIPTEN__
            else if(code==SDL_SCANCODE_ESCAPE) app.running=false;
#endif
        } else if(event.type==SDL_CONTROLLERDEVICEADDED) openPad(app);
        else if(event.type==SDL_CONTROLLERDEVICEREMOVED && app.pad && event.cdevice.which==SDL_JoystickInstanceID(SDL_GameControllerGetJoystick(app.pad))) {
            SDL_GameControllerClose(app.pad); app.pad=nullptr;
        } else if(event.type==SDL_CONTROLLERBUTTONDOWN) {
            if(event.cbutton.button==SDL_CONTROLLER_BUTTON_BACK) command=1;
            else if(event.cbutton.button==SDL_CONTROLLER_BUTTON_START) command=3;
        }
    }
    PadPulse pulse;
    DriveInput input=readDrive(app.pad,pulse);
    if(pulse.reset) command=1;
    if(pulse.pause) command=3;
#ifdef __EMSCRIPTEN__
    bool started=EM_ASM_INT({return !!(window.driftLab&&window.driftLab.started);});
    app.muted=EM_ASM_INT({return !!(window.driftLab&&window.driftLab.muted);});
    int webCommand=EM_ASM_INT({var lab=window.driftLab;if(!lab)return 0;var c=lab.command|0;lab.command=0;return c;});
    if(webCommand) command=webCommand;
    bool hidden=EM_ASM_INT({return document.hidden?1:0;});
    if(hidden && app.started && !app.ended && !app.paused) {
        app.paused=true; app.accumulator=0; app.stateTime=1;
        rememberRun(app); publishState(app,true);
    }
    if(started && !app.started) {
        app.started=true;
        pauseOnFirstStart=command==4||hidden||!EM_ASM_INT({return document.hasFocus()?1:0;});
        command=1;
    }
#endif
    if(modelCommand>=0) {
        app.selectedModel=modelCommand==1?DriftModel::LineControl:DriftModel::Inertia;
        app.group=FeelGroup::Handling; app.candidate=modelCommand;
#ifdef __EMSCRIPTEN__
        EM_ASM({
            const lab=window.driftLab;
            lab.group=0;lab.candidate=$0;lab.handlingModel=$0;
            window.dispatchEvent(new CustomEvent('practice-model-selected',{detail:{model:$0}}));
        },modelCommand);
#endif
        command=1;
    }
    if(command==1) {
        resetTrial(app,false);
        if(pauseOnFirstStart) {
            app.paused=true;
            publishState(app,true);
        }
    }
    else if(command==2) {
        readSettings(app);
        if(!app.lastRecording.empty() && !app.recordingTuningChanged && app.group!=FeelGroup::Handling && app.group!=FeelGroup::Combined) resetTrial(app,true);
    }
    else if((command==3||command==4||command==5) && app.started && !app.ended) {
        app.paused=command==4?true:command==5?false:!app.paused;app.accumulator=0;app.stateTime=1;
        if(app.paused) {rememberRun(app);publishState(app,true);}
    }

    const bool tuningApplied=applyLiveTuning(app);
    app.vehicle.update(0,app.car,app.settings,false);
    if(tuningApplied&&(app.paused||app.ended)&&app.started) rememberRun(app);

    Uint64 now=SDL_GetPerformanceCounter();
    double elapsed=double(now-app.prevCounter)/double(SDL_GetPerformanceFrequency());
    app.prevCounter=now;
    elapsed=std::max(0.0,std::min(0.1,elapsed));
    if(command==1 || command==2) elapsed=0;
    app.currentImpact=app.currentLandingImpact=app.currentAcceleration=0;
    if(app.paused || !app.started || app.ended) app.accumulator=0;
    else {
        app.accumulator+=elapsed;
        while(app.accumulator>=kFixedStepSec && !app.ended) {
            DriveInput drive=input;
            if(app.replay) {
                if(app.replayFrame>=app.lastRecording.size()) {finishRun(app);break;}
                drive=app.lastRecording[app.replayFrame++];
            } else {
                // Capture the last displayed optical direction only on a release.
                // Store it with the controls so camera comparison replays cannot
                // change the vehicle's route or repeatedly steer an active boost.
                if(app.car.handbrakeHeld && drive.handbrake<=0.2f && app.car.driftHoldActive &&
                   app.car.driftChargeLevel>0 && app.camera.ready && std::isfinite(app.camera.lookHeading)) {
                    drive.hasBoostHeading=true;drive.boostHeading=wrapPi(app.camera.lookHeading);
                }
                if(!app.freeDriving() || app.currentRecording.size()<PracticeSession::maxSamples)
                    app.currentRecording.push_back(drive);
                else app.recordingTruncated=true;
            }
            // Equal entry speed is part of the test, even when W is held on the approach.
            bool approach=!app.freeDriving() && !app.entryReached && !app.car.drifting && app.car.boostRemaining<=0;
            if(approach && !app.car.drifting && app.car.boostRemaining<=0) app.car.speed=TrialCourse::entrySpeedKmh/3.6f;
            const bool wasGrounded=app.car.grounded;
            const float previousY=app.car.y,previousSpeed=app.car.speed;
            app.car.update(float(kFixedStepSec),drive,app.track);
            if(wasGrounded&&!app.car.grounded) app.takeoffY=previousY;
            if(wasGrounded&&app.car.grounded&&(drive.throttle>0.05f||app.car.boostRemaining>0))
                app.currentAcceleration=std::max(app.currentAcceleration,(std::fabs(app.car.speed)-std::fabs(previousSpeed))/float(kFixedStepSec));
            app.vehicle.update(float(kFixedStepSec),app.car,app.settings,true,drive.brake);
            if(approach && !app.car.drifting && app.car.boostRemaining<=0) app.car.speed=TrialCourse::entrySpeedKmh/3.6f;
            if(app.car.ground.s>=TrialCourse::entryS || app.car.drifting || app.car.boostRemaining>0) app.entryReached=true;
            app.currentImpact=std::max(app.currentImpact,app.car.impact);
            app.currentLandingImpact=std::max(app.currentLandingImpact,app.car.landingImpact);
            if(!app.car.grounded) {
                app.totalAirTime+=float(kFixedStepSec);
                app.maxAirHeight=std::max(app.maxAirHeight,std::fmax(app.car.y-app.car.groundHeight,0.0f));
                app.maxTakeoffRise=std::max(app.maxTakeoffRise,std::fmax(app.car.y-app.takeoffY,0.0f));
            }
            if(app.freeDriving()) app.trial.tickPractice(app.car,drive,float(kFixedStepSec));
            else app.trial.tick(app.car,drive,float(kFixedStepSec));
            tickEffects(app,float(kFixedStepSec));
            for(const auto& pickup:app.items.update(float(kFixedStepSec),app.car)) {
                app.fx.pickupBurst(pickup.position,pickup.level,pickup.charged);
                app.audio.pickup(pickup.step);
            }
            app.fx.tick(float(kFixedStepSec),app.car,bodyMatrix(app),app.camera.eye);
            ++app.frameNumber;
            app.accumulator-=kFixedStepSec;
            if((!app.freeDriving() && app.trial.metrics.finished && app.car.boostRemaining<=0) ||
               (app.replay && app.replayFrame>=app.lastRecording.size())) finishRun(app);
        }
    }
    app.audio.setMuted(app.muted);
    app.audio.setGain(app.settings.audioScale);
    app.audio.setLayerGains(app.settings.engineGain,app.settings.tireGain);
    app.audio.setWindGain(app.settings.windGain);
    app.audio.setRevProfile(app.settings.engineRevScale,app.settings.engineRevReturn);
    if(app.ended) app.finishedAge+=float(elapsed);
    bool audioPaused=app.paused||!app.started||(app.ended&&app.finishedAge>0.5f);
    app.audio.update(app.ended?0:app.car.speed,app.ended?0:app.car.throttleLevel,
                     app.ended||!app.car.grounded?0:slipFeedback(app.car.speed,app.car.slipAngle,app.car.onAsphalt),app.currentImpact,audioPaused,
                     app.ended?0:std::max(std::fabs(app.car.speed)-app.car.boostSpeedBonus,0.0f),audioPaused?0:app.currentAcceleration);
    app.audio.setDriftState(app.car.driftChargeLevel,app.car.boostLevel,app.car.boostSerial,app.car.boostRemaining);
    app.audio.setLandingState(app.car.landingSerial,app.currentLandingImpact*app.settings.landingImpactScale);
    updateCamera(app,float(elapsed));
    int w=0,h=0; SDL_GL_GetDrawableSize(app.window,&w,&h);
    app.renderer.resize(w,h);
    const float blur=app.started&&!app.ended&&app.car.boostRemaining>0 && app.car.speed>1 && app.settings.boostSpeedScale>0
        ? app.car.boostEnvelopeAmount()*app.settings.boostBlurStrength : 0;
    app.renderer.clearFrame(blur); app.renderer.drawSky();
    float fov=app.camera.fovDegrees/kDegrees;
    Mat4 proj=perspective(fov,float(w)/float(std::max(1,h)),0.15f,4000);
    const Vec3 shake=app.paused?Vec3{}:app.fx.shakeOffset();
    const Vec3 eye=app.camera.eye+shake,look=app.camera.look+shake*0.35f;
    app.renderer.beginWorld(mul(proj,lookAt(eye,look,{0,1,0})),eye);
    app.renderer.draw(app.course,Mat4{}); drawEffects(app);
    drawGroundShadow(app);
    drawItems(app,eye);
    Mat4 model=mul(translation(0,app.car.grounded?app.car.groundHeight-app.car.y:0,0),carModelMatrix(app.car,app.car.terrainPitch,app.car.terrainRoll));
    const Mat4 bodyModel=bodyMatrix(app);
    app.renderer.draw(app.body,bodyModel);
    for(int axle=0;axle<2;++axle) for(int side=-1;side<=1;side+=2) {
        Mat4 wheel=mul(translation(side*0.82f,0.34f,axle==0?1.42f:-1.38f),mul(rotationY(axle==0?app.car.steerAngle:0),rotationX(app.car.wheelSpin)));
        app.renderer.draw(app.wheel,mul(model,wheel));
        int level=app.car.boostRemaining>0?0:app.car.driftChargeLevel;
        if(level>0 && level<=3) {
            float pulse=0.75f+0.1f*std::sin(app.frameNumber*0.07f);
            app.renderer.drawGlow(app.chargeWheel[level-1],mul(model,wheel),pulse);
            Vec3 position=Vec3{app.car.x,app.car.y+0.35f,app.car.z}+
                Vec3{std::cos(app.car.yaw),0,-std::sin(app.car.yaw)}*(side*0.85f)+
                Vec3{std::sin(app.car.yaw),0,std::cos(app.car.yaw)}*(axle==0?1.42f:-1.38f);
            Vec3 right{std::cos(app.camera.heading)*0.58f,0,-std::sin(app.camera.heading)*0.58f};
            app.renderer.drawGlow(app.chargeHalo[level-1],basis(position,right,{0,0.58f,0},{0,0,1}),0.22f,true);
        }
    }
    drawBoostJets(app,bodyModel); drawSmoke(app); drawDriftEffects(app,eye); app.renderer.finishWorld(); drawHud(app); SDL_GL_SwapWindow(app.window);
    app.stateTime+=float(elapsed);
    if(tuningApplied||app.stateTime>=0.1f) {publishState(app);app.stateTime=0;}
}
int selfTest(const Track& track) {
    Car car;
    car.reset(track);
    float travelled = 0;
    float prevS = -1;
    float maxLat = 0;
    for (int step = 0; step < 60 * 50; ++step) {
        TrackPose pose = track.query(car.x, car.z, car.y, car.ground.found ? car.ground.s : -1.0f);
        float lookS = pose.s + 70.0f;
        if (lookS >= track.length) {
            lookS -= track.length;
        }
        int nearest = 0;
        float nearestGap = 1.0e9f;
        for (int i = 0; i < static_cast<int>(track.samples.size()); i += 2) {
            float gap = std::fabs(track.samples[i].s - lookS);
            if (gap > track.length * 0.5f) {
                gap = track.length - gap;
            }
            if (gap < nearestGap) {
                nearestGap = gap;
                nearest = i;
            }
        }
        int next = (nearest + 1) % static_cast<int>(track.samples.size());
        float tx = track.samples[next].x - track.samples[nearest].x;
        float tz = track.samples[next].z - track.samples[nearest].z;
        float targetYaw = std::atan2(tx, tz);
        float errNow = wrapPi(pose.yaw - car.yaw);
        float errAhead = wrapPi(targetYaw - car.yaw);
        float bend = std::fabs(wrapPi(targetYaw - pose.yaw));
        DriveInput input;
        input.steer = clampf(errNow * 2.6f + errAhead * 0.5f, -1.0f, 1.0f);
        input.steer = clampf(input.steer - pose.lateral * 0.12f, -1.0f, 1.0f);
        input.throttle = bend < 0.15f ? 1.0f : 0.45f;
        input.brake = 0;
        if (bend > 0.28f && car.speed > 22.0f) {
            input.throttle = 0;
            input.brake = 1.0f;
        } else if (bend > 0.12f && car.speed > 34.0f) {
            input.throttle = 0;
            input.brake = 0.55f;
        }
        if (car.speed < 10.0f) {
            input.brake = 0;
            input.throttle = std::max(input.throttle, 0.5f);
        }
        car.update(1.0f / 60.0f, input, track);
        pose = track.query(car.x, car.z, car.y, car.ground.found ? car.ground.s : -1.0f);
        if (prevS >= 0) {
            float ds = pose.s - prevS;
            if (ds < -track.length * 0.5f) ds += track.length;
            if (ds > track.length * 0.5f) ds -= track.length;
            if (ds > 0) travelled += ds;
        }
        prevS = pose.s;
        maxLat = std::max(maxLat, pose.xzDist);
        if (!std::isfinite(car.x) || !std::isfinite(car.speed) || !std::isfinite(car.yaw)) {
            std::printf("self-test non-finite\n");
            return 1;
        }
    }
    std::printf("self-test length %.0f travelled %.0f maxLat %.1f speed %.1f km/h\n",
                track.length, travelled, maxLat, car.speed * 3.6f);
    if (track.length < 5200.0f || track.length > 6500.0f) return 2;
    if (travelled < 1200.0f) return 3;
    if (maxLat > 30.0f) return 4;
    return 0;
}

void reportStartupFailure(const char* message) {
#ifdef __EMSCRIPTEN__
    EM_ASM({
        window.dispatchEvent(new CustomEvent('driftgame-error',{detail:{message:UTF8ToString($0)}}));
    },message);
#else
    (void)message;
#endif
}

bool initGl(App& app) {
#ifdef __EMSCRIPTEN__
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
#else
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
#endif
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    app.window = SDL_CreateWindow("DRIFT", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1280, 720,
                                  SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_SHOWN);
    if (!app.window) {
        SDL_Log("SDL_CreateWindow failed: %s", SDL_GetError());
        return false;
    }
    app.gl = SDL_GL_CreateContext(app.window);
    if (!app.gl) {
        SDL_Log("SDL_GL_CreateContext failed: %s", SDL_GetError());
        return false;
    }
    SDL_GL_SetSwapInterval(1);
    if (!app.renderer.init()) {
        SDL_Log("GL init failed");
        return false;
    }
    return true;
}


} // namespace

int main(int argc,char** argv) {
    if(argc>1 && std::strcmp(argv[1],"--self-test")==0) {
        Track track;
        if(!track.load(findTrackPath())) return 1;
        return selfTest(track);
    }
    if(SDL_Init(SDL_INIT_VIDEO|SDL_INIT_AUDIO|SDL_INIT_GAMECONTROLLER)!=0) {
        std::fprintf(stderr,"SDL init failed: %s\n",SDL_GetError());
        reportStartupFailure("ゲームを開始できませんでした。読み込み直して、もう一度お試しください。");
        return 1;
    }
    bool imageOk=(IMG_Init(IMG_INIT_PNG)&IMG_INIT_PNG)!=0;
    bool audioOk=Mix_OpenAudio(44100,MIX_DEFAULT_FORMAT,2,1024)==0;
    App app;
#ifdef __EMSCRIPTEN__
    app.circuit=EM_ASM_INT({return new URLSearchParams(window.location.search).get('course')!=='practice';});
#else
    app.circuit=argc>1 && std::strcmp(argv[1],"--suzuka")==0;
#endif
    if(app.circuit) {
        if(!app.track.load(findTrackPath())) {
            SDL_Log("Failed to load Suzuka course");
            SDL_Quit();
            reportStartupFailure("コースを読み込めませんでした。読み込み直して、もう一度お試しください。");
            return 1;
        }
    } else app.track.makePracticeArea();
    if(!initGl(app)) {
        SDL_Quit();
        reportStartupFailure("3D画面を開始できませんでした。別のブラウザでお試しください。");
        return 1;
    }
    openPad(app);
    if(audioOk) audioOk=app.audio.init();
    if(!audioOk) SDL_Log("Audio unavailable: %s",Mix_GetError());
    app.course=app.renderer.upload(app.track.visual);
    app.items.build(app.track);
    CpuMesh body,wheel,shadow,smoke,jet;
    buildCarBody(body);buildCarWheel(wheel);buildCarShadow(shadow);
    smoke.addQuad({-1,-1,0},{1,-1,0},{1,1,0},{-1,1,0},{0,0,-1},{0.72f,0.74f,0.76f});
    app.body=app.renderer.upload(body);app.wheel=app.renderer.upload(wheel);app.shadow=app.renderer.upload(shadow);app.smokeMesh=app.renderer.upload(smoke);
    jet.addQuad({-1,0,0},{1,0,0},{0.12f,0,-1},{-0.12f,0,-1},{0,1,0},{1,0.30f,0.035f});
    jet.addQuad({0,-1,0},{0,1,0},{0,0.12f,-1},{0,-0.12f,-1},{1,0,0},{1,0.30f,0.035f});
    jet.addQuad({-0.45f,0,0},{0.45f,0,0},{0.04f,0,-0.55f},{-0.04f,0,-0.55f},{0,1,0},{0.35f,0.7f,1});
    jet.addQuad({0,-0.45f,0},{0,0.45f,0},{0,0.04f,-0.55f},{0,-0.04f,-0.55f},{1,0,0},{0.35f,0.7f,1});
    app.boostJet=app.renderer.upload(jet);
    const Vec3 colors[3]={{0.08f,0.48f,1.0f},{1.0f,0.34f,0.035f},{1.0f,0.12f,0.66f}};
    for(int level=0;level<3;++level) {
        CpuMesh rings,halo;
        for(int side=-1;side<=1;side+=2) for(int segment=0;segment<32;++segment) {
            float a=segment*6.2831853f/32,b=(segment+1)*6.2831853f/32;
            float x=side*0.155f;
            rings.addQuad({x,std::cos(a)*0.355f,std::sin(a)*0.355f},
                          {x,std::cos(b)*0.355f,std::sin(b)*0.355f},
                          {x,std::cos(b)*0.28f,std::sin(b)*0.28f},
                          {x,std::cos(a)*0.28f,std::sin(a)*0.28f},{float(side),0,0},colors[level]);
        }
        halo.addQuad({-1,-1,0},{1,-1,0},{1,1,0},{-1,1,0},{0,0,-1},colors[level]);
        app.chargeWheel[level]=app.renderer.upload(rings);
        app.chargeHalo[level]=app.renderer.upload(halo);
    }
#ifndef __EMSCRIPTEN__
    app.started=true;
#endif
    resetTrial(app);
    app.prevCounter=SDL_GetPerformanceCounter();
#ifdef __EMSCRIPTEN__
    emscripten_set_main_loop_arg(frame,&app,0,1);
#else
    while(app.running) {frame(&app);SDL_Delay(1);}
#endif
    app.audio.shutdown();
    app.renderer.shutdown();
    if(app.pad) SDL_GameControllerClose(app.pad);
    Mix_CloseAudio();Mix_Quit();if(imageOk) IMG_Quit();
    SDL_GL_DeleteContext(app.gl);SDL_DestroyWindow(app.window);SDL_Quit();
    return 0;
}
