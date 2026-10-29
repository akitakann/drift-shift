#pragma once

#include <atomic>
#include <cstdint>
#include <vector>

struct SDL_RWops;

// Recorded engine/tire loops with real-time pitch and envelopes. Original
// synthesis remains the fallback, and still provides charge/boost/impact cues.
// Call init after Mix_OpenAudio and shutdown before Mix_CloseAudio.
class DriftAudio {
public:
    bool init();
    // driveSpeed is the nonnegative drivetrain speed without the temporary boost
    // bonus. Omit it (or pass a negative value) to use abs(speed), as before.
    // acceleration is actual positive planar acceleration in m/s². The caller
    // supplies zero in air, on braking, and for a heading change at constant speed.
    void update(float speed, float throttle, float drift, float impact, bool paused, float driveSpeed = -1, float acceleration = 0);
    void setDriftState(int chargeLevel, int boostLevel, unsigned boostSerial, float boostRemaining);
    void bank(float quality);
    // Short synthesized jingles for the time attack. Safe to call every frame;
    // a full queue drops the newest cue rather than blocking.
    enum class Cue { CountTick, CountGo, Item, SectorBest, SectorSlower, Lap, Record };
    void cue(Cue cue, int step = 0);
    void setMuted(bool muted);
    void setGain(float gain);
    void setLayerGains(float engine, float tire);
    void setWindGain(float gain);
    void setRevProfile(float scale, float returnTime);
    // Call on the control thread, after update. The first observed serial is a
    // baseline; a new serial publishes one bounded impact value to the callback.
    void setLandingState(unsigned serial, float impact);
    bool engineSampleLoaded() const { return engineSampleLoaded_.load(std::memory_order_acquire); }
    bool tireSampleLoaded() const { return tireSampleLoaded_.load(std::memory_order_acquire); }
    void reset();
    void shutdown();

private:
    static void postMix(void* userdata, std::uint8_t* stream, int length);
    void render(std::uint8_t* stream, int length);
    bool loadLoop(SDL_RWops* source, std::vector<float>& pcm);
    void startCue(int type, int step);
    void addTone(float delay, float length, float hz, float gain, float bright);
    float renderTones(double dt);
    void loadSampleLoops(SDL_RWops* engineSource, SDL_RWops* tireSource);

    std::atomic<float> speed_{0};
    std::atomic<float> driveSpeed_{0};
    std::atomic<float> acceleration_{0};
    std::atomic<float> throttle_{0};
    std::atomic<float> drift_{0};
    std::atomic<float> impact_{0};
    std::atomic<float> bankQuality_{0};
    std::atomic<unsigned> bankSerial_{0};
    struct CueEvent { int type = 0; int step = 0; };
    static constexpr unsigned kCueQueue = 32;
    CueEvent cueQueue_[kCueQueue];
    std::atomic<unsigned> cueWrite_{0}, cueRead_{0};
    std::atomic<bool> paused_{false};
    std::atomic<bool> muted_{false};
    std::atomic<float> gain_{1};
    std::atomic<float> engineGain_{1}, tireGain_{1};
    // Standalone callers retain the previous mix unless they enable this bed.
    std::atomic<float> windGain_{0};
    std::atomic<float> engineRevScale_{0}, engineRevReturn_{0.65f};
    std::atomic<float> pendingLandingImpact_{0};
    std::atomic<bool> engineSampleLoaded_{false}, tireSampleLoaded_{false};
    std::atomic<unsigned> resetSerial_{0};
    std::atomic<int> chargeLevel_{0};
    std::atomic<int> boostLevel_{0};
    std::atomic<unsigned> boostSerial_{0};
    std::atomic<float> boostRemaining_{0};
    unsigned seenResetSerial_ = 0;
    bool ready_ = false;
    int sampleRate_ = 44100;
    int channels_ = 2;
    std::uint32_t lastImpactMs_ = 0;
    unsigned landingSerial_ = 0; // Control-thread event history, never read by the callback.
    bool landingSerialPrimed_ = false;
    // Loaded/converted before installing the callback, immutable until it has
    // been detached during shutdown. Decoding/allocation stays off the callback.
    std::vector<float> enginePcm_, tirePcm_;

    // Only the audio callback touches the following state.
    double engineSamplePosition_ = 0, tireSamplePosition_ = 0;
    float engineSampleRate_ = 0.75f, tireSampleRate_ = 1;
    float enginePlaybackRate_ = 0.75f, engineRevLevel_ = 0;
    float engineMixGain_ = 1, tireMixGain_ = 1;
    double enginePhase_ = 0;
    double tirePhase_ = 0;
    double cuePhase_ = 0;
    double boostPhase_ = 0;
    double boostAirPhase_ = 0;
    float pitch_ = 48;
    float engineLevel_ = 0;
    float driftLevel_ = 0;
    float gate_ = 0;
    float noiseLow_ = 0;
    float noiseHigh_ = 0;
    float crashEnvelope_ = 0;
    float bankTime_ = -1;
    float cueQuality_ = 0;
    unsigned seenBankSerial_ = 0;
    struct Tone { float delay = 0, age = 0, length = 0, hz = 0, gain = 0, bright = 0; double phase = 0; bool active = false; };
    static constexpr int kTones = 24;
    Tone tones_[kTones];
    int seenChargeLevel_ = 0;
    unsigned seenBoostSerial_ = 0;
    bool boostSerialPrimed_ = false;
    float chargeCueTime_ = -1;
    int chargeCueLevel_ = 0;
    float chargeAirLow_ = 0, chargeAirHigh_ = 0;
    std::uint32_t chargeRandom_ = 0x9e36b421;
    float boostCueTime_ = -1;
    float boostCueDuration_ = 0;
    int boostCueLevel_ = 0;
    float boostNoise_ = 0;
    float boostLowNoise_ = 0;
    float boostTail_ = 0;
    float boostVoiceGate_ = 0;
    float windLevel_ = 0, windLow_ = 0, windHigh_ = 0;
    std::uint32_t windRandom_ = 0x35ba17c9;
    double landingPhase_ = 0;
    float landingCueTime_ = -1, landingCueImpact_ = 0, landingLow_ = 0;
    std::uint32_t random_ = 0x8e4d6a39;
};
