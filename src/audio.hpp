#pragma once

#include <atomic>
#include <cstdint>
#include <vector>

struct SDL_RWops;

// Recorded engine/tire loops with real-time pitch and envelopes, and designed
// one-shot effects (assets/audio/sfx) for items, boost and the time attack.
// Charge, boost and item pushes are heard in the engine itself, which revs.
// Original synthesis remains the fallback for any file that is missing.
// Call init after Mix_OpenAudio and shutdown before Mix_CloseAudio.
class DriftAudio {
public:
    // How the engine answers charge, boost and item pushes. Pitches are
    // semitones on the engine's note, levels dB on the engine bed, files dB
    // on their own level (-30 or lower silences one).
    struct RevTuning {
        float chargeSemis[3] = {1.5f, 3, 5};   // Each held charge level revs the engine this much higher,
        float chargeDb[3] = {1, 2, 3.5f};      // and this much louder.
        float blipSemis = 2.5f, blipDb = 3, blipTime = 0.25f; // A level-up revs past its note and settles.
        float creepSemis = 0.8f;               // A held level keeps easing up, over a second or so.
        float boostSemis[3] = {5, 6.5f, 8};    // A release flares this far above the engine, then settles.
        float boostDb = 4, boostTime = 1;      // boostTime scales the level's base length.
        float itemSemis = 2, itemDb = 2.5f, itemTime = 0.3f; // An item's push.
        float boostWhooshDb = 0, itemWhooshDb = 0, pickupDb = 0, afterfireDb = -6;
    };
    DriftAudio() { setRevTuning(RevTuning{}); }
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
    // Safe to call every frame from the control thread.
    void setRevTuning(const RevTuning& tuning);
    // Call on the control thread, after update. The first observed serial is a
    // baseline; a new serial publishes one bounded impact value to the callback.
    void setLandingState(unsigned serial, float impact);
    bool engineSampleLoaded() const { return engineSampleLoaded_.load(std::memory_order_acquire); }
    // One-shot effects, in file order: item_0..7 (the pickup, one per chain
    // step), boost_1..3 (the afterfire for each release level), whoosh_item,
    // whoosh_boost, count_tick, count_go, sector_best, sector_slower, lap, record.
    enum Shot { kShotItem = 0, kShotBoost = 8, kShotWhooshItem = 11, kShotWhooshBoost, kShotCountTick,
                kShotCountGo, kShotSectorBest, kShotSectorSlower, kShotLap, kShotRecord, kShotCount };
    static const char* shotName(int shot);
    int shotsLoaded() const { return shotsLoaded_.load(std::memory_order_acquire); }
    bool tireSampleLoaded() const { return tireSampleLoaded_.load(std::memory_order_acquire); }
    void reset();
    void shutdown();

private:
    static void postMix(void* userdata, std::uint8_t* stream, int length);
    void render(std::uint8_t* stream, int length);
    bool loadLoop(SDL_RWops* source, std::vector<float>& pcm);
    void startCue(int type, int step, const RevTuning& tuning);
    void addTone(float delay, float length, float hz, float gain, float bright);
    float renderTones(double dt);
    void loadSampleLoops(SDL_RWops* engineSource, SDL_RWops* tireSource);
    void loadShots(const char* directory);
    bool startShot(int shot, bool boost = false, float gain = 1);

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
    std::vector<float> shots_[kShotCount]; // Interleaved at the mixer format; empty when missing.
    static constexpr int kRevFields = 22;
    std::atomic<float> revTuning_[kRevFields];
    RevTuning loadRevTuning() const;
    std::atomic<int> shotsLoaded_{0};

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
    // The engine's answer to charge, boost and items: an offset in semitones
    // and dB on the engine bed, smoothed, and the envelopes that drive it.
    float fxSemis_ = 0, fxDb_ = 0;
    int fxLevel_ = 0;          // The held charge level being heard.
    float fxLevelAge_ = 0;     // Seconds since the engine last went up a level.
    float fxBoostTime_ = -1, fxBoostFromSemis_ = 0, fxBoostFromDb_ = 0;
    int fxBoostLevel_ = 0;
    float fxItemTime_ = -1;
    struct Voice { int shot = -1; std::size_t frame = 0; bool boost = false; float gain = 1; };
    static constexpr int kVoices = 12;
    Voice voices_[kVoices];
    float shotBoostGate_ = 0;
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
