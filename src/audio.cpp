#include "audio.hpp"

#include <SDL.h>
#include <SDL_mixer.h>

#include <algorithm>
#include <cmath>
#include <string>

namespace {
constexpr double kTau = 6.283185307179586;
constexpr float kBoostVoiceDurations[] = {0, 0.35f, 0.50f, 0.65f};
constexpr float kChargeAirDurations[] = {0, 0.080f, 0.100f, 0.130f};
constexpr float kChargeAirGain = 0.10f;
// The engine's answer to a release lasts this long per level, scaled by
// RevTuning::boostTime; it rises in kFxRise and settles over the rest.
constexpr float kFxBoostDuration[] = {0.0f, 0.7f, 0.85f, 1.0f};
constexpr float kFxRise = 0.10f;
constexpr float kFxBoostWhoosh[] = {0.0f, 0.8f, 0.9f, 1.0f}; // Whoosh level per release level.
float semitones(float semis) { return std::exp2(semis / 12.0f); }
float decibels(float db) { return std::pow(10.0f, db / 20.0f); }
float smoothstep01(float t) {
    t = t < 0 ? 0 : (t > 1 ? 1 : t);
    return t * t * (3 - 2 * t);
}
float limit(float value, float low, float high) {
    if (!std::isfinite(value)) return low;
    return std::max(low, std::min(high, value));
}
double advance(double phase, double hz, double dt) {
    phase += hz * dt;
    return phase - std::floor(phase);
}

float loopSample(const std::vector<float>& pcm, std::size_t frames, int channels, double position, int channel) {
    const std::size_t first = static_cast<std::size_t>(position);
    const std::size_t second = first + 1 < frames ? first + 1 : 0;
    const float fraction = static_cast<float>(position - static_cast<double>(first));
    const float a = pcm[first * static_cast<std::size_t>(channels) + channel];
    const float b = pcm[second * static_cast<std::size_t>(channels) + channel];
    return a + (b - a) * fraction;
}

double advanceLoop(double position, float rate, std::size_t frames) {
    position += rate;
    if (position >= static_cast<double>(frames))
        position -= std::floor(position / static_cast<double>(frames)) * static_cast<double>(frames);
    return position;
}
} // namespace

bool DriftAudio::init() {
    if (ready_) return true;
    Uint16 format = 0;
    if (!Mix_QuerySpec(&sampleRate_, &format, &channels_) ||
        format != AUDIO_S16SYS || sampleRate_ <= 0 || channels_ <= 0) {
        return false;
    }
    loadSampleLoops(SDL_RWFromFile("assets/audio/engine-c.wav", "rb"),
                    SDL_RWFromFile("assets/audio/tire-a.wav", "rb"));
    loadShots("assets/audio/sfx");
    if (shotsLoaded() < kShotCount) SDL_Log("%d of %d effect files loaded; the rest are synthesized", shotsLoaded(), int(kShotCount));
    if (!engineSampleLoaded()) SDL_Log("Engine loop unavailable; using synthesized engine fallback");
    if (!tireSampleLoaded()) SDL_Log("Tire loop unavailable; using synthesized tire fallback");
    reset();
    ready_ = true;
    Mix_SetPostMix(postMix, this);
    return true;
}

void DriftAudio::cue(Cue which, int step) {
    const unsigned write = cueWrite_.load(std::memory_order_relaxed);
    if (write - cueRead_.load(std::memory_order_acquire) >= kCueQueue) return;
    cueQueue_[write % kCueQueue] = {int(which), step};
    cueWrite_.store(write + 1, std::memory_order_release);
}

const char* DriftAudio::shotName(int shot) {
    static const char* const names[kShotCount] = {
        "item_0", "item_1", "item_2", "item_3", "item_4", "item_5", "item_6", "item_7",
        "boost_1", "boost_2", "boost_3", "whoosh_item", "whoosh_boost",
        "count_tick", "count_go", "sector_best", "sector_slower", "lap", "record"};
    return shot >= 0 && shot < kShotCount ? names[shot] : "";
}

void DriftAudio::loadShots(const char* directory) {
    // Like the loops: decoded before the callback is installed, never after.
    int loaded = 0;
    for (int shot = 0; shot < kShotCount; ++shot) {
        const std::string path = std::string(directory) + "/" + shotName(shot) + ".ogg";
        if (loadLoop(SDL_RWFromFile(path.c_str(), "rb"), shots_[shot])) ++loaded;
    }
    shotsLoaded_.store(loaded, std::memory_order_release);
}

void DriftAudio::setRevTuning(const RevTuning& t) {
    const float values[kRevFields] = {
        t.chargeSemis[0], t.chargeSemis[1], t.chargeSemis[2], t.chargeDb[0], t.chargeDb[1], t.chargeDb[2],
        t.blipSemis, t.blipDb, t.blipTime, t.creepSemis, t.boostSemis[0], t.boostSemis[1], t.boostSemis[2],
        t.boostDb, t.boostTime, t.itemSemis, t.itemDb, t.itemTime, t.boostWhooshDb, t.itemWhooshDb, t.pickupDb,
        t.afterfireDb};
    for (int i = 0; i < kRevFields; ++i) revTuning_[i].store(values[i], std::memory_order_relaxed);
}

DriftAudio::RevTuning DriftAudio::loadRevTuning() const {
    float v[kRevFields];
    for (int i = 0; i < kRevFields; ++i) v[i] = revTuning_[i].load(std::memory_order_relaxed);
    RevTuning t;
    for (int k = 0; k < 3; ++k) {
        t.chargeSemis[k] = limit(v[k], -12, 12);
        t.chargeDb[k] = limit(v[3 + k], -12, 12);
        t.boostSemis[k] = limit(v[10 + k], 0, 12);
    }
    t.blipSemis = limit(v[6], 0, 12);
    t.blipDb = limit(v[7], 0, 12);
    t.blipTime = limit(v[8], 0.02f, 2);
    t.creepSemis = limit(v[9], 0, 6);
    t.boostDb = limit(v[13], -12, 12);
    t.boostTime = limit(v[14], 0.2f, 3);
    t.itemSemis = limit(v[15], 0, 12);
    t.itemDb = limit(v[16], -12, 12);
    t.itemTime = limit(v[17], 0.02f, 2);
    t.boostWhooshDb = limit(v[18], -40, 12);
    t.itemWhooshDb = limit(v[19], -40, 12);
    t.pickupDb = limit(v[20], -40, 12);
    t.afterfireDb = limit(v[21], -40, 12);
    return t;
}

// Callback only. Plays a designed effect if its file loaded; a busy pool gives
// up the voice that is furthest through its sound.
bool DriftAudio::startShot(int shot, bool boost, float gain) {
    if (shot < 0 || shot >= kShotCount || shots_[shot].empty()) return false;
    Voice* chosen = nullptr;
    for (auto& voice : voices_) {
        if (voice.shot < 0) { chosen = &voice; break; }
        if (!chosen || voice.frame > chosen->frame) chosen = &voice;
    }
    *chosen = Voice{shot, 0, boost, gain};
    return true;
}

void DriftAudio::addTone(float delay, float length, float hz, float gain, float bright) {
    for (auto& tone : tones_) {
        if (tone.active) continue;
        tone = Tone{delay, 0, length, hz, gain, bright, 0, true};
        return;
    }
}

// Each cue is a few bell-like tones on a C major scale, kept above the engine.
void DriftAudio::startCue(int type, int step, const RevTuning& tuning) {
    static const int scale[] = {0, 2, 4, 5, 7, 9, 11, 12, 14, 16, 17, 19, 21};
    if (Cue(type) == Cue::Item) {
        // The push is heard in the engine and the air; the pickup is its own sound.
        fxItemTime_ = 0;
        if (tuning.itemWhooshDb > -30) startShot(kShotWhooshItem, false, decibels(tuning.itemWhooshDb));
        if (tuning.pickupDb <= -30 || startShot(kShotItem + std::max(0, std::min(7, step)), false, decibels(tuning.pickupDb)))
            return;
    }
    const int designed[] = {kShotCountTick, kShotCountGo, -1, kShotSectorBest, kShotSectorSlower, kShotLap, kShotRecord};
    if (type >= 0 && type < int(sizeof(designed) / sizeof(designed[0])) && designed[type] >= 0 && startShot(designed[type])) return;
    switch (Cue(type)) {
    case Cue::CountTick: addTone(0, 0.16f, 659.26f, 0.11f, 0.45f); break;
    case Cue::CountGo:
        addTone(0, 0.55f, 1318.5f, 0.11f, 0.45f);
        addTone(0, 0.55f, 659.26f, 0.06f, 0.2f);
        break;
    case Cue::Item: {
        const float hz = 987.77f * std::pow(2.0f, scale[std::max(0, std::min(12, step))] / 12.0f);
        addTone(0, 0.11f, hz, 0.06f, 0.5f);
        addTone(0.05f, 0.16f, hz * 1.4983f, 0.05f, 0.5f);
        break;
    }
    case Cue::SectorBest:
        addTone(0, 0.22f, 1046.5f, 0.08f, 0.5f);
        addTone(0.08f, 0.32f, 1568.0f, 0.08f, 0.5f);
        break;
    case Cue::SectorSlower: addTone(0, 0.18f, 392.0f, 0.05f, 0.2f); break;
    case Cue::Lap: {
        const float notes[] = {523.25f, 659.26f, 783.99f, 1046.5f};
        for (int i = 0; i < 4; ++i) addTone(0.07f * i, 0.42f, notes[i], 0.07f, 0.45f);
        break;
    }
    case Cue::Record: {
        const float notes[] = {523.25f, 659.26f, 783.99f, 1046.5f, 1318.5f, 1568.0f};
        for (int i = 0; i < 6; ++i) addTone(0.07f * i, 0.36f, notes[i], 0.07f, 0.5f);
        for (float hz : {1046.5f, 1318.5f, 1568.0f}) addTone(0.46f, 1.25f, hz, 0.055f, 0.35f);
        break;
    }
    }
}

float DriftAudio::renderTones(double dt) {
    float sum = 0;
    for (auto& tone : tones_) {
        if (!tone.active) continue;
        if (tone.delay > 0) { tone.delay -= float(dt); continue; }
        const float t = tone.age;
        // A quick strike, then a bell-like decay that reaches silence at the end.
        const float envelope = std::min(t / 0.004f, 1.0f) * std::exp(-t * 4.6f / tone.length) *
                               limit((tone.length - t) / 0.02f, 0, 1);
        tone.phase = advance(tone.phase, tone.hz, dt);
        const double p = tone.phase * kTau;
        sum += float(std::sin(p) + tone.bright * (0.35 * std::sin(2 * p) + 0.15 * std::sin(3 * p))) * envelope * tone.gain;
        tone.age += float(dt);
        if (tone.age >= tone.length) tone.active = false;
    }
    return sum;
}

bool DriftAudio::loadLoop(SDL_RWops* source, std::vector<float>& pcm) {
    pcm.clear();
    if (!source) return false;
    // Mix_LoadWAV_RW decodes and converts to the already opened mixer format.
    // Never play this chunk on a mixer channel: its PCM belongs to our postmix.
    Mix_Chunk* chunk = Mix_LoadWAV_RW(source, 1);
    if (!chunk) return false;
    const std::size_t frameBytes = sizeof(Sint16) * static_cast<std::size_t>(channels_);
    const bool valid = chunk->abuf && frameBytes > 0 && chunk->alen >= frameBytes * 2 && chunk->alen % frameBytes == 0;
    if (valid) {
        const std::size_t samples = chunk->alen / sizeof(Sint16);
        pcm.resize(samples);
        const Sint16* input = reinterpret_cast<const Sint16*>(chunk->abuf);
        for (std::size_t i = 0; i < samples; ++i) pcm[i] = static_cast<float>(input[i]) / 32768.0f;
    }
    Mix_FreeChunk(chunk);
    return valid;
}

void DriftAudio::loadSampleLoops(SDL_RWops* engineSource, SDL_RWops* tireSource) {
    // This helper is used only before callback installation (also by offline
    // PCM fixtures); it must never be called while a callback is rendering.
    engineSampleLoaded_.store(loadLoop(engineSource, enginePcm_), std::memory_order_release);
    tireSampleLoaded_.store(loadLoop(tireSource, tirePcm_), std::memory_order_release);
}

void DriftAudio::update(float speed, float throttle, float drift, float impact, bool paused, float driveSpeed, float acceleration) {
    const float roadSpeed = limit(std::fabs(speed), 0, 120);
    speed_.store(roadSpeed, std::memory_order_relaxed);
    driveSpeed_.store(limit(driveSpeed >= 0 ? driveSpeed : roadSpeed, 0, 90), std::memory_order_relaxed);
    acceleration_.store(paused ? 0 : limit(acceleration, 0, 100), std::memory_order_relaxed);
    throttle_.store(limit(throttle, 0, 1), std::memory_order_relaxed);
    drift_.store(limit(drift, 0, 1), std::memory_order_relaxed);
    paused_.store(paused, std::memory_order_relaxed);
    if (paused) pendingLandingImpact_.store(0, std::memory_order_relaxed);
    const std::uint32_t now = SDL_GetTicks();
    if (!paused && impact > 3.0f && (lastImpactMs_ == 0 || now - lastImpactMs_ > 180)) {
        impact_.store(limit(impact / 20.0f, 0.15f, 1.0f), std::memory_order_relaxed);
        lastImpactMs_ = now;
    }
}

void DriftAudio::bank(float quality) {
    bankQuality_.store(limit(quality, 0, 1), std::memory_order_relaxed);
    bankSerial_.fetch_add(1, std::memory_order_release);
}

void DriftAudio::setDriftState(int chargeLevel, int boostLevel, unsigned boostSerial, float boostRemaining) {
    chargeLevel_.store(std::max(0, std::min(3, chargeLevel)), std::memory_order_relaxed);
    boostLevel_.store(std::max(0, std::min(3, boostLevel)), std::memory_order_relaxed);
    boostRemaining_.store(limit(boostRemaining, 0, 5), std::memory_order_relaxed);
    // Publish the event serial after the matching boost parameters.
    boostSerial_.store(boostSerial, std::memory_order_release);
}

void DriftAudio::setMuted(bool muted) {
    muted_.store(muted, std::memory_order_relaxed);
    if (muted) pendingLandingImpact_.store(0, std::memory_order_relaxed);
}

void DriftAudio::setGain(float gain) {
    const float bounded = limit(gain, 0, 1.5f);
    gain_.store(bounded, std::memory_order_relaxed);
    if (bounded == 0) pendingLandingImpact_.store(0, std::memory_order_relaxed);
}

void DriftAudio::setLayerGains(float engine, float tire) {
    engineGain_.store(limit(engine, 0, 1.5f), std::memory_order_relaxed);
    tireGain_.store(limit(tire, 0, 1.5f), std::memory_order_relaxed);
}

void DriftAudio::setWindGain(float gain) {
    windGain_.store(limit(gain, 0, 1.5f), std::memory_order_relaxed);
}

void DriftAudio::setRevProfile(float scale, float returnTime) {
    engineRevScale_.store(limit(scale, 0, 2), std::memory_order_relaxed);
    engineRevReturn_.store(limit(returnTime, 0.2f, 1.5f), std::memory_order_relaxed);
}

void DriftAudio::setLandingState(unsigned serial, float impact) {
    if (!landingSerialPrimed_) {
        landingSerial_ = serial;
        landingSerialPrimed_ = true;
        return;
    }
    if (serial == landingSerial_) return;
    landingSerial_ = serial;
    if (paused_.load(std::memory_order_relaxed) || muted_.load(std::memory_order_relaxed) ||
        gain_.load(std::memory_order_relaxed) <= 0) return;
    // Serial matching happens entirely on the control thread. A single atomic
    // mailbox transfers the matching amplitude, with no split serial/value pair,
    // callback-side retry, allocation or lock. Sub-threshold contacts stay quiet.
    const float bounded = limit(impact, 0, 30);
    if (bounded > 0.35f)
        pendingLandingImpact_.store(limit(bounded / 10.0f, 0, 1), std::memory_order_release);
}

void DriftAudio::reset() {
    impact_.store(0, std::memory_order_relaxed);
    acceleration_.store(0, std::memory_order_relaxed);
    lastImpactMs_ = 0;
    pendingLandingImpact_.store(0, std::memory_order_relaxed);
    landingSerialPrimed_ = false;
    resetSerial_.fetch_add(1, std::memory_order_release);
}

void DriftAudio::shutdown() {
    if (ready_) Mix_SetPostMix(nullptr, nullptr);
    ready_ = false;
    // Mix_SetPostMix holds the audio lock, so no callback can retain these
    // buffers after detachment. Free their storage only on this control thread.
    engineSampleLoaded_.store(false, std::memory_order_release);
    tireSampleLoaded_.store(false, std::memory_order_release);
    std::vector<float>().swap(enginePcm_);
    std::vector<float>().swap(tirePcm_);
    for (auto& shot : shots_) std::vector<float>().swap(shot);
    shotsLoaded_.store(0, std::memory_order_release);
}

void DriftAudio::postMix(void* userdata, std::uint8_t* stream, int length) {
    static_cast<DriftAudio*>(userdata)->render(stream, length);
}

void DriftAudio::render(std::uint8_t* stream, int length) {
    const unsigned reset = resetSerial_.load(std::memory_order_acquire);
    if (reset != seenResetSerial_) {
        seenResetSerial_ = reset;
        enginePhase_ = tirePhase_ = cuePhase_ = 0;
        engineSamplePosition_ = tireSamplePosition_ = 0;
        engineSampleRate_ = 0.75f;
        enginePlaybackRate_ = 0.75f;
        engineRevLevel_ = 0;
        tireSampleRate_ = 1;
        // Preserve per-layer silence across R/reset. Starting these at one
        // would briefly reveal a muted bed while its gain smoothed back down.
        engineMixGain_ = limit(engineGain_.load(std::memory_order_relaxed), 0, 1.5f);
        tireMixGain_ = limit(tireGain_.load(std::memory_order_relaxed), 0, 1.5f);
        boostPhase_ = boostAirPhase_ = 0;
        pitch_ = 48;
        engineLevel_ = driftLevel_ = gate_ = 0;
        noiseLow_ = noiseHigh_ = crashEnvelope_ = 0;
        bankTime_ = -1;
        cueQuality_ = 0;
        seenChargeLevel_ = chargeLevel_.load(std::memory_order_relaxed);
        seenBoostSerial_ = boostSerial_.load(std::memory_order_acquire);
        boostSerialPrimed_ = true;
        chargeCueTime_ = boostCueTime_ = -1;
        boostCueDuration_ = 0;
        chargeCueLevel_ = boostCueLevel_ = 0;
        chargeAirLow_ = chargeAirHigh_ = 0;
        chargeRandom_ = 0x9e36b421;
        boostNoise_ = boostLowNoise_ = boostTail_ = boostVoiceGate_ = 0;
        windLevel_ = windLow_ = windHigh_ = 0;
        windRandom_ = 0x35ba17c9;
        landingPhase_ = 0;
        landingCueTime_ = -1;
        landingCueImpact_ = landingLow_ = 0;
        random_ = 0x8e4d6a39;
        seenBankSerial_ = bankSerial_.load(std::memory_order_acquire);
        for (auto& tone : tones_) tone.active = false;
        for (auto& voice : voices_) voice.shot = -1;
        shotBoostGate_ = 0;
        fxSemis_ = fxDb_ = 0;
        fxLevel_ = chargeLevel_.load(std::memory_order_relaxed);
        fxLevelAge_ = 10;
        fxBoostTime_ = fxItemTime_ = -1;
        fxBoostFromSemis_ = fxBoostFromDb_ = 0;
        fxBoostLevel_ = 0;
    }
    const double dt = 1.0 / sampleRate_;
    const float smooth = 1.0f - std::exp(-static_cast<float>(dt) * 20.0f);
    // Times specify 90% response, so comparisons have an unambiguous meaning.
    const float driftAttack = 1.0f - std::exp(-static_cast<float>(dt) * 2.302585f / 0.060f);
    const float gateSmooth = 1.0f - std::exp(-static_cast<float>(dt) * 90.0f);
    const float lowCoeff = 1.0f - std::exp(-static_cast<float>(dt) * 800.0f * float(kTau));
    const float highCoeff = 1.0f - std::exp(-static_cast<float>(dt) * 3300.0f * float(kTau));
    const float crashDecay = std::exp(-static_cast<float>(dt) * 16.0f);
    const float speed = speed_.load(std::memory_order_relaxed);
    const float driveSpeed = driveSpeed_.load(std::memory_order_relaxed);
    const float throttle = throttle_.load(std::memory_order_relaxed);
    const float drift = drift_.load(std::memory_order_relaxed);
    const bool silent = paused_.load(std::memory_order_relaxed) || muted_.load(std::memory_order_relaxed);
    const float gain = gain_.load(std::memory_order_relaxed);
    const float engineGain = engineGain_.load(std::memory_order_relaxed);
    const float tireGain = tireGain_.load(std::memory_order_relaxed);
    const float windGain = windGain_.load(std::memory_order_relaxed);
    const float revScale = engineRevScale_.load(std::memory_order_relaxed);
    const float revReturn = engineRevReturn_.load(std::memory_order_relaxed);
    const float acceleration = acceleration_.load(std::memory_order_relaxed);
    const std::size_t engineFrames = enginePcm_.size() / static_cast<std::size_t>(channels_);
    const std::size_t tireFrames = tirePcm_.size() / static_cast<std::size_t>(channels_);
    const bool recordedEngine = engineFrames >= 2;
    const bool recordedTire = tireFrames >= 2;
    const unsigned boostSerial = boostSerial_.load(std::memory_order_acquire);
    const int chargeLevel = chargeLevel_.load(std::memory_order_relaxed);
    const int boostLevel = boostLevel_.load(std::memory_order_relaxed);
    const float boostRemaining = boostRemaining_.load(std::memory_order_relaxed);
    const bool suppressCue = silent || gain <= 0;
    const RevTuning rt = loadRevTuning();
    {
        // Drain the jingle queue. Cues raised while silent are dropped, not delayed.
        const unsigned write = cueWrite_.load(std::memory_order_acquire);
        unsigned read = cueRead_.load(std::memory_order_relaxed);
        for (; read != write; ++read) {
            const CueEvent event = cueQueue_[read % kCueQueue];
            if (!suppressCue) startCue(event.type, event.step, rt);
        }
        cueRead_.store(read, std::memory_order_release);
        if (suppressCue) {
            for (auto& tone : tones_) tone.active = false;
            for (auto& voice : voices_) voice.shot = -1;
        }
    }
    const float landingImpact = pendingLandingImpact_.exchange(0, std::memory_order_acquire);
    if (suppressCue) {
        landingCueTime_ = -1;
        landingCueImpact_ = 0;
    } else if (landingImpact > 0) {
        landingCueTime_ = 0;
        landingCueImpact_ = landingImpact;
        landingPhase_ = 0;
        landingLow_ = 0;
    }
    // Wind uses actual road speed (including boost), not drivetrain RPM. The
    // bed is broad filtered noise; there is no pitched wind oscillator.
    const float windSpeed = limit((speed - 4.0f) / 51.0f, 0, 1);
    const float targetWind = suppressCue ? 0 : 0.16f * std::pow(windSpeed, 1.35f) * windGain;
    if (suppressCue || windGain <= 0) windLevel_ = 0;
    const float windAttack = 1 - std::exp(-static_cast<float>(dt) * 2.302585f / 0.16f);
    const float windRelease = 1 - std::exp(-static_cast<float>(dt) * 2.302585f / 0.30f);
    const float windLowCoeff = 1 - std::exp(-static_cast<float>(dt) * float(kTau) * 100);
    const float windHighCoeff = 1 - std::exp(-static_cast<float>(dt) * float(kTau) * (1600 + 1200 * windSpeed));
    const float landingLowCoeff = 1 - std::exp(-static_cast<float>(dt) * float(kTau) * 160);
    const bool boostActive = !suppressCue && boostRemaining > 0 && boostLevel > 0;
    const float targetRev = !suppressCue && revScale > 0 && speed > 0.5f && (throttle > 0.05f || boostActive)
        ? limit(acceleration / 20.0f, 0, 1) : 0;
    if (suppressCue || revScale <= 0) engineRevLevel_ = 0;
    const float revAttack = 1 - std::exp(-static_cast<float>(dt) * 2.302585f / 0.12f);
    const float revRelease = 1 - std::exp(-static_cast<float>(dt) * 2.302585f / revReturn);
    // A valid boosted exit should reveal the engine/air attack as the tires
    // recover. Ordinary sliding and cancelled exits keep their original release.
    const float driftReleaseTime = boostActive ? 0.075f : 0.180f;
    const float driftRelease = 1.0f - std::exp(-static_cast<float>(dt) * 2.302585f / driftReleaseTime);
    // With the engine recording, the engine itself revs for each charge level
    // (below); the short pressure puffs remain for the synthesized engine.
    if (chargeLevel > seenChargeLevel_ && !suppressCue && !recordedEngine) {
        chargeCueLevel_ = chargeLevel;
        chargeCueTime_ = 0;
        chargeAirLow_ = chargeAirHigh_ = 0;
    }
    seenChargeLevel_ = chargeLevel;
    if (!boostSerialPrimed_) {
        // Joining a running state must not play a release that happened earlier.
        seenBoostSerial_ = boostSerial;
        boostSerialPrimed_ = true;
    } else if (boostSerial != seenBoostSerial_) {
        seenBoostSerial_ = boostSerial;
        if (!suppressCue && boostLevel > 0 && boostRemaining > 0) {
            // The engine flares from wherever the charge left it (below).
            fxBoostTime_ = 0;
            fxBoostLevel_ = boostLevel;
            fxBoostFromSemis_ = fxSemis_;
            fxBoostFromDb_ = fxDb_;
        }
        if (!suppressCue && boostLevel > 0 && boostRemaining > 0 && !shots_[kShotWhooshBoost].empty()) {
            // The rush of air, and the afterfire for the level, ride with it.
            if (rt.boostWhooshDb > -30)
                startShot(kShotWhooshBoost, true, kFxBoostWhoosh[boostLevel] * decibels(rt.boostWhooshDb));
            if (rt.afterfireDb > -30) startShot(kShotBoost + boostLevel - 1, true, decibels(rt.afterfireDb));
            chargeCueTime_ = -1;
            shotBoostGate_ = 1;
        } else if (!suppressCue && boostLevel > 0 && boostRemaining > 0) {
            boostCueLevel_ = boostLevel;
            boostCueTime_ = 0;
            boostCueDuration_ = std::min(boostRemaining, kBoostVoiceDurations[boostLevel]);
            boostPhase_ = boostAirPhase_ = 0;
            boostVoiceGate_ = 0;
            chargeCueTime_ = -1;
        }
    }
    if (suppressCue) {
        fxBoostTime_ = fxItemTime_ = -1;
        chargeCueTime_ = boostCueTime_ = -1;
        boostTail_ = boostVoiceGate_ = 0;
        bankTime_ = -1;
    }
    const float chargeAirDuration = kChargeAirDurations[chargeCueLevel_];
    const float chargeAirLowCoeff = 1 - std::exp(-static_cast<float>(dt) * float(kTau) * (130 + chargeCueLevel_ * 50));
    const float chargeAirHighCoeff = 1 - std::exp(-static_cast<float>(dt) * float(kTau) * (950 + chargeCueLevel_ * 250));
    const float boostNoiseCoeff = 1.0f - std::exp(-static_cast<float>(dt) * 1800.0f * float(kTau));
    const float boostLowCoeff = 1.0f - std::exp(-static_cast<float>(dt) * 280.0f * float(kTau));
    const float boostGateSmooth = 1.0f - std::exp(-static_cast<float>(dt) * 2.302585f / 0.008f);
    // A cancelled boost (brake, barrier) fades its designed sound out quickly.
    const float shotGateSmooth = 1.0f - std::exp(-static_cast<float>(dt) * 2.302585f / 0.060f);
    // The engine's offsets rise quickly and fall back more slowly, as revs do.
    const float fxUp = 1.0f - std::exp(-static_cast<float>(dt) * 2.302585f / 0.025f);
    const float fxDown = 1.0f - std::exp(-static_cast<float>(dt) * 2.302585f / 0.150f);
    const int fxHeld = suppressCue ? 0 : chargeLevel;
    if (fxHeld > fxLevel_) fxLevelAge_ = 0; // Each level-up revs again.
    fxLevel_ = fxHeld;
    // A little pitch fall at each shift gives acceleration a rhythmic pulse.
    // The temporary road-speed pulse is not a gear shift. Keep the drivetrain
    // pitch/load stable while tires and the air voice still use the real speed.
    const int gear = std::min(5, static_cast<int>(driveSpeed / 13.0f));
    const float rpm = 1100.0f + (driveSpeed - gear * 13.0f) * 260.0f + throttle * 950.0f;
    const float targetPitch = 22.0f + rpm / 30.0f;
    const float targetEngine = 0.026f + throttle * 0.060f + limit(driveSpeed / 90.0f, 0, 1) * 0.018f;
    // The selected recording supplies the timbre. Keep its playback range
    // moderate and monotonic at the practice cap, with a small throttle lift.
    // Preserve the previous flow at scale0. With the rev profile enabled, the
    // steady bed keeps a restrained rise from100 through160km/h, while actual
    // acceleration supplies a separate short flare instead of permanent pitch.
    const float highSpeedRate = revScale > 0 ? limit((driveSpeed - 100.0f/3.6f) / (60.0f/3.6f), 0, 1) * 0.12f : 0;
    const float targetEngineRate = limit(0.75f + limit(driveSpeed / (100.0f / 3.6f), 0, 1) * 0.65f + throttle * 0.20f, 0.75f, 1.6f) + highSpeedRate;
    const float targetTireRate = 0.95f + drift * 0.06f + limit(speed / 40.0f, 0, 1) * 0.04f;
    const float targetDrift = speed > 5 ? drift : 0;
    const float impact = impact_.exchange(0, std::memory_order_relaxed);
    crashEnvelope_ = suppressCue ? 0 : std::max(crashEnvelope_, impact);
    const unsigned serial = bankSerial_.load(std::memory_order_acquire);
    if (serial != seenBankSerial_) {
        seenBankSerial_ = serial;
        bankTime_ = suppressCue ? -1 : 0;
        cuePhase_ = 0;
        cueQuality_ = bankQuality_.load(std::memory_order_relaxed);
    }

    Sint16* samples = reinterpret_cast<Sint16*>(stream);
    const int frames = length / (static_cast<int>(sizeof(Sint16)) * channels_);
    for (int frame = 0; frame < frames; ++frame) {
        {
            // How far the engine is revved past its normal note, and how much
            // louder: the held charge, a release, an item's push.
            float semis = 0, db = 0;
            if (fxBoostTime_ >= 0) {
                // A release: up from where the charge left it, then down onto
                // the normal note as the boost runs out. A cut-short boost drops back.
                const float duration = kFxBoostDuration[fxBoostLevel_] * rt.boostTime;
                const float t = fxBoostTime_;
                const float rise = smoothstep01(t / kFxRise);
                const float settle = smoothstep01((t - kFxRise) / (duration - kFxRise));
                const float peak = rt.boostSemis[fxBoostLevel_ - 1];
                semis = (fxBoostFromSemis_ + (peak - fxBoostFromSemis_) * rise) * (1 - settle);
                db = (fxBoostFromDb_ + (rt.boostDb - fxBoostFromDb_) * rise) * (1 - settle);
                if (!boostActive) semis = db = 0;
                fxBoostTime_ += static_cast<float>(dt);
                if (fxBoostTime_ >= duration) fxBoostTime_ = -1;
            } else if (fxLevel_ > 0) {
                // "Bwo": past the level's note and back; "ooon": easing up while held.
                const float t = fxLevelAge_;
                const float blip = smoothstep01(t / 0.06f) * std::exp(-std::max(0.0f, t - 0.06f) / rt.blipTime);
                semis = rt.chargeSemis[fxLevel_ - 1] + rt.blipSemis * blip + rt.creepSemis * (1 - std::exp(-t / 1.2f));
                db = rt.chargeDb[fxLevel_ - 1] + rt.blipDb * blip;
                fxLevelAge_ += static_cast<float>(dt);
            }
            if (fxItemTime_ >= 0) {
                const float t = fxItemTime_;
                const float push = smoothstep01(t / 0.04f) * std::exp(-std::max(0.0f, t - 0.04f) / rt.itemTime);
                semis += rt.itemSemis * push;
                db += rt.itemDb * push;
                fxItemTime_ += static_cast<float>(dt);
                if (t > 0.04f + 6 * rt.itemTime) fxItemTime_ = -1;
            }
            fxSemis_ += (semis - fxSemis_) * (semis > fxSemis_ ? fxUp : fxDown);
            fxDb_ += (db - fxDb_) * (db > fxDb_ ? fxUp : fxDown);
        }
        // Only the engine recording revs; the synthesized fallback keeps its own cues.
        const float fxPitch = recordedEngine ? semitones(fxSemis_) : 1.0f;
        const float fxGain = recordedEngine ? decibels(fxDb_) : 1.0f;
        pitch_ += (targetPitch - pitch_) * smooth;
        engineLevel_ += (targetEngine - engineLevel_) * smooth;
        engineSampleRate_ += (targetEngineRate - engineSampleRate_) * smooth;
        engineRevLevel_ += (targetRev - engineRevLevel_) * (targetRev > engineRevLevel_ ? revAttack : revRelease);
        enginePlaybackRate_ = limit((engineSampleRate_ + engineRevLevel_ * revScale * 0.22f) * fxPitch, 0.6f, 3.6f);
        tireSampleRate_ += (targetTireRate - tireSampleRate_) * smooth;
        engineMixGain_ += (engineGain - engineMixGain_) * smooth;
        tireMixGain_ += (tireGain - tireMixGain_) * smooth;
        driftLevel_ += (targetDrift - driftLevel_) * (targetDrift > driftLevel_ ? driftAttack : driftRelease);
        gate_ += ((silent ? 0.0f : 1.0f) - gate_) * gateSmooth;
        enginePhase_ = advance(enginePhase_, pitch_ * (1 + engineRevLevel_ * revScale * 0.14f), dt);
        const double phase = enginePhase_ * kTau;
        const float synthesizedEngine = engineLevel_ * (
            0.64f * std::sin(phase) + 0.26f * std::sin(phase * 2) +
            0.13f * std::sin(phase * 3) + 0.08f * std::sin(phase * 5));

        random_ ^= random_ << 13;
        random_ ^= random_ >> 17;
        random_ ^= random_ << 5;
        const float noise = float(random_ & 0xffffu) / 32767.5f - 1.0f;
        noiseLow_ += (noise - noiseLow_) * lowCoeff;
        noiseHigh_ += (noise - noiseHigh_) * highCoeff;
        const float scrub = noiseHigh_ - noiseLow_;
        tirePhase_ = advance(tirePhase_, 780.0f + driftLevel_ * 430.0f + speed * 2.1f, dt);
        const float synthesizedTire = driftLevel_ * (
            scrub * 0.15f + std::sin(tirePhase_ * kTau) * 0.025f);

        const float crash = crashEnvelope_ * (noise * 0.17f + noiseLow_ * 0.20f);
        crashEnvelope_ *= crashDecay;
        windRandom_ ^= windRandom_ << 13;
        windRandom_ ^= windRandom_ >> 17;
        windRandom_ ^= windRandom_ << 5;
        const float windNoise = float(windRandom_ & 0xffffu) / 32767.5f - 1.0f;
        windLow_ += (windNoise - windLow_) * windLowCoeff;
        windHigh_ += (windNoise - windHigh_) * windHighCoeff;
        windLevel_ += (targetWind - windLevel_) * (targetWind > windLevel_ ? windAttack : windRelease);
        const float wind = (windHigh_ - windLow_) * windLevel_;
        float landing = 0;
        if (landingCueTime_ >= 0 && landingCueTime_ < 0.18f) {
            const float t = landingCueTime_;
            landingPhase_ = advance(landingPhase_, 52.0f + 38.0f * std::exp(-t * 32), dt);
            landingLow_ += (noise - landingLow_) * landingLowCoeff;
            const float envelope = std::min(t / 0.004f, 1.0f) * std::exp(-t * 30) *
                                   limit((0.18f - t) / 0.025f, 0, 1);
            landing = (0.82f * std::sin(landingPhase_ * kTau) + 0.18f * landingLow_) *
                      envelope * landingCueImpact_ * 0.16f;
            landingCueTime_ += static_cast<float>(dt);
        }
        float cue = 0;
        if (bankTime_ >= 0 && bankTime_ < 0.44f) {
            // Three rising notes confirm a clean exit without covering the tires.
            const int note = std::min(2, static_cast<int>(bankTime_ / 0.12f));
            const float notes[] = {523.251f, 659.255f, 783.991f};
            const float noteTime = bankTime_ - note * 0.12f;
            const float envelope = std::min(noteTime * 180.0f, 1.0f) * std::exp(-noteTime * 15.0f);
            cuePhase_ = advance(cuePhase_, notes[note] * (1.0f + cueQuality_ * 0.04f), dt);
            cue = std::sin(cuePhase_ * kTau) * envelope * 0.085f;
            bankTime_ += static_cast<float>(dt);
        }
        // Each readiness step is one short pressure puff, with no note or
        // harmonic oscillator. Its own noise state leaves every other voice
        // unchanged; repeated held-state updates still cannot retrigger it.
        float chargeCue = 0;
        if (chargeCueTime_ >= 0 && chargeCueTime_ < chargeAirDuration) {
            chargeRandom_ ^= chargeRandom_ << 13;
            chargeRandom_ ^= chargeRandom_ >> 17;
            chargeRandom_ ^= chargeRandom_ << 5;
            const float pressureNoise = float(chargeRandom_ & 0xffffu) / 32767.5f - 1.0f;
            chargeAirLow_ += (pressureNoise - chargeAirLow_) * chargeAirLowCoeff;
            chargeAirHigh_ += (pressureNoise - chargeAirHigh_) * chargeAirHighCoeff;
            const float progress = chargeCueTime_ / chargeAirDuration;
            const float envelope = std::min(chargeCueTime_ / 0.005f, 1.0f) * (1-progress) * (1-progress);
            const float intensity = 1 + (chargeCueLevel_ - 1) * 0.12f;
            chargeCue = (chargeAirHigh_ - chargeAirLow_) * envelope * kChargeAirGain * intensity;
            chargeCueTime_ += static_cast<float>(dt);
        }
        boostNoise_ += (noise - boostNoise_) * boostNoiseCoeff;
        boostLowNoise_ += (noise - boostLowNoise_) * boostLowCoeff;
        boostVoiceGate_ += ((boostActive ? 1.0f : 0.0f) - boostVoiceGate_) * boostGateSmooth;
        float boostCue = 0;
        boostTail_ = 0;
        if (boostCueTime_ >= 0 && boostCueTime_ < boostCueDuration_) {
            // A short, warm kick establishes the release. A separate, lower
            // band of air follows it; neither voice reuses the charge puff.
            const float progress = limit(boostCueTime_ / boostCueDuration_, 0, 1);
            const float kickProgress = limit(boostCueTime_ / 0.095f, 0, 1);
            const float kickEnvelope = std::min(boostCueTime_ / 0.005f, 1.0f) *
                                       (1-kickProgress) * (1-kickProgress);
            boostPhase_ = advance(boostPhase_, 145.0f - 70.0f * kickProgress, dt);
            const float kick = (0.72f * std::sin(boostPhase_ * kTau) + boostLowNoise_ * 0.28f) *
                               kickEnvelope * (0.11f + boostCueLevel_ * 0.012f);
            boostTail_ = std::min(boostCueTime_ / 0.016f, 1.0f) *
                         (1-progress) * (1-progress) * boostVoiceGate_;
            const float jetHz = 285.0f - 105.0f * progress + speed * 0.18f;
            boostAirPhase_ = advance(boostAirPhase_, jetHz, dt);
            const float air = (0.92f * (boostNoise_ - boostLowNoise_) +
                               0.08f * std::sin(boostAirPhase_ * kTau)) *
                              boostTail_ * (0.13f + boostCueLevel_ * 0.012f);
            boostCue = kick * boostVoiceGate_ + air;
            boostCueTime_ += static_cast<float>(dt);
        }
        const float effects = crash + cue + chargeCue + boostCue + wind + landing + renderTones(dt);
        // Designed one-shots keep their stereo image.
        float shot[2] = {0, 0};
        shotBoostGate_ += ((boostActive ? 1.0f : 0.0f) - shotBoostGate_) * shotGateSmooth;
        for (auto& voice : voices_) {
            if (voice.shot < 0) continue;
            const std::vector<float>& pcm = shots_[voice.shot];
            const std::size_t frames = pcm.size() / static_cast<std::size_t>(channels_);
            if (voice.frame >= frames || (voice.boost && shotBoostGate_ < 0.001f)) { voice.shot = -1; continue; }
            const float level = (voice.boost ? shotBoostGate_ : 1.0f) * voice.gain;
            const std::size_t at = voice.frame * static_cast<std::size_t>(channels_);
            shot[0] += pcm[at] * level;
            shot[1] += pcm[at + (channels_ > 1 ? 1 : 0)] * level;
            ++voice.frame;
        }
        for (int channel = 0; channel < channels_; ++channel) {
            // Preserve the recordings' converted channel image. Only fallback
            // tire synthesis needs the original small artificial stereo spread.
            const float spread = (channel & 1) ? 0.96f : 1.04f;
            const float engine = recordedEngine ? loopSample(enginePcm_, engineFrames, channels_, engineSamplePosition_, channel) * engineLevel_ * fxGain * 4.0f : synthesizedEngine;
            const float tire = recordedTire ? loopSample(tirePcm_, tireFrames, channels_, tireSamplePosition_, channel) * driftLevel_ * 0.45f : synthesizedTire * spread;
            const float value = (engine * engineMixGain_ * (1 + engineRevLevel_ * revScale * 0.06f) + tire * tireMixGain_ + effects +
                                 shot[channel & 1]) * gate_;
            const int index = frame * channels_ + channel;
            const float mixed = float(samples[index]) + value * gain * 32767.0f;
            samples[index] = static_cast<Sint16>(limit(mixed, -29203, 29203)); // -1 dBFS headroom
        }
        if (recordedEngine) engineSamplePosition_ = advanceLoop(engineSamplePosition_, enginePlaybackRate_, engineFrames);
        if (recordedTire) tireSamplePosition_ = advanceLoop(tireSamplePosition_, tireSampleRate_, tireFrames);
    }
}
