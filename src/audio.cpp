#include "audio.hpp"

#include <SDL.h>
#include <SDL_mixer.h>

#include <algorithm>
#include <cmath>

namespace {
constexpr double kTau = 6.283185307179586;
constexpr float kBoostVoiceDurations[] = {0, 0.35f, 0.50f, 0.65f};
constexpr float kChargeAirDurations[] = {0, 0.080f, 0.100f, 0.130f};
constexpr float kChargeAirGain = 0.10f;
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
    if (!engineSampleLoaded()) SDL_Log("Engine loop unavailable; using synthesized engine fallback");
    if (!tireSampleLoaded()) SDL_Log("Tire loop unavailable; using synthesized tire fallback");
    reset();
    ready_ = true;
    Mix_SetPostMix(postMix, this);
    return true;
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

void DriftAudio::pickup(int step) {
    pickupStep_.store(std::max(0, std::min(12, step)), std::memory_order_relaxed);
    pickupSerial_.fetch_add(1, std::memory_order_release);
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
        pickupTime_ = -1;
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
    if (chargeLevel > seenChargeLevel_ && !suppressCue) {
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
            boostCueLevel_ = boostLevel;
            boostCueTime_ = 0;
            boostCueDuration_ = std::min(boostRemaining, kBoostVoiceDurations[boostLevel]);
            boostPhase_ = boostAirPhase_ = 0;
            boostVoiceGate_ = 0;
            chargeCueTime_ = -1;
        }
    }
    if (suppressCue) {
        chargeCueTime_ = boostCueTime_ = -1;
        boostTail_ = boostVoiceGate_ = 0;
        bankTime_ = -1;
        pickupTime_ = -1;
    }
    const float chargeAirDuration = kChargeAirDurations[chargeCueLevel_];
    const float chargeAirLowCoeff = 1 - std::exp(-static_cast<float>(dt) * float(kTau) * (130 + chargeCueLevel_ * 50));
    const float chargeAirHighCoeff = 1 - std::exp(-static_cast<float>(dt) * float(kTau) * (950 + chargeCueLevel_ * 250));
    const float boostNoiseCoeff = 1.0f - std::exp(-static_cast<float>(dt) * 1800.0f * float(kTau));
    const float boostLowCoeff = 1.0f - std::exp(-static_cast<float>(dt) * 280.0f * float(kTau));
    const float boostGateSmooth = 1.0f - std::exp(-static_cast<float>(dt) * 2.302585f / 0.008f);
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
    const unsigned pickupSerial = pickupSerial_.load(std::memory_order_acquire);
    if (pickupSerial != seenPickupSerial_) {
        seenPickupSerial_ = pickupSerial;
        pickupTime_ = suppressCue ? -1 : 0;
        pickupPhase_ = 0;
        // A major-scale climb: each item in a chain sounds one step higher.
        static const int scale[] = {0, 2, 4, 5, 7, 9, 11, 12, 14, 16, 17, 19, 21};
        pickupFreq_ = 987.77f * std::pow(2.0f, scale[pickupStep_.load(std::memory_order_relaxed)] / 12.0f);
    }
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
        pitch_ += (targetPitch - pitch_) * smooth;
        engineLevel_ += (targetEngine - engineLevel_) * smooth;
        engineSampleRate_ += (targetEngineRate - engineSampleRate_) * smooth;
        engineRevLevel_ += (targetRev - engineRevLevel_) * (targetRev > engineRevLevel_ ? revAttack : revRelease);
        enginePlaybackRate_ = limit(engineSampleRate_ + engineRevLevel_ * revScale * 0.22f, 0.75f, 2.16f);
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
        if (pickupTime_ >= 0 && pickupTime_ < 0.22f) {
            // Two quick bright partials, the second a fifth above.
            const float jump = pickupTime_ < 0.055f ? 1.0f : 1.4983f;
            const float noteTime = pickupTime_ < 0.055f ? pickupTime_ : pickupTime_ - 0.055f;
            const float envelope = std::min(noteTime * 400.0f, 1.0f) * std::exp(-noteTime * 22.0f);
            pickupPhase_ = advance(pickupPhase_, pickupFreq_ * jump, dt);
            cue += (std::sin(pickupPhase_ * kTau) + 0.25f * std::sin(pickupPhase_ * 2 * kTau)) * envelope * 0.07f;
            pickupTime_ += static_cast<float>(dt);
        }
        if (bankTime_ >= 0 && bankTime_ < 0.44f) {
            // Three rising notes confirm a clean exit without covering the tires.
            const int note = std::min(2, static_cast<int>(bankTime_ / 0.12f));
            const float notes[] = {523.251f, 659.255f, 783.991f};
            const float noteTime = bankTime_ - note * 0.12f;
            const float envelope = std::min(noteTime * 180.0f, 1.0f) * std::exp(-noteTime * 15.0f);
            cuePhase_ = advance(cuePhase_, notes[note] * (1.0f + cueQuality_ * 0.04f), dt);
            cue += std::sin(cuePhase_ * kTau) * envelope * 0.085f;
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
        const float effects = crash + cue + chargeCue + boostCue + wind + landing;
        for (int channel = 0; channel < channels_; ++channel) {
            // Preserve the recordings' converted channel image. Only fallback
            // tire synthesis needs the original small artificial stereo spread.
            const float spread = (channel & 1) ? 0.96f : 1.04f;
            const float engine = recordedEngine ? loopSample(enginePcm_, engineFrames, channels_, engineSamplePosition_, channel) * engineLevel_ * 4.0f : synthesizedEngine;
            const float tire = recordedTire ? loopSample(tirePcm_, tireFrames, channels_, tireSamplePosition_, channel) * driftLevel_ * 0.45f : synthesizedTire * spread;
            const float value = (engine * engineMixGain_ * (1 + engineRevLevel_ * revScale * 0.06f) + tire * tireMixGain_ + effects) * gate_;
            const int index = frame * channels_ + channel;
            const float mixed = float(samples[index]) + value * gain * 32767.0f;
            samples[index] = static_cast<Sint16>(limit(mixed, -29203, 29203)); // -1 dBFS headroom
        }
        if (recordedEngine) engineSamplePosition_ = advanceLoop(engineSamplePosition_, enginePlaybackRate_, engineFrames);
        if (recordedTire) tireSamplePosition_ = advanceLoop(tireSamplePosition_, tireSampleRate_, tireFrames);
    }
}
