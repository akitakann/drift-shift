#include <SDL.h>
#include <SDL_mixer.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <vector>

// This standalone test exercises the callback deterministically without a real
// audio device racing its state. No test hooks are added to the game interface.
#define private public
#include "audio.hpp"
#undef private
#include "../src/audio.cpp"

namespace {
constexpr int rate = 44100;
constexpr int channels = 2;
int failures = 0;

void check(bool condition, const char* label) {
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", label);
        ++failures;
    }
}

std::vector<Sint16> renderFrames(DriftAudio& audio, int frames, Sint16 initial = 0) {
    std::vector<Sint16> pcm(static_cast<std::size_t>(frames) * channels, initial);
    audio.render(reinterpret_cast<std::uint8_t*>(pcm.data()),
                 static_cast<int>(pcm.size() * sizeof(Sint16)));
    return pcm;
}

std::vector<Sint16> renderSeconds(DriftAudio& audio, double seconds) {
    return renderFrames(audio, static_cast<int>(std::lround(seconds * rate)));
}

double rms(const std::vector<Sint16>& pcm) {
    double energy = 0;
    for (Sint16 sample : pcm) {
        const double value = sample / 32768.0;
        energy += value * value;
    }
    return pcm.empty() ? 0 : std::sqrt(energy / pcm.size());
}

double differenceRms(const std::vector<Sint16>& pcm) {
    double energy = 0;
    for (std::size_t i = channels; i < pcm.size(); i += channels) {
        const double difference = (double(pcm[i]) - pcm[i-channels]) / 32768.0;
        energy += difference * difference;
    }
    return std::sqrt(energy / (pcm.size() / channels - 1));
}

double fundamental(const std::vector<Sint16>& pcm, int lowHz = 40, int highHz = 300) {
    double bestHz = 0;
    double bestEnergy = 0;
    for (int hz = lowHz; hz <= highHz; ++hz) {
        const double coefficient = 2 * std::cos(6.283185307179586 * hz / rate);
        double previous = 0;
        double before = 0;
        for (std::size_t i = 0; i < pcm.size(); i += channels) {
            const double next = pcm[i] / 32768.0 + coefficient * previous - before;
            before = previous;
            previous = next;
        }
        const double energy = previous * previous + before * before - coefficient * previous * before;
        if (energy > bestEnergy) {
            bestEnergy = energy;
            bestHz = hz;
        }
    }
    return bestHz;
}

double toneFraction(const std::vector<Sint16>& pcm, double hz) {
    const double coefficient=2*std::cos(6.283185307179586*hz/rate);
    double previous=0,before=0,energy=0;
    for (std::size_t i=0;i<pcm.size();i+=channels) {
        const double value=pcm[i]/32768.0;
        const double next=value+coefficient*previous-before;
        before=previous;previous=next;energy+=value*value;
    }
    const double power=previous*previous+before*before-coefficient*previous*before;
    return energy>0?2*std::fmax(power,0)/(energy*(pcm.size()/channels)):0;
}

std::vector<Sint16> difference(const std::vector<Sint16>& a, const std::vector<Sint16>& b) {
    check(a.size() == b.size(), "audio comparison blocks have equal lengths");
    std::vector<Sint16> result(std::min(a.size(), b.size()));
    for (std::size_t i = 0; i < result.size(); ++i) {
        const int value = int(a[i]) - int(b[i]);
        result[i] = static_cast<Sint16>(std::max(-32768, std::min(32767, value)));
    }
    return result;
}

bool isSilence(const std::vector<Sint16>& pcm) {
    return std::all_of(pcm.begin(), pcm.end(), [](Sint16 value) { return value == 0; });
}

void lifecycle() {
    DriftAudio unavailable;
    check(!unavailable.init(), "init fails cleanly when SDL_mixer has no open device");
    if (Mix_OpenAudio(rate, AUDIO_S16SYS, channels, 512) != 0) {
        check(false, "dummy SDL audio device opens");
        std::fprintf(stderr, "SDL mixer: %s\n", Mix_GetError());
        return;
    }
    {
        DriftAudio live;
        check(live.init() && live.init(), "SDL_mixer callback installs and repeated init is safe");
        check(live.ready_ && live.sampleRate_ == rate && live.channels_ == channels,
              "callback uses the actual signed16 stereo mixer format");
        check(live.engineSampleLoaded()==!live.enginePcm_.empty() && live.tireSampleLoaded()==!live.tirePcm_.empty(),
              "public sample status matches whichever optional engine and tire assets init could load");
        live.update(22.2f, 0.5f, 0.5f, 0, false);
        live.setMuted(true);
        live.shutdown();
        live.shutdown();
        check(!live.ready_ && !live.engineSampleLoaded() && !live.tireSampleLoaded() &&
              live.enginePcm_.empty() && live.tirePcm_.empty(),
              "shutdown removes the callback before freeing sample buffers and closing the mixer");
    }
    Mix_CloseAudio();
}

void envelopes() {
    DriftAudio audio;
    audio.update(22.2f, 0, 1, 0, false);
    renderSeconds(audio, 0.060);
    check(std::fabs(audio.driftLevel_ - 0.9f) < 0.001f,
          "tire attack reaches 90 percent after 60ms of generated audio");
    const float held = audio.driftLevel_;
    audio.update(22.2f, 0, 0, 0, false);
    renderSeconds(audio, 0.180);
    check(std::fabs(audio.driftLevel_ - held * 0.1f) < 0.001f,
          "tire release removes 90 percent after 180ms of generated audio");

    DriftAudio slow;
    slow.update(2, 1, 1, 0, false);
    renderSeconds(slow, 0.5);
    check(slow.driftLevel_ == 0, "walking-speed inputs cannot trigger tire squeal");
}

void loadAndSlip() {
    DriftAudio coast;
    DriftAudio power;
    coast.update(22.2f, 0, 0, 0, false);
    power.update(22.2f, 1, 0, 0, false);
    renderSeconds(coast, 0.5);
    renderSeconds(power, 0.5);
    const auto coastPcm = renderSeconds(coast, 0.25);
    const auto powerPcm = renderSeconds(power, 0.25);
    const double coastHz = fundamental(coastPcm);
    const double powerHz = fundamental(powerPcm);
    check(rms(powerPcm) > rms(coastPcm) * 2.5,
          "throttle load creates an audible engine level difference at the same road speed");
    check(powerHz > coastHz + 20 && std::fabs(powerHz - power.pitch_) < 3,
          "engine PCM spectrum follows the changing engine pitch");

    DriftAudio slide;
    slide.update(22.2f, 1, 1, 0, false);
    renderSeconds(slide, 0.5);
    const auto slidePcm = renderSeconds(slide, 0.25);
    check(rms(slidePcm) > rms(powerPcm) && differenceRms(slidePcm) > differenceRms(powerPcm) * 3,
          "actual slip adds distinct high-frequency tire content above the engine");
    std::printf("audio PCM: coast RMS %.4f / %.0f Hz; throttle %.4f / %.0f Hz; slide %.4f\n",
                rms(coastPcm), coastHz, rms(powerPcm), powerHz, rms(slidePcm));
}

void silenceAndHeadroom() {
    DriftAudio audio;
    audio.setGain(1.5f);
    audio.update(90, 1, 1, 100, false);
    audio.bank(1);
    const auto maximum = renderSeconds(audio, 1);
    check(std::all_of(maximum.begin(), maximum.end(), [](Sint16 value) { return value >= -29203 && value <= 29203; }),
          "combined engine tire impact and cue retain minus1dBFS sample headroom at maximum gain");
    check(rms(maximum) > 0.04, "headroom check contains audible generated content");
    const auto alreadyLoud = renderFrames(audio, 1024, 32767);
    check(*std::max_element(alreadyLoud.begin(), alreadyLoud.end()) <= 29203,
          "postmix bounds also apply when earlier mixer channels already contain loud samples");

    audio.setMuted(true);
    renderSeconds(audio, 0.3);
    check(isSilence(renderSeconds(audio, 0.2)), "mute settles to exact zero PCM");
    audio.setMuted(false);
    audio.update(22.2f, 1, 1, 0, false);
    check(rms(renderSeconds(audio, 0.2)) > 0.01, "unmute restores engine and tires");
    audio.update(22.2f, 1, 1, 0, true);
    renderSeconds(audio, 0.3);
    check(isSilence(renderSeconds(audio, 2)), "pause remains silent over a long output block");
    audio.update(22.2f, 1, 1, 0, false);
    audio.setGain(0);
    check(isSilence(renderSeconds(audio, 0.2)), "the no-audio comparison candidate generates zero PCM");
    check(std::isfinite(audio.pitch_) && std::isfinite(audio.engineLevel_) &&
          std::isfinite(audio.driftLevel_) && std::isfinite(audio.noiseLow_) &&
          std::isfinite(audio.noiseHigh_) && std::isfinite(audio.gate_) &&
          audio.enginePhase_ >= 0 && audio.enginePhase_ < 1 && audio.tirePhase_ >= 0 && audio.tirePhase_ < 1,
          "long renders keep synthesis state finite and oscillator phases bounded");
}

void controlBoundsAndDeterminism() {
    DriftAudio bounded;
    bounded.update(-200, 4, 3, 0, false);
    bounded.setGain(100);
    check(bounded.speed_.load() == 120 && bounded.driveSpeed_.load() == 90 && bounded.throttle_.load() == 1 &&
          bounded.drift_.load() == 1 && bounded.gain_.load() == 1.5f,
          "external speed throttle drift and gain inputs are bounded");
    bounded.update(0, -2, -2, 0, false);
    bounded.setGain(-1);
    check(bounded.throttle_.load() == 0 && bounded.drift_.load() == 0 && bounded.gain_.load() == 0,
          "negative control amounts cannot become negative output gain");
    bounded.update(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::quiet_NaN(),
                   std::numeric_limits<float>::infinity(), 0, false,
                   std::numeric_limits<float>::quiet_NaN());
    bounded.setGain(std::numeric_limits<float>::quiet_NaN());
    renderSeconds(bounded, 0.1);
    check(std::isfinite(bounded.pitch_) && std::isfinite(bounded.engineLevel_) &&
          std::isfinite(bounded.driftLevel_) && isSilence(renderSeconds(bounded, 0.1)),
          "nonfinite public controls cannot poison oscillators or produce invalid PCM");

    DriftAudio whole;
    DriftAudio chunks;
    whole.update(22.2f, 0.7f, 0.6f, 0, false);
    chunks.update(22.2f, 0.7f, 0.6f, 0, false);
    whole.bank(0.5f);
    chunks.bank(0.5f);
    const auto single = renderSeconds(whole, 1);
    std::vector<Sint16> divided;
    for (int i = 0; i < 10; ++i) {
        const auto block = renderSeconds(chunks, 0.1);
        divided.insert(divided.end(), block.begin(), block.end());
    }
    check(single == divided, "steady replay output is independent of audio callback block boundaries");
}

void boostedDrivetrainPitch() {
    constexpr float base = 80.0f / 3.6f;
    DriftAudio ordinary;
    DriftAudio boosted;
    DriftAudio oldRoadCoupling;
    for (DriftAudio* audio : {&ordinary, &boosted, &oldRoadCoupling}) {
        audio->update(base, 1, 0, 0, false);
        renderSeconds(*audio, 0.4);
    }
    ordinary.update(base, 1, 0, 0, false, base);
    boosted.update(base + 18, 1, 0, 0, false, base);
    oldRoadCoupling.update(base + 18, 1, 0, 0, false);
    const auto normalPcm = renderSeconds(ordinary, 0.3);
    const auto boostedPcm = renderSeconds(boosted, 0.3);
    renderSeconds(oldRoadCoupling, 0.3);
    check(normalPcm == boostedPcm && std::fabs(boosted.pitch_ - ordinary.pitch_) < 0.001f,
          "temporary road-speed bonus leaves the normal engine waveform and RPM unchanged");
    check(oldRoadCoupling.pitch_ < ordinary.pitch_ - 40,
          "the regression setup crosses road-speed gears that formerly made the boost pitch fall");
    const double engineHz = fundamental(boostedPcm);
    check(std::fabs(engineHz - ordinary.pitch_) < 3,
          "boost-independent drivetrain pitch is present in the rendered engine spectrum");

    ordinary.update(base, 1, 1, 0, false, base);
    boosted.update(base + 18, 1, 1, 0, false, base);
    check(renderSeconds(ordinary, 0.2) != renderSeconds(boosted, 0.2),
          "road-speed-dependent tire motion still differs when the engine speed is held equal");

    DriftAudio reverseDefault;
    DriftAudio reverseExplicit;
    reverseDefault.update(-8, 0.7f, 0, 0, false);
    reverseExplicit.update(-8, 0.7f, 0, 0, false, 8);
    check(renderSeconds(reverseDefault, 0.3) == renderSeconds(reverseExplicit, 0.3),
          "explicit nonnegative drivetrain speed preserves the normal reverse engine sound");
    std::printf("boost engine: baseline %.1fHz / explicit drivetrain %.1fHz / old road-coupled %.1fHz\n",
                ordinary.pitch_, boosted.pitch_, oldRoadCoupling.pitch_);
}

void boostedTireRecovery() {
    DriftAudio boosted;
    DriftAudio ordinary;
    for (DriftAudio* audio : {&boosted, &ordinary}) {
        audio->update(22.2f, 1, 1, 0, false);
        audio->setDriftState(0, 0, 0, 0);
        renderSeconds(*audio, 0.3);
        audio->update(22.2f, 1, 0, 0, false);
    }
    const float held = boosted.driftLevel_;
    boosted.setDriftState(0, 3, 1, 0.65f);
    renderSeconds(boosted, 0.075);
    renderSeconds(ordinary, 0.075);
    check(std::fabs(boosted.driftLevel_ - held * 0.1f) < 0.001f &&
          ordinary.driftLevel_ > boosted.driftLevel_ * 3,
          "a valid boosted exit reveals its kick by clearing tires in75ms; ordinary release keeps180ms");

    boosted.update(22.2f, 1, 1, 0, false);
    renderSeconds(boosted, 0.060);
    check(boosted.driftLevel_ > 0.9f,
          "new actual slip still attacks normally during a boost instead of being artificially silenced");
}

void boostKickAndAirTail() {
    constexpr float base = 80.0f / 3.6f;
    DriftAudio boosted;
    DriftAudio plain;
    for (DriftAudio* audio : {&boosted, &plain}) {
        audio->update(base, 1, 0, 0, false, base);
        audio->setDriftState(0, 0, 0, 0);
        renderSeconds(*audio, 0.3);
    }
    boosted.update(base + 18, 1, 0, 0, false, base);
    boosted.setDriftState(0, 3, 1, 0.65f);
    const auto onset = difference(renderSeconds(boosted, 0.060), renderSeconds(plain, 0.060));
    renderSeconds(boosted, 0.060); renderSeconds(plain, 0.060);
    const auto tail = difference(renderSeconds(boosted, 0.080), renderSeconds(plain, 0.080));
    renderSeconds(boosted, 0.250); renderSeconds(plain, 0.250);
    const auto late = difference(renderSeconds(boosted, 0.100), renderSeconds(plain, 0.100));
    renderSeconds(boosted, 0.200); renderSeconds(plain, 0.200);
    const auto expired = difference(renderSeconds(boosted, 0.200), renderSeconds(plain, 0.200));
    const double onsetRms = rms(onset);
    const double tailRms = rms(tail);
    const double lateRms = rms(late);
    check(onsetRms > 0.012 && onsetRms > tailRms * 1.3 && tailRms > 0.002,
          "release PCM has a distinct immediate kick followed by a quieter audible air tail");
    check(lateRms < tailRms * 0.3 && isSilence(expired),
          "the air tail naturally fades and ends even if the last published boost state is unchanged");

    DriftAudio charge;
    DriftAudio chargePlain;
    for (DriftAudio* audio : {&charge, &chargePlain}) {
        audio->update(base, 1, 0, 0, false, base);
        audio->setDriftState(0, 0, 0, 0);
        renderSeconds(*audio, 0.3);
    }
    charge.setDriftState(3, 0, 0, 0);
    const auto readiness = difference(renderSeconds(charge, 0.060), renderSeconds(chargePlain, 0.060));
    const double kickHz = fundamental(onset, 50, 650);
    check(kickHz < 220 && rms(readiness)>0.002 &&
          differenceRms(readiness)/rms(readiness)>differenceRms(onset)/onsetRms,
          "boost onset keeps its low kick distinct from the broader pressure readiness accent");

    DriftAudio whole;
    DriftAudio chunks;
    for (DriftAudio* audio : {&whole, &chunks}) {
        audio->update(base + 18, 1, 0, 0, false, base);
        audio->setDriftState(0, 0, 0, 0);
        renderSeconds(*audio, 0.1);
        audio->setDriftState(0, 3, 1, 0.65f);
    }
    const auto single = renderSeconds(whole, 0.65);
    std::vector<Sint16> divided;
    for (int i = 0; i < 6; ++i) {
        const auto block = renderSeconds(chunks, 0.1);
        divided.insert(divided.end(), block.begin(), block.end());
    }
    const auto finalBlock = renderSeconds(chunks, 0.05);
    divided.insert(divided.end(), finalBlock.begin(), finalBlock.end());
    check(single == divided,
          "release kick and air tail are deterministic across audio callback block boundaries");
    std::printf("boost voice PCM: onset RMS %.4f / %.0fHz; air %.4f; late %.4f; pressure readiness RMS %.4f\n",
                onsetRms, kickHz, tailRms, lateRms, rms(readiness));
}

void pressureReadiness() {
    std::vector<std::vector<Sint16>> stages;
    int previousLast=0;
    for (int level=1;level<=3;++level) {
        DriftAudio audio;
        audio.setLayerGains(0,0);
        audio.reset();
        audio.update(22.2f,1,0,0,false);
        audio.setDriftState(0,0,0,0);
        renderSeconds(audio,.1);
        audio.setDriftState(level,0,0,0);
        const auto pcm=renderSeconds(audio,.2);
        double strongest=0,lowerBand=0,upperBand=0;
        for (int hz=200;hz<=2400;hz+=10) {
            const double fraction=toneFraction(pcm,hz);
            strongest=std::fmax(strongest,fraction);
            if (hz>=300&&hz<=900) lowerBand+=fraction;
            if (hz>=1100&&hz<=2200) upperBand+=fraction;
        }
        check(rms(pcm)>.003 && strongest<.04 && lowerBand>.08 && upperBand>.08,
              "each pressure cue spreads measurable energy across broad bands without a dominant stable musical partial");
        for (double oldNote : {329.628,415.305,523.251,659.256,830.61,1046.502})
            check(toneFraction(pcm,oldNote)<.03,"old readiness notes and harmonics no longer dominate the pressure cue spectrum");
        int last=0;
        bool mono=true,bounded=true;
        for (std::size_t i=0;i<pcm.size();i+=channels) {
            if (pcm[i]!=0) last=static_cast<int>(i/channels);
            mono &= pcm[i]==pcm[i+1];
            bounded &= std::abs(int(pcm[i]))<5000;
        }
        const double duration=last/double(rate);
        const double expected[]={0,.08,.10,.13};
        check(duration>expected[level]-.008 && duration<expected[level] && last>previousLast &&
              audio.chargeCueTime_>=expected[level]-.0001 && mono && bounded,
              "stages have distinct80/100/130ms finite pressure tails with equal stereo channels and ample headroom");
        previousLast=last;
        audio.setDriftState(level,0,0,0);
        check(isSilence(renderSeconds(audio,.1)),"holding a completed stage cannot restart its pressure cue");
        audio.setDriftState(level-1,0,0,0);
        check(isSilence(renderSeconds(audio,.1)),"a descending stage cannot start a new pressure cue");
        stages.push_back(pcm);
        std::printf("pressure level%d: %.0fms, RMS %.4f, strongest narrow component %.3f\n",level,duration*1000,rms(pcm),strongest);
    }
    check(rms(stages[1])>rms(stages[0])*1.1 && rms(stages[2])>rms(stages[1])*1.1,
          "higher readiness stages differ in measured duration and energy rather than a musical note sequence");

    DriftAudio charged,plain;
    for (DriftAudio* audio : {&charged,&plain}) {
        audio->setWindGain(.6f);audio->setRevProfile(1,.65f);
        audio->update(30,1,.4f,0,false,30,20);
        audio->setDriftState(0,0,0,0);
        renderSeconds(*audio,.2);
    }
    charged.setDriftState(3,0,0,0);
    renderSeconds(charged,.2);renderSeconds(plain,.2);
    check(charged.random_==plain.random_ && charged.windRandom_==plain.windRandom_ &&
          charged.enginePhase_==plain.enginePhase_ && renderSeconds(charged,.1)==renderSeconds(plain,.1),
          "charge noise has independent state and leaves engine tire wind rev and other synthesis identical after its finite cue");

    DriftAudio whole,chunks;
    for (DriftAudio* audio : {&whole,&chunks}) {
        audio->setLayerGains(0,0);audio->reset();audio->update(22.2f,1,0,0,false);
        audio->setDriftState(0,0,0,0);renderSeconds(*audio,.1);
        audio->setDriftState(3,0,0,0);
    }
    const auto single=renderSeconds(whole,.2);
    std::vector<Sint16> divided;
    for (int i=0;i<20;++i) {
        const auto block=renderSeconds(chunks,.01);
        divided.insert(divided.end(),block.begin(),block.end());
    }
    check(single==divided,"pressure noise filters and its finite envelope remain deterministic across callback block boundaries");
    whole.reset();whole.setDriftState(0,0,0,0);whole.update(22.2f,1,0,0,false);renderSeconds(whole,.1);
    whole.setDriftState(3,0,0,0);
    check(single==renderSeconds(whole,.2),"reset restores the pressure noise seed filters and envelope without changing stage timing");
}

void resetIsolation() {
    DriftAudio previousRun;
    previousRun.setGain(1.5f);
    previousRun.update(82, 1, 1, 0, false);
    previousRun.bank(1);
    renderSeconds(previousRun, 0.37);
    check(previousRun.driftLevel_ > 0.99f && previousRun.bankTime_ > 0 &&
          previousRun.bankTime_ < 0.44f && previousRun.enginePhase_ != 0,
          "reset isolation begins with active tires engine oscillators and an unfinished success cue");

    // Queue additional old-trial events without consuming them in the callback.
    previousRun.update(82, 1, 1, 100, false);
    previousRun.bank(0.9f);
    check(previousRun.impact_.load() > 0 && previousRun.bankSerial_.load() != previousRun.seenBankSerial_,
          "isolation test contains pending old-trial impact and success events");
    previousRun.reset();
    previousRun.setGain(1);
    previousRun.update(22.2f, 0.4f, 0.25f, 0, false);

    DriftAudio freshRun;
    freshRun.update(22.2f, 0.4f, 0.25f, 0, false);
    const auto resetPcm = renderSeconds(previousRun, 0.4);
    const auto freshPcm = renderSeconds(freshRun, 0.4);
    check(resetPcm == freshPcm,
          "reset trial produces identical PCM to a fresh synthesizer at the same controls");
    check(previousRun.crashEnvelope_ == 0 && previousRun.bankTime_ == -1 &&
          previousRun.seenBankSerial_ == previousRun.bankSerial_.load(),
          "queued previous-trial impact and success cue cannot leak through reset");
    check(previousRun.enginePhase_ == freshRun.enginePhase_ &&
          previousRun.tirePhase_ == freshRun.tirePhase_ &&
          previousRun.driftLevel_ == freshRun.driftLevel_ &&
          previousRun.noiseLow_ == freshRun.noiseLow_ &&
          previousRun.noiseHigh_ == freshRun.noiseHigh_ &&
          previousRun.random_ == freshRun.random_,
          "reset restores oscillator envelope filter and noise-generator state consistently");

    // Once the reset has been consumed, genuinely new events remain audible.
    previousRun.bank(0.5f);
    freshRun.bank(0.5f);
    const auto newCue = renderSeconds(previousRun, 0.1);
    check(newCue == renderSeconds(freshRun, 0.1) && previousRun.bankTime_ > 0,
          "new-trial success events work normally after reset");

    previousRun.reset();
    previousRun.reset();
    previousRun.update(22.2f, 0, 0, 0, false);
    DriftAudio secondFresh;
    secondFresh.update(22.2f, 0, 0, 0, false);
    check(renderSeconds(previousRun, 0.2) == renderSeconds(secondFresh, 0.2),
          "multiple reset requests before a callback still establish one clean trial start");
}

void driftStateCues() {
    DriftAudio audio;
    DriftAudio plain;
    audio.update(22.2f, 0.6f, 0, 0, false);
    plain.update(22.2f, 0.6f, 0, 0, false);
    audio.setDriftState(0, 0, 0, 0);
    renderSeconds(audio, 0.25);
    renderSeconds(plain, 0.25);
    audio.setDriftState(1, 0, 0, 0);
    const auto firstCharge = renderSeconds(audio, 0.025);
    const auto plainCharge = renderSeconds(plain, 0.025);
    check(firstCharge != plainCharge && audio.chargeCueLevel_ == 1 && audio.chargeCueTime_ > 0,
          "increasing charge produces an original audible readiness cue");
    const float firstTime = audio.chargeCueTime_;
    audio.setDriftState(1, 0, 0, 0);
    renderSeconds(audio, 0.025);
    check(audio.chargeCueTime_ > firstTime + 0.024f,
          "repeated held charge state does not restart its readiness cue");
    audio.setDriftState(2, 0, 0, 0);
    renderSeconds(audio, 0.015);
    check(audio.chargeCueLevel_ == 2 && audio.chargeCueTime_ < 0.016f,
          "each higher charge step gets its own short cue");
    const float secondTime = audio.chargeCueTime_;
    audio.setDriftState(0, 0, 0, 0);
    renderSeconds(audio, 0.015);
    check(audio.chargeCueLevel_ == 2 && audio.chargeCueTime_ > secondTime + 0.014f,
          "falling charge levels do not emit a fake readiness cue");

    audio.setDriftState(0, 3, 1, 0.8f);
    renderSeconds(audio, 0.025);
    check(audio.boostCueLevel_ == 3 && audio.boostCueTime_ > 0 && audio.boostCueTime_ < 0.026f && audio.boostTail_ > 0,
          "a new release serial creates a whoosh attack and ongoing boost tail");
    const float boostTime = audio.boostCueTime_;
    audio.setDriftState(0, 3, 1, 0.75f);
    renderSeconds(audio, 0.025);
    check(audio.boostCueTime_ > boostTime + 0.024f,
          "a repeated boost serial cannot retrigger the release attack");
    audio.setDriftState(0, 0, 1, 0);
    renderSeconds(audio, 0.8);
    check(audio.boostTail_ < 0.00001f && audio.boostCueTime_ >= 0.45f,
          "boost audio decays after the boost ends instead of becoming a permanent layer");

    DriftAudio joining;
    joining.setDriftState(0, 2, 40, 0.5f);
    renderSeconds(joining, 0.025);
    check(joining.seenBoostSerial_ == 40 && joining.boostCueTime_ == -1,
          "the first observed serial establishes a baseline without inventing a past release event");

    DriftAudio maximum;
    maximum.update(90, 1, 1, 100, false);
    maximum.setGain(1.5f);
    maximum.setDriftState(0, 0, 0, 0);
    renderSeconds(maximum, 0.01);
    maximum.setDriftState(3, 3, 1, 1);
    maximum.bank(1);
    const auto loud = renderSeconds(maximum, 1);
    check(std::all_of(loud.begin(), loud.end(), [](Sint16 v) { return v >= -29203 && v <= 29203; }) &&
          std::isfinite(maximum.boostNoise_) && std::isfinite(maximum.boostLowNoise_) &&
          std::isfinite(maximum.boostTail_) && std::isfinite(maximum.boostVoiceGate_),
          "readiness release engine tires and legacy cue stay finite with minus1dBFS headroom");
    check(std::none_of(loud.begin(), loud.end(), [](Sint16 v) { return v == -29203 || v == 29203; }),
          "the loudest generated layers retain headroom without relying on the output safety clamp");
    maximum.setDriftState(100, 100, 2, 100);
    check(maximum.chargeLevel_.load() == 3 && maximum.boostLevel_.load() == 3 && maximum.boostRemaining_.load() == 5,
          "drift cue inputs are bounded before reaching oscillators and envelopes");
}

void driftCueSuppressionAndReset() {
    DriftAudio audio;
    audio.update(22.2f, 0.5f, 0, 0, false);
    audio.setDriftState(0, 0, 0, 0);
    renderSeconds(audio, 0.1);
    audio.setMuted(true);
    audio.setDriftState(2, 3, 1, 0.8f);
    renderSeconds(audio, 0.05);
    check(audio.seenChargeLevel_ == 2 && audio.seenBoostSerial_ == 1 &&
          audio.chargeCueTime_ == -1 && audio.boostCueTime_ == -1 && audio.boostTail_ == 0,
          "muted events are consumed without queued charge or release cues");
    audio.setMuted(false);
    audio.setDriftState(2, 0, 1, 0);
    renderSeconds(audio, 0.05);
    check(audio.chargeCueTime_ == -1 && audio.boostCueTime_ == -1,
          "unmuting does not play charge or boost events that happened while muted");
    audio.setDriftState(3, 2, 2, 0.5f);
    renderSeconds(audio, 0.025);
    check(audio.boostCueTime_ >= 0 && audio.boostTail_ > 0 && audio.chargeCueTime_ == -1,
          "new release works after unmuting and takes priority over the readiness chime");
    audio.update(22.2f, 0.5f, 0, 0, true);
    renderSeconds(audio, 0.05);
    check(audio.chargeCueTime_ == -1 && audio.boostCueTime_ == -1 && audio.boostTail_ == 0,
          "pausing clears active cue tails instead of resuming a stale transient");
    audio.setDriftState(0, 0, 2, 0);
    audio.reset();
    audio.update(22.2f, 0.5f, 0.4f, 0, false);
    DriftAudio fresh;
    fresh.update(22.2f, 0.5f, 0.4f, 0, false);
    fresh.setDriftState(0, 0, 2, 0);
    check(renderSeconds(audio, 0.2) == renderSeconds(fresh, 0.2),
          "reset removes readiness and release history and restores deterministic clean audio");
    check(audio.chargeCueTime_ == -1 && audio.boostCueTime_ == -1 && audio.boostTail_ == 0,
          "old charge and boost cannot leak into the new trial");
}

void suppressedEventsNeverReturn() {
    constexpr float base = 80.0f / 3.6f;
    DriftAudio events;
    DriftAudio plain;
    for (DriftAudio* audio : {&events, &plain}) {
        audio->update(base, 1, 0, 0, false, base);
        audio->setDriftState(0, 0, 0, 0);
        renderSeconds(*audio, 0.2);
        audio->setMuted(true);
    }
    events.update(base + 18, 1, 0, 100, false, base);
    events.setDriftState(3, 3, 1, 0.65f);
    events.bank(1);
    renderSeconds(events, 0.050);
    renderSeconds(plain, 0.050);
    events.setMuted(false);
    plain.setMuted(false);
    events.setDriftState(3, 3, 1, 0.60f);
    check(renderSeconds(events, 0.100) == renderSeconds(plain, 0.100),
          "unmuting with an active state never plays old release charge crash or success events");

    events.setDriftState(0, 3, 2, 0.65f);
    const auto freshKick = renderSeconds(events, 0.025);
    const auto noKick = renderSeconds(plain, 0.025);
    check(rms(difference(freshKick, noKick)) > 0.01,
          "a genuinely new release is audible after muted events have been consumed");
    events.update(base + 18, 1, 0, 0, true, base);
    plain.update(base, 1, 0, 0, true, base);
    renderSeconds(events, 0.050);
    renderSeconds(plain, 0.050);
    events.update(base + 18, 1, 0, 0, false, base);
    plain.update(base, 1, 0, 0, false, base);
    events.setDriftState(0, 3, 2, 0.60f);
    check(renderSeconds(events, 0.150) == renderSeconds(plain, 0.150),
          "resuming while the same boost remains active never restarts its release attack or air tail");

    events.setGain(0);
    events.setDriftState(2, 2, 3, 0.50f);
    renderSeconds(events, 0.025);
    events.setGain(1);
    events.setDriftState(2, 0, 3, 0);
    check(events.seenBoostSerial_ == 3 && events.boostCueTime_ == -1 && events.chargeCueTime_ == -1,
          "the zero-audio candidate also consumes its events rather than queuing a later sound");
}

void windBed() {
    DriftAudio wind;
    wind.setLayerGains(0, 0);
    wind.reset();
    wind.setWindGain(0.6f);
    wind.update(0, 1, 0, 0, false);
    check(isSilence(renderSeconds(wind, 0.3)), "enabled wind is silent while stationary even with the throttle held");
    wind.update(12, 0, 0, 0, false);
    const auto onset = renderSeconds(wind, 0.025);
    renderSeconds(wind, 0.5);
    const auto slow = renderSeconds(wind, 0.25);
    wind.update(33.3f, 0, 0, 0, false);
    renderSeconds(wind, 0.5);
    const auto fast = renderSeconds(wind, 0.25);
    wind.update(51, 0, 0, 0, false, 33.3f);
    renderSeconds(wind, 0.5);
    const auto boosted = renderSeconds(wind, 0.25);
    check(rms(onset) < rms(slow) * 0.7 && rms(slow) > 0.0003 &&
          rms(fast) > rms(slow) * 3 && rms(boosted) > rms(fast) * 1.5,
          "wind attacks smoothly and follows real road speed including boost independently of throttle and drivetrain RPM");
    check(differenceRms(fast) < rms(fast) && differenceRms(fast) > rms(fast) * 0.1,
          "wind output is a bounded broad filtered-noise bed rather than raw high-frequency white noise");
    wind.update(0, 0, 0, 0, false);
    const float oldLevel = wind.windLevel_;
    renderSeconds(wind, 0.3);
    check(std::fabs(wind.windLevel_ - oldLevel * 0.1f) < 0.00001f,
          "wind loses90percent of its held level in the300ms release instead of stepping when speed falls");
    wind.update(51, 1, 0, 0, false);
    renderSeconds(wind, 0.2);
    wind.setWindGain(0);
    check(isSilence(renderSeconds(wind, 0.02)) && wind.windLevel_ == 0,
          "zero wind gain removes only the wind layer immediately and completely");
    wind.setWindGain(std::numeric_limits<float>::quiet_NaN());
    check(wind.windGain_.load() == 0, "nonfinite wind gain is sanitized to silence");
    wind.setWindGain(100);
    check(wind.windGain_.load() == 1.5f, "wind gain has a bounded maximum before the callback");
    wind.setMuted(true);
    renderSeconds(wind, 0.3);
    check(isSilence(renderSeconds(wind, 0.1)) && wind.windLevel_ == 0,
          "mute suppresses the wind bed without a lingering wind envelope");
    wind.setMuted(false);
    wind.update(51, 1, 0, 0, true);
    renderSeconds(wind, 0.3);
    check(isSilence(renderSeconds(wind, 0.1)) && wind.windLevel_ == 0, "pause also removes wind and its envelope");

    DriftAudio whole, chunks;
    for (DriftAudio* audio : {&whole, &chunks}) {
        audio->setLayerGains(0, 0);
        audio->setWindGain(0.6f);
        audio->reset();
        audio->update(45, 0, 0, 0, false);
    }
    const auto single = renderSeconds(whole, 0.5);
    std::vector<Sint16> split;
    for (int block=0; block<10; ++block) {
        const auto pcm = renderSeconds(chunks, 0.05);
        split.insert(split.end(), pcm.begin(), pcm.end());
    }
    check(single == split, "wind synthesis is deterministic across callback block boundaries");
    whole.reset();
    whole.update(45, 0, 0, 0, false);
    check(single == renderSeconds(whole, 0.5), "reset clears wind filter noise and envelope history deterministically");
}

void landingCues() {
    DriftAudio audio;
    audio.setLayerGains(0, 0);
    audio.reset();
    audio.update(25, 0, 0, 0, false);
    audio.setLandingState(20, 6);
    check(isSilence(renderSeconds(audio, 0.2)), "joining a nonzero landing serial does not replay a past touchdown");
    audio.setLandingState(21, 6);
    const auto landing = renderSeconds(audio, 0.04);
    const float age = audio.landingCueTime_;
    audio.setLandingState(21, 30);
    renderSeconds(audio, 0.02);
    check(rms(landing) > 0.01 && audio.landingCueTime_ > age + 0.019f && audio.landingCueImpact_ == 0.6f,
          "one new landing serial emits one brief thud and held telemetry cannot restart or strengthen it");
    renderSeconds(audio, 0.2);
    check(isSilence(renderSeconds(audio, 0.05)), "the finite landing transient ends without becoming a continuous rumble");
    audio.setLandingState(22, 0);
    check(isSilence(renderSeconds(audio, 0.05)), "zero-scaled landing consumes its event silently");
    audio.setLandingState(22, 9);
    check(isSilence(renderSeconds(audio, 0.05)), "turning impact scale back up cannot revive the same zero-scaled landing");
    audio.setLandingState(23, 0.2f);
    audio.setLandingState(24, std::numeric_limits<float>::quiet_NaN());
    check(isSilence(renderSeconds(audio, 0.05)), "tiny support contacts and invalid impact values cannot make false landing hits");

    auto suppressed = [&](int mode, unsigned serial) {
        if (mode==0) audio.setMuted(true);
        else if (mode==1) audio.update(25, 0, 0, 0, true);
        else audio.setGain(0);
        audio.setLandingState(serial, 8);
        renderSeconds(audio, 0.2);
        if (mode==0) audio.setMuted(false);
        else if (mode==1) audio.update(25, 0, 0, 0, false);
        else audio.setGain(1);
        audio.setLandingState(serial, 8);
        check(isSilence(renderSeconds(audio, 0.05)) && audio.landingCueTime_ == -1,
              "a touchdown observed while muted paused or zero-gain is consumed without a stale resume cue");
    };
    suppressed(0, 25); suppressed(1, 26); suppressed(2, 27);
    audio.setLandingState(28, 8);
    audio.setMuted(true);
    audio.setMuted(false);
    check(isSilence(renderSeconds(audio, 0.05)), "muting clears a queued landing even if unmuted before the next callback");
    audio.setLandingState(29, 8);
    audio.reset();
    audio.setLandingState(0, 0);
    check(isSilence(renderSeconds(audio, 0.05)) && audio.landingCueTime_ == -1,
          "reset removes queued and active landing tails and rebases the new trial serial");
    audio.setLandingState(1, 5);
    check(rms(renderSeconds(audio, 0.05)) > 0.01, "a genuine new landing remains audible after reset");

    DriftAudio light, hard;
    for (DriftAudio* candidate : {&light, &hard}) {
        candidate->setLayerGains(0, 0);
        candidate->reset();
        candidate->update(0, 0, 0, 0, false);
        candidate->setLandingState(0, 0);
        renderSeconds(*candidate, 0.01);
    }
    light.setLandingState(1, 3); hard.setLandingState(1, 6);
    check(std::fabs(rms(renderSeconds(hard, 0.06)) / rms(renderSeconds(light, 0.06)) - 2) < 0.01,
          "landing impact scales the same bounded transient without changing its duration or pitch");
    hard.setLandingState(2, std::numeric_limits<float>::infinity());
    renderSeconds(hard, 0.3);
    hard.setLandingState(3, 1000);
    const auto bounded = renderSeconds(hard, 0.2);
    check(hard.landingCueImpact_ <= 1 && std::all_of(bounded.begin(), bounded.end(), [](Sint16 v) { return std::abs(int(v)) < 10000; }),
          "extreme landing impacts remain finite and retain ample output headroom");
}

std::vector<std::uint8_t> waveFixture(int hz, int sourceRate=rate, int sourceChannels=channels,
                                    int frames=4410, bool opposingStereo=false) {
    // Valid little-endian PCM16 WAV data exercises the real decoder/converter
    // without depending on game assets or writing files into the workspace.
    std::vector<std::uint8_t> wav;
    auto text=[&](const char* value) { for (int i=0;i<4;++i) wav.push_back(static_cast<std::uint8_t>(value[i])); };
    auto u16=[&](unsigned value) { wav.push_back(value&255); wav.push_back((value>>8)&255); };
    auto u32=[&](unsigned value) { for (int i=0;i<4;++i) wav.push_back((value>>(i*8))&255); };
    const unsigned bytes=frames*sourceChannels*sizeof(Sint16);
    text("RIFF"); u32(36+bytes); text("WAVE"); text("fmt "); u32(16);
    u16(1); u16(sourceChannels); u32(sourceRate); u32(sourceRate*sourceChannels*sizeof(Sint16));
    u16(sourceChannels*sizeof(Sint16)); u16(16); text("data"); u32(bytes);
    for (int frame=0;frame<frames;++frame) {
        const Sint16 value=static_cast<Sint16>(6000*std::sin(6.283185307179586*hz*frame/sourceRate));
        for (int channel=0;channel<sourceChannels;++channel)
            u16(static_cast<std::uint16_t>(opposingStereo && (channel&1)?-value:value));
    }
    return wav;
}

SDL_RWops* fixtureStream(const std::vector<std::uint8_t>& wav) {
    return SDL_RWFromConstMem(wav.data(),static_cast<int>(wav.size()));
}

void engineRevs() {
    if (Mix_OpenAudio(rate,AUDIO_S16SYS,channels,512)!=0) {
        check(false,"a dummy mixer opens for recorded engine acceleration fixtures");
        return;
    }
    const auto wave=waveFixture(220);
    auto prepare=[&](DriftAudio& audio,float scale=1) {
        audio.loadSampleLoops(fixtureStream(wave),nullptr);
        audio.setLayerGains(1,0);
        audio.setRevProfile(scale,.65f);
        audio.reset();
        audio.update(80/3.6f,1,0,0,false,80/3.6f,0);
        renderSeconds(audio,.5);
    };
    DriftAudio rev;
    prepare(rev);
    const auto base=renderSeconds(rev,.2);
    const float baseRate=rev.enginePlaybackRate_;
    rev.update(80/3.6f,1,0,0,false,80/3.6f,20);
    renderSeconds(rev,.12);
    check(std::fabs(rev.engineRevLevel_-.9f)<.001f && rev.enginePlaybackRate_>baseRate+.19f,
          "actual acceleration reaches90percent of a separate recorded-engine pitch flare in120ms");
    renderSeconds(rev,.18);
    const auto accelerated=renderSeconds(rev,.2);
    check(fundamental(accelerated,250,500)>fundamental(base,250,500)+35 && rms(accelerated)>rms(base),
          "the existing recording gains a measurable temporary pitch rise and restrained level accent during acceleration");
    const float held=rev.engineRevLevel_;
    rev.update(80/3.6f,1,0,0,false,80/3.6f,0);
    renderSeconds(rev,.65);
    check(std::fabs(rev.engineRevLevel_-held*.1f)<.001f && rev.enginePlaybackRate_<baseRate+.025f,
          "constant-speed travel lets the accent lose90percent in the configured650ms and return toward its sustained bed");
    renderSeconds(rev,2);
    rev.update(120/3.6f,1,0,0,false,120/3.6f,0);
    renderSeconds(rev,.5);
    const float at120=rev.enginePlaybackRate_;
    rev.update(160/3.6f,1,0,0,false,160/3.6f,0);
    renderSeconds(rev,.5);
    check(at120>1.63f && rev.enginePlaybackRate_>at120+.07f && rev.enginePlaybackRate_<=1.72001f,
          "enabled rev profile retains a modest sustained pitch increase from120 through160kmh instead of flattening at100");

    DriftAudio disabled,legacy;
    prepare(disabled,0); prepare(legacy,0);
    disabled.update(160/3.6f,1,0,0,false,160/3.6f,100);
    legacy.update(160/3.6f,1,0,0,false,160/3.6f);
    check(renderSeconds(disabled,.2)==renderSeconds(legacy,.2) && disabled.engineRevLevel_==0,
          "zero rev scale restores exactly the previous recorded-engine flow while retaining its audible engine bed");
    check(rms(renderSeconds(disabled,.05))>.01,"turning rev scale to zero does not mute the engine");

    for (int condition=0;condition<4;++condition) {
        rev.reset();
        const float speed=condition==0?0:30;
        const float throttle=condition==1?0:1;
        const float acceleration=condition==2?-20:condition==3?0:20;
        rev.setDriftState(0,0,0,0);
        rev.update(speed,throttle,0,0,false,speed,acceleration);
        renderSeconds(rev,.2);
        check(rev.engineRevLevel_==0,"stationary revving no-load travel braking or an airborne/constant-speed zero-acceleration input cannot start a flare");
    }
    rev.update(45,0,0,0,false,30,20);
    rev.setDriftState(0,3,1,.65f);
    renderSeconds(rev,.12);
    check(rev.engineRevLevel_>.89f,"a real accelerating boost may flare the engine independently of drivetrain speed or throttle load");
    rev.setMuted(true);
    renderSeconds(rev,.1);
    check(rev.engineRevLevel_==0 && isSilence(renderSeconds(rev,.2)),"mute clears an active rev accent with settled exact silence");
    rev.setMuted(false);
    rev.update(30,1,0,0,true,30,20);
    renderSeconds(rev,.1);
    check(rev.engineRevLevel_==0 && rev.acceleration_.load()==0,"pause suppresses actual-acceleration input and clears the rev envelope");
    rev.setDriftState(0,0,1,0);
    rev.reset();
    rev.update(80/3.6f,1,0,0,false,80/3.6f,0);
    DriftAudio fresh;
    fresh.loadSampleLoops(fixtureStream(wave),nullptr);fresh.setLayerGains(1,0);fresh.setRevProfile(1,.65f);fresh.reset();
    fresh.update(80/3.6f,1,0,0,false,80/3.6f,0);fresh.setDriftState(0,0,1,0);
    check(renderSeconds(rev,.3)==renderSeconds(fresh,.3) && rev.engineRevLevel_==0,
          "reset clears prior pitch-flare and sample history without leaking acceleration from the previous run");
    rev.setRevProfile(100,.001f);
    check(rev.engineRevScale_.load()==2 && rev.engineRevReturn_.load()==.2f,"live rev strength and return duration have finite bounded limits");
    rev.update(120,1,1,0,false,90,1000);
    const auto extreme=renderSeconds(rev,.5);
    check(rev.acceleration_.load()==100 && std::isfinite(rev.enginePlaybackRate_) && rev.enginePlaybackRate_<=2.16001f &&
          std::all_of(extreme.begin(),extreme.end(),[](Sint16 v){return v>=-29203&&v<=29203;}),
          "extreme valid acceleration and profile values keep playback finite and bounded with the established output limit");
    rev.setRevProfile(std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::infinity());
    rev.update(30,1,0,0,false,30,std::numeric_limits<float>::quiet_NaN());
    renderSeconds(rev,.1);
    check(rev.engineRevScale_.load()==0 && rev.acceleration_.load()==0 && rev.engineRevLevel_==0,
          "nonfinite rev controls cannot poison the playback rate or create a false acceleration cue");
    Mix_CloseAudio();
}

void recordedLoops() {
    if (Mix_OpenAudio(rate,AUDIO_S16SYS,channels,512)!=0) {
        check(false,"a dummy mixer opens for real WAV decoder and conversion fixtures");
        return;
    }
    const auto engine=waveFixture(220,rate,channels,4410,true);
    const auto tire=waveFixture(1500);
    auto load=[&](DriftAudio& audio) { audio.loadSampleLoops(fixtureStream(engine),fixtureStream(tire)); };
    DriftAudio recorded;
    load(recorded);
    check(recorded.engineSampleLoaded() && recorded.tireSampleLoaded() && recorded.enginePcm_.size()==8820 && recorded.tirePcm_.size()==8820,
          "recorded engine and tire beds use real decoded PCM fixtures and publish independent successful load status");
    recorded.update(0,0,0,0,false);
    renderSeconds(recorded,.5);
    const auto idle=renderSeconds(recorded,.25);
    const double idleHz=fundamental(idle,100,400);
    check(std::fabs(idleHz-165)<3 && std::fabs(recorded.engineSampleRate_-.75f)<.001f,
          "the loaded engine's recognizable220Hz source is interpolated at its bounded0.75x idle playback rate");
    bool preservesStereo=true;
    for (std::size_t i=0;i<idle.size();i+=channels)
        preservesStereo &= std::abs(int(idle[i])+int(idle[i+1]))<=1;
    check(preservesStereo,"sampled engine playback preserves its source channel image rather than replacing it with a centered oscillator");
    recorded.update(100/3.6f,1,0,0,false);
    renderSeconds(recorded,.5);
    const auto power=renderSeconds(recorded,.25);
    const double powerHz=fundamental(power,250,450);
    check(std::fabs(powerHz-352)<3 && recorded.engineSampleRate_<=1.60001f && rms(power)>rms(idle)*2.5,
          "speed and throttle smoothly raise the recorded engine pitch and load level without exceeding its1.6x ceiling");
    recorded.update(90,1,0,0,false);
    renderSeconds(recorded,.3);
    check(recorded.engineSampleRate_<=1.60001f,
          "very high road or drivetrain speed cannot make the selected engine loop excessively high pitched");

    DriftAudio engineOnly,tireOnly;
    load(engineOnly); load(tireOnly);
    engineOnly.setLayerGains(1,0);
    tireOnly.setLayerGains(0,1);
    for (DriftAudio* audio : {&engineOnly,&tireOnly}) {
        audio->update(22.2f,1,1,0,false);
        renderSeconds(*audio,.6);
    }
    const auto engineBed=renderSeconds(engineOnly,.25),tireBed=renderSeconds(tireOnly,.25);
    check(fundamental(engineBed,250,450)>300 && fundamental(tireBed,1300,1800)>1450 &&
          rms(engineBed)>.01 && rms(tireBed)>.02 && tireOnly.tireSampleRate_>=.95f && tireOnly.tireSampleRate_<=1.05001f,
          "independent layer gains select the recorded engine or tire timbre, with only a restrained tire playback-rate change");
    tireOnly.update(22.2f,1,0,0,false);
    const float heldLevel=tireOnly.driftLevel_;
    renderSeconds(tireOnly,.180);
    check(std::fabs(tireOnly.driftLevel_-heldLevel*.1f)<.001f,
          "a sampled tire uses the same180ms ordinary release envelope as the original tire bed");
    tireOnly.update(22.2f,1,1,0,false);
    renderSeconds(tireOnly,.3);
    tireOnly.update(22.2f,1,0,0,false);
    tireOnly.setDriftState(0,3,1,.65f);
    const float boostHeld=tireOnly.driftLevel_;
    renderSeconds(tireOnly,.075);
    check(std::fabs(tireOnly.driftLevel_-boostHeld*.1f)<.001f,
          "a sampled tire also retains the75ms recovery for a valid boosted exit");

    const auto monoWave=waveFixture(200,22050,1,2205);
    DriftAudio converted;
    converted.loadSampleLoops(fixtureStream(monoWave),nullptr);
    check(converted.engineSampleLoaded() && !converted.tireSampleLoaded() && converted.enginePcm_.size()>=8800 && converted.enginePcm_.size()<=8840,
          "the initialization loader converts mono22050Hz recordings into the actual44100Hz stereo mixer format");
    bool duplicated=true;
    for (std::size_t i=0;i+1<converted.enginePcm_.size();i+=channels)
        duplicated &= converted.enginePcm_[i]==converted.enginePcm_[i+1];
    check(duplicated,"converted mono sample channels are valid stereo PCM before callback playback begins");
    const std::vector<std::uint8_t> broken{'n','o','t',' ','a','u','d','i','o'};
    const auto tooShort=waveFixture(200,rate,channels,1);
    DriftAudio fallback,plain;
    fallback.loadSampleLoops(nullptr,fixtureStream(broken));
    check(!fallback.engineSampleLoaded() && !fallback.tireSampleLoaded(),
          "missing or malformed optional recordings select synthesis independently and cannot prevent audio startup");
    for (DriftAudio* audio : {&fallback,&plain}) audio->update(22.2f,.7f,.6f,0,false);
    check(renderSeconds(fallback,.3)==renderSeconds(plain,.3),
          "failed optional loads produce exactly the established synthesis fallback at the same controls");
    fallback.loadSampleLoops(fixtureStream(tooShort),fixtureStream(tire));
    check(!fallback.engineSampleLoaded() && fallback.tireSampleLoaded(),
          "an unusable one-frame engine loop falls back while a valid tire recording still loads");
    const std::vector<float> boundary{0,.5f,1,-.5f};
    check(std::fabs(loopSample(boundary,2,2,1.5,0)-.5f)<.00001f &&
          std::fabs(loopSample(boundary,2,2,1.5,1))<.00001f && std::fabs(advanceLoop(1.5,1.6f,2)-1.1)<.00001,
          "PCM interpolation crosses the last-to-first frame correctly and wraps fractional playback positions");

    DriftAudio events,bed;
    load(events); load(bed);
    for (DriftAudio* audio : {&events,&bed}) {
        audio->update(22.2f,.7f,.3f,0,false);
        audio->setDriftState(0,0,0,0);
        renderSeconds(*audio,.3);
    }
    events.setDriftState(3,3,1,1.3f);
    events.bank(.7f);
    check(rms(difference(renderSeconds(events,.06),renderSeconds(bed,.06)))>.01,
          "recorded beds leave the existing charge, boost and success voices audible through their established control path");
    events.setMuted(true);
    renderSeconds(events,.3);
    check(isSilence(renderSeconds(events,.1)),"sample-backed engine and tire beds retain exact settled mute silence");
    events.setMuted(false);
    events.update(22.2f,.7f,.3f,0,true);
    renderSeconds(events,.3);
    check(isSilence(renderSeconds(events,.5)),"sample-backed pause remains silent and cannot replay a stale transient");
    events.setDriftState(0,0,1,0);
    events.reset();
    events.update(22.2f,.4f,.25f,0,false);
    DriftAudio fresh;
    load(fresh);
    fresh.setDriftState(0,0,1,0);
    fresh.update(22.2f,.4f,.25f,0,false);
    check(renderSeconds(events,.4)==renderSeconds(fresh,.4),
          "resetting recorded playback restores sample positions, smoothed rates and envelopes deterministically without reloading files");

    DriftAudio whole,chunks;
    load(whole); load(chunks);
    for (DriftAudio* audio : {&whole,&chunks}) audio->update(22.2f,.8f,.9f,0,false);
    const auto single=renderSeconds(whole,1);
    std::vector<Sint16> split;
    for (int block=0;block<10;++block) {
        const auto pcm=renderSeconds(chunks,.1);
        split.insert(split.end(),pcm.begin(),pcm.end());
    }
    check(single==split,"recorded-loop PCM remains deterministic across arbitrary callback block boundaries and many wraps");
    const auto engineCapacity=chunks.enginePcm_.capacity(),tireCapacity=chunks.tirePcm_.capacity();
    chunks.setGain(1.5f);
    chunks.setLayerGains(1.5f,1.5f);
    for (int block=0;block<50;++block) {
        chunks.update(block%2?90:22.2f,1,1,0,false);
        const auto pcm=renderSeconds(chunks,.2);
        check(std::all_of(pcm.begin(),pcm.end(),[](Sint16 value){return value>=-29203 && value<=29203;}) &&
              std::isfinite(chunks.engineSamplePosition_) && std::isfinite(chunks.tireSamplePosition_) &&
              chunks.engineSamplePosition_>=0 && chunks.engineSamplePosition_<chunks.enginePcm_.size()/channels &&
              chunks.tireSamplePosition_>=0 && chunks.tireSamplePosition_<chunks.tirePcm_.size()/channels,
              "extended sample-loop playback remains finite, wraps within PCM bounds and retains the existing output headroom limit");
    }
    check(chunks.enginePcm_.capacity()==engineCapacity && chunks.tirePcm_.capacity()==tireCapacity,
          "rendering never changes or reallocates the immutable decoded sample storage");
    chunks.setLayerGains(std::numeric_limits<float>::quiet_NaN(),100);
    check(chunks.engineGain_.load()==0 && chunks.tireGain_.load()==1.5f,
          "live layer gains sanitize nonfinite values and clamp before reaching the audio callback");
    chunks.setLayerGains(0,0);
    chunks.update(std::numeric_limits<float>::infinity(),std::numeric_limits<float>::quiet_NaN(),
                  std::numeric_limits<float>::infinity(),0,false,std::numeric_limits<float>::quiet_NaN());
    renderSeconds(chunks,1);
    check(isSilence(renderSeconds(chunks,.1)) && std::isfinite(chunks.engineSampleRate_) && std::isfinite(chunks.tireSampleRate_),
          "nonfinite controls cannot poison recorded playback and zero layer gains settle to exact silence");
    chunks.reset();
    chunks.update(22.2f,1,1,0,false);
    check(isSilence(renderSeconds(chunks,.060)) && chunks.engineMixGain_==0 && chunks.tireMixGain_==0,
          "reset with both recorded layer gains at zero cannot leak an engine or tire transient on the first callback");
    DriftAudio silentFallback;
    silentFallback.setLayerGains(0,0);
    silentFallback.update(22.2f,1,1,0,false);
    silentFallback.reset();
    check(isSilence(renderSeconds(silentFallback,.060)) && silentFallback.engineMixGain_==0 && silentFallback.tireMixGain_==0,
          "reset also preserves zero individual layer gains immediately when optional recordings use synthesis fallback");
    recorded.shutdown();
    check(!recorded.engineSampleLoaded() && !recorded.tireSampleLoaded() && recorded.enginePcm_.empty() && recorded.tirePcm_.empty(),
          "sample shutdown releases decoded memory only after callback detachment and clears public status");
    Mix_CloseAudio();
    std::printf("recorded PCM fixture: idle %.0fHz / loaded %.0fHz; preserved stereo, converted format, smooth layer gains, fallback and wrap checks\n",idleHz,powerHz);
}
} // namespace

int main() {
    SDL_setenv("SDL_AUDIODRIVER", "dummy", 1);
    if (SDL_Init(SDL_INIT_AUDIO | SDL_INIT_TIMER) != 0) {
        std::fprintf(stderr, "SDL_Init: %s\n", SDL_GetError());
        return EXIT_FAILURE;
    }
    lifecycle();
    envelopes();
    loadAndSlip();
    silenceAndHeadroom();
    controlBoundsAndDeterminism();
    boostedDrivetrainPitch();
    boostedTireRecovery();
    boostKickAndAirTail();
    pressureReadiness();
    resetIsolation();
    driftStateCues();
    driftCueSuppressionAndReset();
    suppressedEventsNeverReturn();
    windBed();
    landingCues();
    recordedLoops();
    engineRevs();
    Mix_Quit();
    SDL_Quit();
    if (failures) return EXIT_FAILURE;
    std::puts("audio tests passed: SDL lifecycle,60ms tire attack,180ms/75ms recovery,drivetrain pitch,acceleration rev flare,load/slip spectrum,finite kick+air tail,speed wind,one-shot landings,headroom,mute/pause,no stale cues,bounds,replay determinism,reset isolation");
    return EXIT_SUCCESS;
}
