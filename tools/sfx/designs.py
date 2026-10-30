"""drift-shift's one-shot effects, synthesized for the game.

    python3 -I tools/sfx/designs.py OUT_DIR [--pickup piro|pon|cha]
        writes the game's set, one WAV per effect, named as the game loads them
    python3 -I tools/sfx/designs.py --pickups OUT_DIR
        writes every pickup candidate into its own folder, for auditioning

tools/sfx/build.sh renders the set and encodes it to assets/audio/sfx. The
charge and the boost's flare are not files: the game revs its own engine
recording for them (src/audio.cpp). Loudness targets are short-window RMS in
dBFS; the engine bed in play sits near -26 dB.
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sfxlib import *  # noqa: E402,F401,F403

IR_SMALL = make_ir(0.55, bright=1.2, seed=21)
IR_ROOM = make_ir(0.9, bright=1.0, seed=31)
IR_HALL = make_ir(1.7, bright=0.9, seed=41)

def finish(mix, loudness, ceiling=-3.0):
    return to_loudness(trim(mix), loudness, ceiling)


def sparkle(mix, base, count, start, spread, seed, gain=0.16, lowest=3000.0):
    r = np.random.default_rng(seed)
    for _ in range(count):
        f = base * r.choice([2, 3, 4, 5])
        while f > 14000:
            f /= 2
        while f < lowest:
            f *= 2
        f *= 1 + r.uniform(-0.01, 0.01)
        n = n_of(0.08)
        s = np.sin(2 * np.pi * f * np.arange(n) / SR) * env_exp(n, 0.0008, r.uniform(0.012, 0.03))
        place(mix, pan(s * gain, r.uniform(-0.8, 0.8)), start + r.uniform(0, spread))
    return mix


def epiano(f, seconds, decay, index=1.8, bright=0.3):
    n = n_of(max(seconds, decay * 5.5))  # ring out instead of stopping
    t = np.arange(n) / SR
    idx = index * np.exp(-t / 0.03) + 0.25
    v = fm(f, 1.0, idx, n) + bright * np.sin(2 * np.pi * 2 * f * t) * np.exp(-t / 0.05)
    return fade_tail(v * env_exp(n, 0.0015, decay), 0.01)


def brass(f, seconds, hold=0.15, decay=0.35, bright=5000.0, detune=0.004, seed=0, vibrato=0.0):
    n = n_of(max(seconds, hold + decay * 5.5))
    t = np.arange(n) / SR
    vib = 1 + vibrato * np.clip((t - 0.45) / 0.3, 0, 1) * np.sin(2 * np.pi * 5.5 * t)
    out = np.zeros((n, 2))
    r = np.random.default_rng(seed)
    for k, d in enumerate((-detune, 0.0, detune)):
        v = saw(f * (1 + d) * vib, start_phase=r.random())
        out += pan(v, (k - 1) * 0.6)
    cutoff = 900 + bright * np.exp(-t / 0.16)
    for ch in range(2):
        out[:, ch] = svf(out[:, ch], cutoff, q=0.9, mode="lp")
    amp = np.clip(t / 0.012, 0, 1) * np.where(t < hold, 1.0, np.exp(-(t - hold) / decay))
    return fade_tail(out * amp[:, None] / 3, 0.02)


def crash(seconds, seed, decay, high=3500.0):
    n = n_of(seconds)
    left = butter(noise(n, seed), high, "high")
    right = butter(noise(n, seed + 1), high, "high")
    e = env_exp(n, 0.002, decay)
    return np.stack([left, right], axis=1) * e[:, None]


def thump(seed_level, start_hz=150.0, end_hz=48.0, decay=0.11, amount=2.2, seconds=0.35):
    n = n_of(seconds)
    t = np.arange(n) / SR
    f = end_hz + (start_hz - end_hz) * np.exp(-t / 0.035)
    return drive(np.sin(2 * np.pi * phase(f)), amount) * env_exp(n, 0.0015, decay) * seed_level


# ------------------------------------------------------------------- items
# A cell is taken every quarter to half second, so each sound is short enough
# not to run into the next. The pickup climbs the major scale along a chain;
# the push itself is the engine revving (in the game) and a short rush of air.
MAJOR = [0, 2, 4, 5, 7, 9, 11, 12]


def mallet(seed, cutoff=2600.0, decay=0.004):
    n = n_of(0.03)
    return butter(noise(n, seed), cutoff, "low") * env_exp(n, 0.0004, decay)


def pickup_piro(step):
    """"Piro": two quick notes a fifth apart, a soft square in the fifth octave."""
    f1 = note("D5") * 2 ** (MAJOR[step] / 12)
    mix = np.zeros((n_of(0.45), 2))
    for k, (f, at, decay) in enumerate(((f1, 0.0, 0.045), (f1 * 1.5, 0.055, 0.09))):
        n = n_of(0.3)
        v = butter(pulse(np.full(n, f), 0.35), 3200, "low") * env_exp(n, 0.0015, decay)
        place(mix, pan(fade_tail(v, 0.01) * 0.6, (-0.15, 0.15)[k]), at)
    return finish(reverb(mix, IR_SMALL, 0.08), -21)


def pickup_pon(step):
    """"Pon": one round note that drops into place, like a bubble."""
    base = note("A4") * 2 ** (MAJOR[step] / 12)
    n = n_of(0.35)
    t = np.arange(n) / SR
    f = base * (1 + 0.5 * np.exp(-t / 0.012))
    p = phase(f)
    tone = (np.sin(2 * np.pi * p) + 0.12 * np.sin(6 * np.pi * p)) * env_exp(n, 0.001, 0.07)
    mix = np.zeros((n_of(0.45), 2))
    place(mix, fade_tail(tone, 0.01) * 0.8, 0)
    place(mix, mallet(601 + step, 1800, 0.003) * 0.25, 0)
    return finish(reverb(mix, IR_SMALL, 0.08), -21)


def pickup_cha(step):
    """"Cha": a short metal tick, like a coin landing on a hard counter."""
    lift = 2 ** (MAJOR[step] / 24)
    n = n_of(0.25)
    t = np.arange(n) / SR
    ring = sum(a * np.sin(2 * np.pi * 1150 * lift * r * t) * np.exp(-t / d)
               for r, a, d in ((1.0, 1.0, 0.05), (2.76, 0.55, 0.03), (5.4, 0.3, 0.015)))
    mix = np.zeros((n_of(0.4), 2))
    place(mix, ring * np.clip(t / 0.0008, 0, 1) * 0.5, 0)
    place(mix, butter(noise(n_of(0.03), 611 + step), [2500, 8000], "band") * env_exp(n_of(0.03), 0.0003, 0.005) * 0.8, 0)
    return finish(reverb(mix, IR_SMALL, 0.06), -22)


def whoosh_item():
    """"Shu": a short rush of air rising past the car."""
    n = n_of(0.22)
    t = np.arange(n) / SR
    fc = 700 * (3000 / 700) ** np.clip(t / 0.12, 0, 1)
    air = np.stack([svf(noise(n, 651), fc, 1.2, "bp"), svf(noise(n, 652), fc * 1.06, 1.2, "bp")], axis=1)
    shape = np.clip(t / 0.02, 0, 1) * np.where(t < 0.03, 1.0, np.exp(-(t - 0.03) / 0.05))
    mix = np.zeros((n_of(0.35), 2))
    place(mix, air * shape[:, None], 0)
    return finish(mix, -25)


PICKUPS = {"piro": pickup_piro, "pon": pickup_pon, "cha": pickup_cha}


# ------------------------------------------------------------------- boost
# On a release the game revs its own engine recording up and back (src/audio.cpp).
# These files carry the air and the exhaust around it: a rush of air as the
# boost fires, and a muffled bang then one per level as the revs fall away.
BOOST_SECONDS = {1: 0.7, 2: 0.85, 3: 1.0}  # kFxBoostDuration in src/audio.cpp.


def whoosh_boost():
    """"Go": a deep swell of air past the car, with a little rumble under it."""
    seconds = 1.1
    n = n_of(seconds)
    t = np.arange(n) / SR
    fc = 250 + 1100 * np.clip(t / 0.15, 0, 1) * np.exp(-np.clip(t - 0.15, 0, None) / 0.5)
    air = np.stack([svf(noise(n, 661), fc, 0.9, "bp"), svf(noise(n, 662), fc * 1.05, 0.9, "bp")], axis=1)
    rumble = butter(noise(n, 663), 160, "low")
    shape = np.clip(t / 0.06, 0, 1) ** 2 * (1 - np.clip((t - 0.1) / (seconds - 0.1), 0, 1)) ** 1.6
    mix = np.zeros((n_of(seconds + 0.2), 2))
    place(mix, air * shape[:, None], 0)
    place(mix, rumble * shape * 2.5, 0)
    return finish(mix, -19)


def low_afterfire(seed, size=1.0):
    """A muffled bang: a thud and a low crack."""
    r = np.random.default_rng(seed)
    n = n_of(0.3)
    t = np.arange(n) / SR
    thud = np.sin(2 * np.pi * r.uniform(55, 70) * t) * env_exp(n, 0.0005, 0.06)
    crack = butter(noise(n, seed), [120, 800], "band") * env_exp(n, 0.0003, 0.016)
    return drive(thud + 1.6 * crack, 2.5) * size


def boost_afterfire(level):
    seconds = BOOST_SECONDS[level]
    mix = np.zeros((n_of(seconds + 0.6), 2))
    place(mix, low_afterfire(700 + level, 1.0), 0)
    for k in range(level):
        place(mix, pan(low_afterfire(710 + 10 * level + k, 0.85 - 0.1 * k), (0.3, -0.3, 0.1)[k]), 0.55 * seconds + 0.13 * k)
    return finish(reverb(mix, IR_ROOM, 0.1), -17)


# --------------------------------------------------------------- countdown
def count_tick():
    n = n_of(0.16)
    t = np.arange(n) / SR
    e = env_exp(n, 0.001, 0.05, hold=0.06)
    v = butter(pulse(np.full(n, note("A5")), 0.5), 5000, "low") * e
    v += 0.25 * np.sin(2 * np.pi * note("A6") * t) * e
    mix = stereo(fade_tail(v, 0.01))
    return finish(reverb(mix, IR_ROOM, 0.15), -18)


def count_go():
    mix = np.zeros((n_of(2.2), 2))
    n = n_of(0.85)
    core = butter(pulse(np.full(n, note("A6")), 0.5), 7000, "low") * env_exp(n, 0.001, 0.32, hold=0.12)
    place(mix, fade_tail(core, 0.02) * 0.55, 0)
    for k, name in enumerate(("A4", "C#5", "E5", "A5")):
        place(mix, brass(note(name), 0.9, hold=0.1, decay=0.3, bright=5200, seed=60 + k) * 0.6, 0)
    place(mix, crash(1.0, 61, 0.42) * 0.22, 0)
    place(mix, thump(0.7, start_hz=130, end_hz=50, decay=0.12), 0)
    return finish(reverb(mix, IR_HALL, 0.2), -15)


# ----------------------------------------------------------------- sectors
def sector_best():
    mix = np.zeros((n_of(1.8), 2))
    place(mix, pan(epiano(note("E6"), 0.3, 0.12) * 0.7, -0.2), 0)
    place(mix, pan(epiano(note("B6"), 0.6, 0.28) * 0.8, 0.2), 0.075)
    sparkle(mix, note("B6"), 6, 0.08, 0.15, 900, gain=0.12)
    return finish(reverb(mix, IR_ROOM, 0.2), -18)


def sector_slower():
    mix = np.zeros((n_of(0.5), 2))
    for name, at in (("D5", 0), ("A4", 0.09)):
        n = n_of(0.25)
        t = np.arange(n) / SR
        f = note(name)
        v = (np.sin(2 * np.pi * f * t) + 0.15 * np.sin(2 * np.pi * 2 * f * t)) * env_exp(n, 0.003, 0.07, hold=0.01)
        place(mix, fade_tail(v, 0.01) * 0.7, at)
    mix = butter(mix, 3000, "low")
    return finish(reverb(mix, IR_ROOM, 0.12), -24)


# ------------------------------------------------------------- lap/record
def lap():
    mix = np.zeros((n_of(2.6), 2))
    for k, name in enumerate(("G5", "C6", "E6", "G6")):
        place(mix, pan(epiano(note(name), 0.35, 0.16) * 0.6, -0.4 + 0.27 * k), 0.07 * k)
    for k, name in enumerate(("C6", "E6", "G6", "C7")):
        place(mix, brass(note(name), 0.95, hold=0.18, decay=0.3, bright=4800, seed=80 + k) * 0.5, 0.28)
        place(mix, pan(epiano(note(name), 0.9, 0.35, index=1.2) * 0.22, (-0.6, -0.2, 0.2, 0.6)[k]), 0.28)
    place(mix, crash(0.9, 81, 0.35, high=5000) * 0.18, 0.28)
    sparkle(mix, note("C7"), 9, 0.3, 0.45, 82, gain=0.1)
    return finish(reverb(mix, IR_HALL, 0.22), -16)


def record():
    mix = np.zeros((n_of(4.0), 2))
    for k, name in enumerate(("C6", "D6", "E6", "G6", "A6", "C7")):
        place(mix, pan(epiano(note(name), 0.3, 0.12) * 0.6, -0.5 + 0.2 * k), 0.055 * k)
    at = 0.36
    for k, name in enumerate(("C5", "G5", "C6", "E6", "G6", "D7")):
        place(mix, brass(note(name), 1.8, hold=0.35, decay=0.55, bright=5200, seed=90 + k, vibrato=0.004) * 0.42, at)
    for k, name in enumerate(("C6", "E6", "G6")):
        place(mix, pan(epiano(note(name), 1.4, 0.5, index=1.4) * 0.3, (-0.5, 0, 0.5)[k]), at)
    place(mix, thump(0.9, start_hz=140, end_hz=46, decay=0.14), at)
    place(mix, crash(1.8, 91, 0.9, high=3000) * 0.3, at)
    r = np.random.default_rng(92)
    for k in range(14):
        f = 9000 * (3000 / 9000) ** (k / 13) * (1 + r.uniform(-0.03, 0.03))
        n = n_of(0.1)
        s = np.sin(2 * np.pi * f * np.arange(n) / SR) * env_exp(n, 0.0008, 0.03)
        place(mix, pan(s * 0.14, r.uniform(-0.9, 0.9)), at + 0.05 + k * 0.075)
    return finish(reverb(mix, IR_HALL, 0.25), -14)


def game_set(pickup):
    maker = PICKUPS[pickup]
    return [(f"item_{step}", maker, step) for step in range(8)] + \
        [(f"boost_{level}", boost_afterfire, level) for level in (1, 2, 3)] + \
        [(name, maker, None) for name, maker in (("whoosh_item", whoosh_item), ("whoosh_boost", whoosh_boost),
                                                 ("count_tick", count_tick), ("count_go", count_go),
                                                 ("sector_best", sector_best), ("sector_slower", sector_slower),
                                                 ("lap", lap), ("record", record))]


def save(path, mix):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    write(path, mix)
    print(f"{os.path.basename(path):18s} {len(mix)/SR:5.2f}s  loud {loudness_db(mix):6.1f}  peak {peak_db(mix):6.1f}")


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("out")
    parser.add_argument("--pickup", choices=list(PICKUPS), default="piro")
    parser.add_argument("--pickups", action="store_true")
    args = parser.parse_args()
    if args.pickups:
        for name, maker in PICKUPS.items():
            for step in range(8):
                save(os.path.join(args.out, f"pickup-{name}", f"item_{step}.wav"), maker(step))
    else:
        for name, maker, arg in game_set(args.pickup):
            save(os.path.join(args.out, name + ".wav"), maker() if arg is None else maker(arg))
