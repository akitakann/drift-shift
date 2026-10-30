"""Small offline synthesis kit for the drift-shift sound effects.

Everything is float64 numpy at 44.1 kHz. Mono voices are 1-D arrays; a mix is
(n, 2). Nothing here is random unless a seed is passed, so renders repeat.
"""
import numpy as np
from scipy import signal
from scipy.io import wavfile

SR = 44100


def n_of(seconds):
    return int(round(seconds * SR))


def times(seconds):
    return np.arange(n_of(seconds)) / SR


def note(name):
    """'A5' -> Hz, equal temperament with A4 = 440."""
    steps = {"C": -9, "D": -7, "E": -5, "F": -4, "G": -2, "A": 0, "B": 2}
    letter, rest = name[0], name[1:]
    shift = 0
    while rest and rest[0] in "#b":
        shift += 1 if rest[0] == "#" else -1
        rest = rest[1:]
    octave = int(rest)
    return 440.0 * 2 ** ((steps[letter] + shift + 12 * (octave - 4)) / 12)


# ---------------------------------------------------------------- oscillators
def phase(freq):
    """Cycles (not radians) for a per-sample frequency array."""
    return np.cumsum(np.asarray(freq, dtype=float) / SR)


def sine(freq, n=None, start_phase=0.0):
    freq = np.broadcast_to(np.asarray(freq, dtype=float), (n,) if n else np.shape(freq))
    return np.sin(2 * np.pi * (phase(freq) + start_phase))


def _blep(p, dt):
    y = np.zeros_like(p)
    m = p < dt
    x = p[m] / dt[m]
    y[m] = x + x - x * x - 1
    m = p > 1 - dt
    x = (p[m] - 1) / dt[m]
    y[m] = x * x + x + x + 1
    return y


def saw(freq, n=None, start_phase=0.0):
    freq = np.broadcast_to(np.asarray(freq, dtype=float), (n,) if n else np.shape(freq)).astype(float)
    dt = freq / SR
    p = (np.cumsum(dt) + start_phase) % 1.0
    return 2 * p - 1 - _blep(p, dt)


def pulse(freq, duty=0.5, n=None, start_phase=0.0):
    freq = np.broadcast_to(np.asarray(freq, dtype=float), (n,) if n else np.shape(freq)).astype(float)
    dt = freq / SR
    p = (np.cumsum(dt) + start_phase) % 1.0
    y = np.where(p < duty, 1.0, -1.0)
    y = y + _blep(p, dt) - _blep((p - duty) % 1.0, dt)
    return y - (2 * duty - 1)  # remove DC of an asymmetric pulse


def fm(carrier, ratio, index, n):
    """Two-operator FM. index may be an envelope array."""
    carrier = np.broadcast_to(np.asarray(carrier, dtype=float), (n,))
    mod = np.sin(2 * np.pi * phase(carrier * ratio))
    return np.sin(2 * np.pi * phase(carrier) + np.broadcast_to(index, (n,)) * mod)


def noise(n, seed):
    return np.random.default_rng(seed).standard_normal(n)


# ------------------------------------------------------------------ envelopes
def env_exp(n, attack, decay, hold=0.0):
    t = np.arange(n) / SR
    a = np.clip(t / max(attack, 1e-5), 0, 1)
    d = np.where(t < attack + hold, 1.0, np.exp(-(t - attack - hold) / decay))
    return a * d


def fade_tail(x, seconds=0.02):
    m = min(len(x), n_of(seconds))
    if m > 0:
        ramp = np.linspace(1, 0, m)
        x = x.copy()
        x[-m:] = x[-m:] * (ramp[:, None] if x.ndim == 2 else ramp)
    return x


def sweep_exp(n, start, end, tau):
    """Exponential glide from start toward end with time constant tau."""
    t = np.arange(n) / SR
    return end + (start - end) * np.exp(-t / tau)


# -------------------------------------------------------------------- filters
def svf(x, fc, q=0.707, mode="lp"):
    """Zavalishin TPT state-variable filter with per-sample cutoff."""
    x = np.asarray(x, dtype=float)
    fc = np.broadcast_to(np.asarray(fc, dtype=float), x.shape)
    g = np.tan(np.pi * np.clip(fc, 20, SR * 0.45) / SR)
    k = 1.0 / q
    a1 = 1 / (1 + g * (g + k))
    a2 = g * a1
    a3 = g * a2
    out = np.empty_like(x)
    ic1 = ic2 = 0.0
    for i in range(len(x)):
        v3 = x[i] - ic2
        v1 = a1[i] * ic1 + a2[i] * v3
        v2 = ic2 + a2[i] * ic1 + a3[i] * v3
        ic1 = 2 * v1 - ic1
        ic2 = 2 * v2 - ic2
        if mode == "lp":
            out[i] = v2
        elif mode == "bp":
            out[i] = v1 * k  # unity peak gain
        else:
            out[i] = x[i] - k * v1 - v2
    return out


def butter(x, fc, btype="low", order=2):
    sos = signal.butter(order, fc, btype=btype, fs=SR, output="sos")
    return signal.sosfilt(sos, x, axis=0)


def comb_flange(x, delay_ms, feedback=0.4, mix=0.5):
    """Short modulated delay; delay_ms is a per-sample array."""
    n = len(x)
    d = np.asarray(delay_ms, dtype=float) * SR / 1000
    y = np.zeros(n)
    buf = np.zeros(n)
    for i in range(n):
        pos = i - d[i]
        j = int(np.floor(pos))
        frac = pos - j
        delayed = 0.0
        if j >= 1:
            delayed = buf[j] * (1 - frac) + buf[j + 1 if j + 1 < i else j] * frac
        buf[i] = x[i] + feedback * delayed
        y[i] = x[i] * (1 - mix) + delayed * mix
    return y


def drive(x, amount):
    return np.tanh(amount * x) / np.tanh(amount)


# ----------------------------------------------------------------- space/mix
def pan(mono, position):
    """Equal-power pan, position -1 (left) .. 1 (right)."""
    angle = (np.clip(position, -1, 1) + 1) * np.pi / 4
    return np.stack([mono * np.cos(angle), mono * np.sin(angle)], axis=1) * np.sqrt(2)


def stereo(mono):
    return np.stack([mono, mono], axis=1)


def place(mix, voice, at):
    """Adds voice (mono or stereo) into mix starting at `at` seconds."""
    start = n_of(at)
    if voice.ndim == 1:
        voice = stereo(voice)
    voice = fade_tail(voice, 0.005)  # no voice may stop on a click
    end = min(len(mix), start + len(voice))
    if end > start:
        mix[start:end] += voice[: end - start]
    return mix


def make_ir(t60=0.8, seconds=None, seed=11, bright=1.0, predelay=0.012):
    seconds = seconds or t60 * 1.1
    n = n_of(seconds)
    t = np.arange(n) / SR
    ir = np.zeros((n, 2))
    bands = [(80, 400, 1.15), (400, 1500, 1.0), (1500, 5000, 0.8 * bright), (5000, 14000, 0.55 * bright)]
    for ch in range(2):
        acc = np.zeros(n)
        for b, (lo, hi, k) in enumerate(bands):
            src = noise(n, seed + 17 * ch + b)
            acc += butter(src, [lo, hi], "band") * np.exp(-6.9 * t / (t60 * k))
        ir[:, ch] = acc
    ramp = n_of(0.006)
    ir[:ramp] *= np.linspace(0, 1, ramp)[:, None]
    ir = np.concatenate([np.zeros((n_of(predelay), 2)), ir])
    return ir / np.sqrt((ir ** 2).sum(axis=0, keepdims=True))


def reverb(mix, ir, wet):
    out = np.zeros((len(mix) + len(ir) - 1, 2))
    for ch in range(2):
        out[:, ch] = signal.fftconvolve(mix[:, ch], ir[:, ch])
    out *= wet
    out[: len(mix)] += mix
    return out


def trim(mix, floor_db=-70):
    """Drops the silent tail below floor_db relative to the peak, then fades."""
    level = np.abs(mix).max(axis=1)
    peak = level.max()
    if peak <= 0:
        return mix[:1]
    above = np.nonzero(level > peak * 10 ** (floor_db / 20))[0]
    end = min(len(mix), above[-1] + n_of(0.01))
    return fade_tail(mix[:end], 0.01)


def rms_db(mix):
    m = mix.mean(axis=1) if mix.ndim == 2 else mix
    active = np.abs(m) > np.abs(m).max() * 10 ** (-40 / 20)
    if not active.any():
        return -120.0
    return 20 * np.log10(np.sqrt((m[active] ** 2).mean()))


def peak_db(mix):
    return 20 * np.log10(np.abs(mix).max() + 1e-12)


def loudness_db(mix, window=0.1):
    """Max RMS over a short window, a stand-in for how loud a one-shot feels."""
    m = mix.mean(axis=1) if mix.ndim == 2 else mix
    w = n_of(window)
    if len(m) <= w:
        return 20 * np.log10(np.sqrt((m ** 2).mean()) + 1e-12)
    c = np.cumsum(np.concatenate([[0], m ** 2]))
    return 10 * np.log10(((c[w:] - c[:-w]) / w).max() + 1e-12)


def to_loudness(mix, target_db, ceiling_db=-3.0):
    mix = mix * 10 ** ((target_db - loudness_db(mix)) / 20)
    ceiling = 10 ** (ceiling_db / 20)
    peak = np.abs(mix).max()
    if peak > ceiling:  # gentle soft clip above the ceiling only
        knee = ceiling * 0.7
        a = np.abs(mix)
        over = a > knee
        mix = mix.copy()
        mix[over] = np.sign(mix[over]) * (knee + (ceiling - knee) * np.tanh((a[over] - knee) / (ceiling - knee)))
    return mix


def write(path, mix):
    pcm = np.clip(mix, -1, 1)
    wavfile.write(path, SR, (pcm * 32767).astype(np.int16))
