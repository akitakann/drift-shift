(function (root, factory) {
  if (typeof module === 'object' && module.exports) {
    module.exports = factory();
    module.exports.DriftBgmPlayer = module.exports;
  } else root.DriftBgmPlayer = factory();
}(typeof globalThis !== 'undefined' ? globalThis : this, function () {
  'use strict';

  // Two reusable streaming media elements. Game restarts never own this clock.
  class DriftBgmPlayer {
    constructor(playlist, options) {
      if (!Array.isArray(playlist) || playlist.length !== 2 || playlist.some(t => !t.url)) {
        throw new TypeError('BGM requires two tracks with URLs');
      }
      const opts = options || {};
      this.tracks = playlist.map(t => ({ id: t.id, title: t.title || t.id || '', url: t.url,
        durationSeconds: t.durationSeconds }));
      this.volume = Math.max(0, Math.min(1, opts.volume === undefined ? 0.35 : opts.volume));
      this.fadeSeconds = Math.max(0, opts.fadeSeconds === undefined ? 2.5 : opts.fadeSeconds);
      this.now = opts.now || (() => performance.now());
      this.raf = opts.requestAnimationFrame || (fn => requestAnimationFrame(fn));
      this.cancel = opts.cancelAnimationFrame || (id => cancelAnimationFrame(id));
      this.onState = opts.onState || (() => {});
      this.stallMs = opts.stallTimeoutMs || 5000;
      this.index = 0;
      this.playing = false;
      this.muted = false;
      this.disposed = false;
      this.fade = null;
      this.frame = null;
      this.lastTick = this.now();
      this.slots = this.tracks.map((track, index) => {
        const audio = opts.createAudio ? opts.createAudio(track.url, index) : new Audio(track.url);
        audio.src = track.url;
        audio.preload = 'auto';
        audio.loop = false;
        const slot = { audio, requested: false, used: false, failed: null, token: 0,
          clock: 0, checkedAt: this.now(), handlers: {} };
        slot.handlers.ended = () => this._ended(index);
        slot.handlers.error = () => this._fail(index, 'media');
        slot.handlers.loadedmetadata = () => this._update();
        slot.handlers.playing = () => this._update();
        Object.keys(slot.handlers).forEach(name => audio.addEventListener(name, slot.handlers[name]));
        return slot;
      });
      this._gains();
      this._emit();
    }

    setPlaying(value) {
      value = !!value;
      if (this.disposed || value === this.playing) return;
      this.playing = value;
      this.lastTick = this.now();
      if (value) {
        this.slots.forEach(slot => {
          if (slot.failed === 'play') slot.failed = null;
          slot.checkedAt = this.now();
          slot.clock = slot.audio.currentTime || 0;
        });
        if (this.slots[this.index].failed) this._advance();
        else if (this.slots[this.index].audio.ended) this._ended(this.index);
        else this._play(this.index);
        if (this.fade) this._play(this.fade.to);
      } else {
        this.slots.forEach((slot, i) => this._pause(i));
        if (this.frame !== null) this.cancel(this.frame);
        this.frame = null;
      }
      this._update();
    }

    setMuted(value) {
      if (this.disposed) return;
      this.muted = !!value;
      this._gains();
      this._emit();
    }

    getState() {
      const slot = this.slots[this.index];
      const status = this.disposed ? 'disposed' : !this.playing ? 'paused'
        : this.slots.every(s => s.failed) ? 'unavailable'
        : this.fade ? 'crossfading' : slot.audio.paused ? 'starting' : 'playing';
      return Object.freeze({ index: this.index, id: this.tracks[this.index].id,
        title: this.tracks[this.index].title, currentTime: slot.audio.currentTime || 0,
        duration: Number.isFinite(this._duration(this.index)) ? this._duration(this.index) : 0,
        mediaDuration: Number.isFinite(slot.audio.duration) ? slot.audio.duration : 0,
        status, fade: this.fade ? this.fade.progress : 0, playing: this.playing, muted: this.muted });
    }

    _emit() { this.onState(this.getState()); }

    _duration(index) {
      const verified = this.tracks[index].durationSeconds;
      // Ogg metadata can report a provisional estimate before the complete stream is read.
      return Number.isFinite(verified) && verified > 0 ? verified : this.slots[index].audio.duration;
    }

    _pause(index) {
      const slot = this.slots[index];
      slot.token++;
      slot.requested = false;
      slot.audio.pause();
    }

    _prepare(index) {
      const slot = this.slots[index];
      if (slot.used || slot.audio.ended) {
        try { slot.audio.currentTime = 0; } catch (_) { /* Metadata may still be loading. */ }
      }
      slot.clock = slot.audio.currentTime || 0;
      slot.checkedAt = this.now();
    }

    _play(index) {
      const slot = this.slots[index];
      if (!this.playing || this.disposed || slot.failed || slot.requested) return;
      slot.requested = true;
      slot.used = true;
      slot.clock = slot.audio.currentTime || 0;
      slot.checkedAt = this.now();
      const token = ++slot.token;
      try {
        const result = slot.audio.play();
        if (result && typeof result.then === 'function') {
          result.then(() => {
            // A late play resolution must not undo pause/dispose or restart a retired track.
            if (this.disposed || !this.playing || (index !== this.index && (!this.fade || index !== this.fade.to))) {
              slot.audio.pause();
            }
          }, () => {
            if (!this.disposed && slot.token === token && this.playing) this._fail(index, 'play');
          });
        }
      } catch (_) { this._fail(index, 'play'); }
    }

    _other(index) {
      const next = (index + 1) % this.slots.length;
      return this.slots[next].failed ? -1 : next;
    }

    _advance() {
      const previous = this.index;
      let next = this._other(previous);
      if (next < 0 && !this.slots[previous].failed) next = previous;
      this.fade = null;
      this._pause(previous);
      if (next >= 0) {
        this.index = next;
        this._prepare(next);
        this._gains();
        this._play(next);
      }
    }

    _promote() {
      const next = this.fade.to;
      this._pause(this.index);
      this.index = next;
      this.fade = null;
      this._gains();
    }

    _ended(index) {
      if (this.disposed || !this.playing || index !== this.index) return;
      if (this.fade && this.slots[this.fade.to].audio.currentTime > this.fade.inStart) this._promote();
      else if (this.fade) this._fail(this.fade.to, 'play');
      else this._advance();
      this._update();
    }

    _fail(index, reason) {
      if (this.disposed || this.slots[index].failed) return;
      this.slots[index].failed = reason;
      this._pause(index);
      if (this.fade && this.fade.to === index) {
        this.fade = null;
        if (this.slots[this.index].audio.ended) this._advance();
      } else if (this.index === index) {
        if (this.fade && !this.slots[this.fade.to].failed) this._promote();
        else this._advance();
      }
      this._gains();
      this._update();
    }

    _gains() {
      const base = this.muted ? 0 : this.volume;
      this.slots.forEach((slot, i) => {
        let gain = i === this.index ? 1 : 0;
        if (this.fade) gain = i === this.index ? Math.cos(this.fade.progress * Math.PI / 2)
          : i === this.fade.to ? Math.sin(this.fade.progress * Math.PI / 2) : 0;
        slot.audio.volume = base * gain;
        slot.audio.muted = this.muted;
      });
    }

    _update() {
      if (this.disposed) return;
      if (this.playing && !this.slots.every(slot => slot.failed)) {
        const now = this.now();
        const interrupted = now - this.lastTick > 1000;
        this.lastTick = now;
        for (let i = 0; i < this.slots.length; ++i) {
          const slot = this.slots[i];
          if (!slot.requested || slot.failed) continue;
          const time = slot.audio.currentTime || 0;
          if (interrupted || time !== slot.clock) { slot.clock = time; slot.checkedAt = now; }
          else if (now - slot.checkedAt >= this.stallMs) { this._fail(i, 'play'); return; }
        }
        const audio = this.slots[this.index].audio;
        if (audio.ended) { this._ended(this.index); return; }
        const duration = this._duration(this.index);
        // Very short/unknown-duration clips use ended instead of overlapping their whole content.
        const length = Number.isFinite(duration) && duration >= 0.5 ? Math.min(this.fadeSeconds, duration / 2) : 0;
        if (!this.fade && length > 0 && duration - audio.currentTime <= length + 1e-6) {
          const next = this._other(this.index);
          if (next >= 0) {
            this._prepare(next);
            this.fade = { to: next, length, progress: 0, outClock: audio.currentTime,
              inClock: this.slots[next].audio.currentTime || 0, inStart: this.slots[next].audio.currentTime || 0 };
            this._play(next);
          }
        }
        if (this.fade) {
          const fade = this.fade;
          const incoming = this.slots[fade.to].audio.currentTime || 0;
          // Only time that both tracks actually played contributes to the overlap.
          const step = Math.min(Math.max(0, audio.currentTime - fade.outClock), Math.max(0, incoming - fade.inClock));
          fade.progress = Math.min(1, fade.progress + step / fade.length);
          fade.outClock = audio.currentTime;
          fade.inClock = incoming;
          if (fade.progress >= 1) this._promote();
        }
        this._gains();
        if (this.frame === null) this.frame = this.raf(() => { this.frame = null; this._update(); });
      }
      this._emit();
    }

    dispose() {
      if (this.disposed) return;
      this.disposed = true;
      this.playing = false;
      if (this.frame !== null) this.cancel(this.frame);
      this.frame = null;
      this.slots.forEach((slot, i) => {
        this._pause(i);
        Object.keys(slot.handlers).forEach(name => slot.audio.removeEventListener(name, slot.handlers[name]));
        slot.audio.removeAttribute('src');
        slot.audio.load();
      });
      this._emit();
    }
  }
  return DriftBgmPlayer;
}));
