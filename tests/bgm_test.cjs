'use strict';

const test = require('node:test');
const assert = require('node:assert/strict');
const DriftBgmPlayer = require('../web/bgm.js');
const BASE_VOLUME = 0.35;
const near = (actual, expected, epsilon = 1e-7) => assert.ok(Math.abs(actual - expected) < epsilon,
  `${actual} should be near ${expected}`);

class FakeAudio {
  constructor(duration) {
    this.duration = duration;
    this.actualDuration = duration;
    this._time = 0;
    this.paused = true;
    this.ended = false;
    this.volume = 1;
    this.muted = false;
    this.playCalls = 0;
    this.pauseCalls = 0;
    this.seeks = [];
    this.listeners = new Map();
    this.mode = 'playing';
    this.loadCalls = 0;
  }
  get currentTime() { return this._time; }
  set currentTime(value) { this.seeks.push(value); this._time = value; this.ended = false; }
  addEventListener(type, handler) {
    if (!this.listeners.has(type)) this.listeners.set(type, new Set());
    this.listeners.get(type).add(handler);
  }
  removeEventListener(type, handler) { this.listeners.get(type).delete(handler); }
  emit(type) { for (const handler of this.listeners.get(type) || []) handler(); }
  play() {
    this.playCalls++;
    if (this.mode === 'reject') return Promise.reject(new Error('play blocked'));
    if (this.mode === 'throw') throw new Error('unsupported playback');
    this.paused = false;
    if (this.mode === 'pending') return new Promise(resolve => { this.resolvePlay = resolve; });
    return Promise.resolve();
  }
  pause() { this.pauseCalls++; this.paused = true; }
  removeAttribute(name) { if (name === 'src') this.src = ''; }
  load() { this.loadCalls++; }
  advance(seconds) {
    if (this.paused || this.mode !== 'playing') return;
    this._time += seconds;
    if (Number.isFinite(this.actualDuration) && this._time >= this.actualDuration - 1e-8) {
      this._time = this.actualDuration;
      this.ended = true;
      this.paused = true;
      this.emit('ended');
    }
  }
}

function fixture(options = {}) {
  const audios = [];
  const frames = new Map();
  const states = [];
  let time = 0;
  let id = 0;
  const durations = options.durations || [100, 120];
  const trackDurations = options.trackDurations || [];
  const player = new DriftBgmPlayer([
    { id: 'slam-van', title: 'Slam Van', url: 'music/slam-van.ogg', durationSeconds: trackDurations[0] },
    { id: 'time-bomb', title: 'Time Bomb (remix)', url: 'music/time-bomb-remix.ogg', durationSeconds: trackDurations[1] }
  ], {
    ...options,
    createAudio: (url, i) => { const audio = new FakeAudio(durations[i]); audios.push(audio); return audio; },
    now: () => time,
    requestAnimationFrame: callback => { frames.set(++id, callback); return id; },
    cancelAnimationFrame: handle => frames.delete(handle),
    onState: state => states.push(state)
  });
  function step(seconds, advanceAudio = true) {
    time += seconds * 1000;
    const scheduled = [...frames.values()];
    frames.clear();
    // All clocks advance together before ended events, as real media clocks do.
    const endings = [];
    for (const audio of audios) {
      if (advanceAudio && !audio.paused && audio.mode === 'playing') {
        audio._time += seconds;
        if (Number.isFinite(audio.actualDuration) && audio._time >= audio.actualDuration - 1e-8) {
          audio._time = audio.actualDuration;
          audio.ended = true;
          audio.paused = true;
          endings.push(audio);
        }
      }
    }
    endings.forEach(audio => audio.emit('ended'));
    scheduled.forEach(callback => callback(time));
  }
  function advance(seconds) {
    while (seconds > 1e-8) { const delta = Math.min(0.1, seconds); step(delta); seconds -= delta; }
  }
  return { player, audios, frames, states, advance, step };
}
const settle = async () => { await Promise.resolve(); await Promise.resolve(); };

test('streams full tracks in order, with equal-power tails, then repeats', () => {
  const f = fixture();
  const [a, b] = f.audios;
  assert.equal(f.audios.length, 2);
  assert.equal(a.src, 'music/slam-van.ogg');
  assert.equal(b.src, 'music/time-bomb-remix.ogg');
  assert.equal(a.preload, 'auto');
  assert.equal(f.player.getState().duration, 100);
  assert.equal(f.player.getState().mediaDuration, 100);
  f.player.setPlaying(true);
  f.advance(97.4);
  assert.equal(b.playCalls, 0);
  near(a.volume, BASE_VOLUME);
  f.advance(0.1);
  assert.equal(b.playCalls, 1);
  near(a.currentTime, 97.5);
  near(b.currentTime, 0);
  f.advance(1.25);
  near(f.player.getState().fade, 0.5);
  near(a.volume, BASE_VOLUME / Math.sqrt(2));
  near(b.volume, BASE_VOLUME / Math.sqrt(2));
  near(a.volume ** 2 + b.volume ** 2, BASE_VOLUME ** 2);
  f.advance(1.25);
  assert.equal(f.player.getState().index, 1);
  assert.equal(a.paused, true);
  near(b.currentTime, 2.5);
  near(b.volume, BASE_VOLUME);
  f.advance(115);
  assert.equal(a.playCalls, 2);
  assert.deepEqual(a.seeks, [0]);
  near(a.currentTime, 0);
  f.advance(2.5);
  assert.equal(f.player.getState().index, 0);
  near(a.currentTime, 2.5);
  assert.deepEqual(b.seeks, []);
  f.player.dispose();
});

test('verified track length prevents provisional Ogg estimates from cutting the song early', () => {
  const verified = 123.636349;
  const estimated = 119.619433;
  const f = fixture({ durations: [estimated, 186.048005], trackDurations: [verified, 186.048005] });
  const [a, b] = f.audios;
  a.actualDuration = verified;
  f.player.setPlaying(true);
  f.advance(121.1);
  assert.equal(f.player.getState().duration, verified);
  assert.equal(f.player.getState().mediaDuration, estimated);
  assert.equal(b.playCalls, 0);
  near(a.volume, BASE_VOLUME);
  f.advance(0.1);
  assert.equal(b.playCalls, 1);
  near(f.player.getState().fade, 0);
  a.duration = 123.639252;
  a.emit('loadedmetadata');
  assert.equal(f.player.getState().duration, verified);
  assert.equal(f.player.getState().mediaDuration, 123.639252);
  f.advance(1.25);
  near(f.player.getState().fade, 0.5);
  f.advance(1.25);
  assert.equal(f.player.getState().index, 1);
  f.player.dispose();
});

test('native ended still advances when a stream ends before its verified length', () => {
  const f = fixture({ durations: [10, 120], trackDurations: [100, 120] });
  f.player.setPlaying(true);
  f.advance(10);
  assert.equal(f.player.getState().index, 1);
  assert.equal(f.audios[1].playCalls, 1);
  near(f.audios[1].volume, BASE_VOLUME);
  f.player.dispose();
});

test('frame updates and game restarts are idempotent; pause/resume never seeks', () => {
  const f = fixture();
  const [a, b] = f.audios;
  f.player.setPlaying(true);
  f.advance(10);
  for (let i = 0; i < 1000; i++) f.player.setPlaying(true);
  assert.equal(a.playCalls, 1);
  assert.equal(b.playCalls, 0);
  near(a.currentTime, 10);
  assert.deepEqual(a.seeks, []);
  f.player.setPlaying(false);
  f.advance(30);
  near(a.currentTime, 10);
  assert.equal(f.frames.size, 0);
  f.player.setPlaying(true);
  f.advance(1);
  near(a.currentTime, 11);
  assert.equal(a.playCalls, 2);
  assert.deepEqual(a.seeks, []);
  f.player.dispose();
});

test('pause freezes both clocks and the middle of a crossfade', () => {
  const f = fixture();
  f.player.setPlaying(true);
  f.advance(98.5);
  const [a, b] = f.audios;
  near(f.player.getState().fade, 0.4);
  const before = [a.currentTime, b.currentTime, a.volume, b.volume];
  f.player.setPlaying(false);
  f.advance(60);
  assert.deepEqual([a.currentTime, b.currentTime, a.volume, b.volume], before);
  f.player.setPlaying(true);
  f.advance(0.5);
  near(f.player.getState().fade, 0.6);
  assert.equal(a.playCalls, 2);
  assert.equal(b.playCalls, 2);
  assert.deepEqual(a.seeks, []);
  assert.deepEqual(b.seeks, []);
  f.player.dispose();
});

test('muting keeps the music clock and overlap running, and restores the mix', () => {
  const f = fixture();
  f.player.setPlaying(true);
  f.advance(97.5);
  f.player.setMuted(true);
  f.advance(1.25);
  f.audios.forEach(audio => { assert.equal(audio.volume, 0); assert.equal(audio.muted, true); });
  near(f.player.getState().fade, 0.5);
  near(f.audios[1].currentTime, 1.25);
  f.player.setMuted(false);
  f.audios.forEach(audio => near(audio.volume, BASE_VOLUME / Math.sqrt(2)));
  assert.equal(f.audios[0].playCalls, 1);
  assert.equal(f.audios[1].playCalls, 1);
  f.player.dispose();
});

test('a rejected next play restores full outgoing volume and repeats the healthy track', async () => {
  const f = fixture();
  const [a, b] = f.audios;
  b.mode = 'reject';
  f.player.setPlaying(true);
  f.advance(97.5);
  await settle();
  near(a.volume, BASE_VOLUME);
  near(b.volume, 0);
  assert.equal(f.player.getState().fade, 0);
  f.advance(2.5);
  assert.equal(f.player.getState().index, 0);
  assert.equal(a.playCalls, 2);
  assert.equal(b.playCalls, 1);
  near(a.currentTime, 0);
  f.advance(98);
  assert.equal(b.playCalls, 1);
  f.player.dispose();
});

test('a next play that never advances cannot fade out the healthy track or leave silence', () => {
  const f = fixture();
  const [a, b] = f.audios;
  b.mode = 'pending';
  f.player.setPlaying(true);
  f.advance(99.9);
  near(a.volume, BASE_VOLUME);
  near(b.volume, 0);
  near(f.player.getState().fade, 0);
  f.advance(0.1);
  assert.equal(f.player.getState().index, 0);
  assert.equal(a.playCalls, 2);
  assert.equal(b.playCalls, 1);
  assert.equal(a.paused, false);
  near(a.volume, BASE_VOLUME);
  f.player.dispose();
});

test('active stream errors skip to the healthy track, and both failures stop retries', () => {
  const f = fixture();
  f.player.setPlaying(true);
  f.advance(20);
  f.audios[0].emit('error');
  assert.equal(f.player.getState().index, 1);
  assert.equal(f.audios[0].paused, true);
  assert.equal(f.audios[1].playCalls, 1);
  near(f.audios[1].volume, BASE_VOLUME);
  f.audios[1].emit('error');
  assert.equal(f.player.getState().status, 'unavailable');
  f.advance(10);
  for (let i = 0; i < 100; i++) f.player.setPlaying(true);
  assert.equal(f.frames.size, 0);
  assert.equal(f.audios[0].playCalls, 1);
  assert.equal(f.audios[1].playCalls, 1);
  f.player.dispose();
});

test('all rejected playback attempts remain bounded and retry on the next playing edge', async () => {
  const f = fixture();
  f.audios.forEach(audio => { audio.mode = 'reject'; });
  f.player.setPlaying(true);
  await settle();
  assert.equal(f.player.getState().status, 'unavailable');
  f.advance(10);
  assert.equal(f.frames.size, 0);
  f.audios.forEach(audio => assert.equal(audio.playCalls, 1));
  f.player.setPlaying(true);
  await settle();
  f.audios.forEach(audio => assert.equal(audio.playCalls, 1));
  f.player.setPlaying(false);
  f.audios.forEach(audio => { audio.mode = 'playing'; });
  f.player.setPlaying(true);
  f.advance(1);
  assert.equal(f.player.getState().status, 'playing');
  assert.equal(f.audios[1].playCalls, 2);
  near(f.audios[1].currentTime, 1);
  f.player.dispose();
});

test('decoder stalls fail over once; a background frame gap does not skip music', () => {
  const f = fixture();
  f.audios.forEach(audio => { audio.mode = 'pending'; });
  f.player.setPlaying(true);
  f.advance(5.1);
  assert.equal(f.player.getState().index, 1);
  assert.equal(f.audios[1].playCalls, 1);
  f.advance(5.1);
  assert.equal(f.player.getState().status, 'unavailable');
  f.advance(1);
  assert.equal(f.frames.size, 0);
  f.player.dispose();

  const background = fixture();
  background.player.setPlaying(true);
  background.advance(98);
  const fade = background.player.getState().fade;
  background.step(30, false);
  near(background.player.getState().fade, fade);
  assert.equal(background.player.getState().index, 0);
  background.advance(0.5);
  near(background.player.getState().fade, fade + 0.2);
  background.player.dispose();
});

test('unknown durations and tiny tracks advance on ended without pathological overlaps', () => {
  const f = fixture({ durations: [NaN, 0.25] });
  f.player.setPlaying(true);
  f.advance(1);
  assert.equal(f.audios[1].playCalls, 0);
  f.audios[0].ended = true;
  f.audios[0].emit('ended');
  assert.equal(f.player.getState().index, 1);
  f.advance(0.2);
  assert.equal(f.player.getState().fade, 0);
  f.advance(0.05);
  assert.equal(f.player.getState().index, 0);
  assert.equal(f.audios[0].playCalls, 2);
  f.player.dispose();
});

test('dispose removes listeners, stops streaming, and ignores late promise completion', async () => {
  const f = fixture();
  f.audios[0].mode = 'pending';
  f.player.setPlaying(true);
  const resolve = f.audios[0].resolvePlay;
  const snapshot = f.player.getState();
  assert.ok(Object.isFrozen(snapshot));
  f.player.dispose();
  f.player.dispose();
  resolve();
  await settle();
  f.player.setPlaying(true);
  f.player.setMuted(true);
  assert.equal(f.player.getState().status, 'disposed');
  assert.equal(f.frames.size, 0);
  f.audios.forEach(audio => {
    assert.equal(audio.paused, true);
    assert.equal(audio.src, '');
    assert.equal(audio.loadCalls, 1);
    assert.equal([...audio.listeners.values()].reduce((n, set) => n + set.size, 0), 0);
  });
  assert.equal(f.states.at(-1).status, 'disposed');
});
