import test from 'node:test';
import assert from 'node:assert/strict';
import { score, ffFit } from './pidf.mjs';

// first-order response to `target` with an optional overshoot, sampled at 50 Hz, one 3% dip at the shot
const trace = (fn, n = 600) => Array.from({ length: n }, (_, i) => ({ t: i * 0.02, rps: fn(i * 0.02) }));

test('clean spin-up and one recovered dip: recovery time, no overshoot, no oscillation', () => {
  const tr = trace((t) => (t < 4 ? 60 * (1 - Math.exp(-t / 0.5)) : 60 - 1.8 * Math.exp(-(t - 4) / 0.3)));
  const s = score(tr, 60, [4]);
  assert.ok(s.overshoot < 0.01);
  assert.ok(s.recovery[0] > 0.1 && s.recovery[0] < 0.4, `recovery ${s.recovery[0]}`);   // 1.8 -> 1 at 0.3*ln(1.8)=0.176 s
  assert.ok(Math.abs(s.steadyErr) < 0.05);
  assert.equal(s.oscillating, false);
});

test('a ringing trace is flagged oscillating and its overshoot measured', () => {
  const tr = trace((t) => 60 + 3 * Math.exp(-t / 5) * Math.sin(2 * Math.PI * 2 * t) * (t > 1 ? 1 : 0));
  const s = score(tr, 60, []);
  assert.equal(s.oscillating, true);
  assert.ok(s.overshoot > 2);
});

test('never recovering gives Infinity recovery and the steady error', () => {
  const tr = trace((t) => (t < 4 ? 60 : 58));
  const s = score(tr, 60, [4]);
  assert.equal(s.recovery[0], Infinity);
  assert.ok(Math.abs(s.steadyErr + 2) < 1e-9);
});

test('ffFit recovers the analytic plant: kS, kV = (1-kS)/TOP', () => {
  const pts = [0.1, 0.3, 0.5, 0.7, 0.9].map((p) => [p, ((p - 0.04) / 0.96) * 90]);
  const f = ffFit(pts);
  assert.ok(Math.abs(f.ks - 0.04) < 1e-9 && Math.abs(f.kv - 0.96 / 90) < 1e-9);
});
