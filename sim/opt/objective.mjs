// Scores one parsed run for the optimiser: auto points minus penalties. Pure, no I/O.
import { analyse, score } from '../app/web/stats.js';

// Penalty sizes, in points. These are tuning choices, not manual values.
export const WEIGHTS = { contact: 10, nearMiss: 5, jerk: 0.1 };   // per partner touch; at zero gap; per m/s^3 of RMS jerk
export const THRESHOLD_MM = 150;                                   // closer than this to the partner is a near miss (matches stats THRESHOLDS.partnerWarnMm)
const JERK_STRIDE = 5;                                             // frames between samples, about 0.1 s, so 0.1 mm rounding in the log does not read as jerk

/** RMS jerk of the true path in m/s^3, from third differences of position. */
function rmsJerk(frames) {
  const s = frames.filter((_, i) => i % JERK_STRIDE === 0);
  let sum = 0, n = 0;
  for (let i = 3; i < s.length; i++) {
    const dt = (s[i][0] - s[i - 3][0]) / 3;
    if (dt <= 0) continue;
    for (const c of [1, 2]) {
      const j = (s[i][c] - 3 * s[i - 1][c] + 3 * s[i - 2][c] - s[i - 3][c]) / (dt * dt * dt) / 1000;
      sum += j * j;
    }
    n++;
  }
  return n ? Math.sqrt(sum / n) : 0;
}

/** { value, points, penalties: { contact, nearMiss, jerk } } for one run from parseRun(). */
export function objective(run) {
  const a = analyse(run);
  const points = score(run, a).final;
  const touches = a.contactsByName.partner || 0;
  const closest = a.numbers.partnerClosest;
  const penalties = {
    contact: touches * WEIGHTS.contact,
    // ponytail: only the single closest approach counts, not how long the robots stayed close
    nearMiss: closest != null && closest < THRESHOLD_MM ? WEIGHTS.nearMiss * (1 - closest / THRESHOLD_MM) : 0,
    jerk: WEIGHTS.jerk * rmsJerk(run.frames),
  };
  return { value: points - penalties.contact - penalties.nearMiss - penalties.jerk, points, penalties };
}

/** results: objective() outputs, or { error } for runs that crashed or timed out. mean/worst cover the finished ones only. */
export function aggregate(results) {
  const ok = results.filter((r) => !r.error).map((r) => r.value);
  return {
    mean: ok.length ? ok.reduce((x, y) => x + y, 0) / ok.length : null,
    worst: ok.length ? Math.min(...ok) : null,
    n: results.length,
    errors: results.length - ok.length,
  };
}
