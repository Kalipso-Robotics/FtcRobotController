// Turns one recorded run into the numbers and checks on the Stats page. Pure functions, no DOM, so
// they can be tested on their own (see ../stats.test.mjs). Everything here is computed from what the
// sim recorded: frames, events and telemetry. Nothing is estimated or invented.
//
// A frame is [seconds, trueX, trueY, trueTheta, odoX, odoY, odoTheta] in mm and radians, with the
// time counted from START. Odometry values are NaN until the code has created its Odometry.

/** How far off is "fine", "worth a look" and "wrong". These are this page's choices, shown next to each result. */
export const THRESHOLDS = {
  odoMmWarn: 30, odoMmFail: 80,        // odometry position error
  headingWarn: 3, headingFail: 8,      // odometry heading error, degrees
  idleTailWarn: 3,                     // seconds standing still at the end of the match
  standingMmPerS: 20, standingRadPerS: 0.05, // slower than this counts as standing still
  minIdleStretch: 0.5,                 // shortest standing-still stretch worth reporting, seconds
};

const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
const deg = (rad) => (rad * 180) / Math.PI;

/** Frames with a speed and turn rate attached, measured over about 100 ms so 20 ms rounding does not show up as noise. */
function motion(frames) {
  const speed = [], turn = [];
  let lo = 0, hi = 0;
  for (let i = 0; i < frames.length; i++) {
    while (frames[lo][0] < frames[i][0] - 0.05) lo++;
    while (hi + 1 < frames.length && frames[hi + 1][0] <= frames[i][0] + 0.05) hi++;
    const dt = frames[hi][0] - frames[lo][0];
    if (dt <= 0) { speed.push(0); turn.push(0); continue; }
    speed.push(Math.hypot(frames[hi][1] - frames[lo][1], frames[hi][2] - frames[lo][2]) / dt);
    turn.push(Math.abs(frames[hi][3] - frames[lo][3]) / dt);
  }
  return { speed, turn };
}

function corners(frame, robot) {
  const [, x, y, theta] = frame;
  const c = Math.cos(theta), s = Math.sin(theta), h = robot.width / 2;
  return [[robot.front, h], [robot.front, -h], [-robot.back, -h], [-robot.back, h]]
    .map(([lx, ly]) => [x + lx * c - ly * s, y + lx * s + ly * c]);
}

export function analyse(run) {
  const frames = run.frames;
  const t0 = frames[0][0];
  const t1 = frames[frames.length - 1][0];
  const { speed, turn } = motion(frames);

  // ---- distance, speed, acceleration ----
  let distance = 0, rotation = 0;
  for (let i = 1; i < frames.length; i++) {
    distance += Math.hypot(frames[i][1] - frames[i - 1][1], frames[i][2] - frames[i - 1][2]);
    rotation += Math.abs(frames[i][3] - frames[i - 1][3]);
  }
  let peakSpeed = 0, peakSpeedAt = 0;
  speed.forEach((v, i) => { if (v > peakSpeed) { peakSpeed = v; peakSpeedAt = frames[i][0]; } });
  let peakAccel = 0, peakAccelAt = 0;
  for (let i = 5; i < frames.length - 5; i++) {
    const a = Math.abs((speed[i + 5] - speed[i - 5]) / (frames[i + 5][0] - frames[i - 5][0]));
    if (a > peakAccel) { peakAccel = a; peakAccelAt = frames[i][0]; }
  }

  // ---- standing still ----
  const idle = [];
  let from = null;
  for (let i = 0; i < frames.length; i++) {
    const still = speed[i] < THRESHOLDS.standingMmPerS && turn[i] < THRESHOLDS.standingRadPerS;
    if (still && from === null) from = frames[i][0];
    if ((!still || i === frames.length - 1) && from !== null) {
      const end = still ? frames[i][0] : frames[i - 1][0];
      if (end - from >= THRESHOLDS.minIdleStretch) idle.push({ from, to: end });
      from = null;
    }
  }
  const idleTotal = idle.reduce((sum, s) => sum + (s.to - s.from), 0);
  const tail = idle.length && idle[idle.length - 1].to >= t1 - 0.1 ? idle[idle.length - 1] : null;
  const idleTail = tail ? tail.to - tail.from : 0;

  // ---- odometry against the real robot ----
  const odo = [], heading = [];
  let odoMax = 0, odoMaxAt = 0, odoSum = 0, odoCount = 0, headingMax = 0, headingMaxAt = 0;
  for (const f of frames) {
    if (!Number.isFinite(f[4])) continue;
    const e = Math.hypot(f[1] - f[4], f[2] - f[5]);
    const h = Math.abs(deg(wrap(f[3] - f[6])));
    odo.push([f[0], e]);
    heading.push([f[0], h]);
    odoSum += e; odoCount++;
    if (e > odoMax) { odoMax = e; odoMaxAt = f[0]; }
    if (h > headingMax) { headingMax = h; headingMaxAt = f[0]; }
  }
  const odoFinal = odo.length ? odo[odo.length - 1][1] : NaN;
  const headingFinal = heading.length ? heading[heading.length - 1][1] : NaN;

  // ---- own side of the field: the body, not just the centre ----
  let maxX = -Infinity, maxXAt = 0;
  for (const f of frames) {
    for (const [x] of corners(f, run.field.robot)) {
      if (x > maxX) { maxX = x; maxXAt = f[0]; }
    }
  }
  const midline = run.field.midline;

  // ---- contacts ----
  const touches = run.events.filter((e) => e.text.startsWith('robot touched '));
  const contacts = touches.filter((e) => e.t >= 0).map((e) => ({ t: e.t, name: e.text.replace(/^robot touched /, '') }));
  const setupContacts = touches.filter((e) => e.t < 0).map((e) => ({ t: e.t, name: e.text.replace(/^robot touched /, '') }));
  const contactsByName = {};
  for (const c of contacts) contactsByName[c.name] = (contactsByName[c.name] || 0) + 1;

  // ---- balls (only runs that have balls, and only when something happened to them) ----
  const hasBalls = Boolean(run.balls && run.balls.info.length);
  const eventTimes = (test) => run.events.filter((e) => e.t >= 0 && test(e.text)).map((e) => e.t);
  const pickups = eventTimes((t) => t.startsWith('picked up'));
  const shots = eventTimes((t) => t.startsWith('shot a'));
  const shotsIn = eventTimes((t) => t.includes('went into'));
  const tips = eventTimes((t) => t.endsWith(' tipped'));
  const held = hasBalls ? run.balls.held : [];
  const heldAtEnd = held.length ? held[held.length - 1][1] : 0;

  // ---- what decide() chose (only autos that report "decide says") ----
  const decisions = [];
  for (const entry of run.telemetry) {
    const move = entry.items['decide says'];
    if (move === undefined) continue;
    const last = decisions[decisions.length - 1];
    if (last && last.move === move) continue;
    if (last) last.to = entry.t;
    decisions.push({ move, from: Math.max(0, entry.t), to: t1 });
  }
  const decisionTime = {};
  for (const d of decisions) decisionTime[d.move] = (decisionTime[d.move] || 0) + (d.to - d.from);

  // ---- checks ----
  const checks = [];
  const add = (id, label, status, detail, seek = null) => checks.push({ id, label, status, detail, seek });

  const firstErrorLine = run.error ? run.error.split('\n').find((l) => l.trim()) : '';
  if (run.error) add('ran', 'Ran to the end', 'fail', `The auto stopped with an error: ${firstErrorLine}`);
  else if (!run.finished) add('ran', 'Ran to the end', 'fail', 'The auto did not finish before the sim gave up.');
  else add('ran', 'Ran to the end', 'pass', `The match ran the full ${run.used.toFixed(1)} s with no errors.`);

  if (run.warnings.length) add('setup', 'Setup is valid', 'warn', run.warnings.join(' '));
  else add('setup', 'Setup is valid', 'pass', 'The start pose fits inside the field and clear of every obstacle.');

  if (contacts.length) {
    const list = Object.entries(contactsByName).map(([n, c]) => `${n}${c > 1 ? ` ×${c}` : ''}`).join(', ');
    add('contacts', 'Touched nothing', 'warn', `The robot touched ${list}. First at ${contacts[0].t.toFixed(2)} s.`, contacts[0].t);
  } else add('contacts', 'Touched nothing', 'pass', 'The robot never touched a wall or an obstacle during the match.');

  if (!odo.length) add('odometry', 'Odometry tracks the robot', 'warn', 'The code never started its odometry, so there is nothing to compare.');
  else {
    const status = odoMax > THRESHOLDS.odoMmFail ? 'fail' : odoMax > THRESHOLDS.odoMmWarn ? 'warn' : 'pass';
    add('odometry', 'Odometry tracks the robot', status,
      `Position was off by up to ${odoMax.toFixed(0)} mm (at ${odoMaxAt.toFixed(1)} s) and ${odoFinal.toFixed(0)} mm at the end. Fine is under ${THRESHOLDS.odoMmWarn} mm.`, odoMaxAt);
    const hStatus = headingMax > THRESHOLDS.headingFail ? 'fail' : headingMax > THRESHOLDS.headingWarn ? 'warn' : 'pass';
    add('heading', 'Heading tracks the robot', hStatus,
      `Heading was off by up to ${headingMax.toFixed(1)}° (at ${headingMaxAt.toFixed(1)} s). Fine is under ${THRESHOLDS.headingWarn}°.`, headingMaxAt);
  }

  if (maxX > midline) add('side', 'Stays on its own side', 'fail', `The robot body reached ${(maxX - midline).toFixed(0)} mm past the midline at ${maxXAt.toFixed(1)} s.`, maxXAt);
  else add('side', 'Stays on its own side', 'pass', `The body got within ${(midline - maxX).toFixed(0)} mm of the midline at most.`, maxXAt);

  if (idleTail > THRESHOLDS.idleTailWarn) add('idle', 'Uses the whole match', 'warn',
    `The robot stood still for the last ${idleTail.toFixed(1)} s (from ${tail.from.toFixed(1)} s).`, tail.from);
  else add('idle', 'Uses the whole match', 'pass', idleTail > 0 ? `It stood still for the last ${idleTail.toFixed(1)} s.` : 'It was still moving at the end.');

  if (hasBalls && shots.length) {
    const status = shotsIn.length === shots.length ? 'pass' : shotsIn.length ? 'warn' : 'fail';
    add('shots', 'Shots land in a Hive Cell', status,
      `${shotsIn.length} of ${shots.length} shots went into a Cell${shotsIn.length < shots.length ? `; the other ${shots.length - shotsIn.length} missed or hit the Hive` : ''}.`, shots[0]);
    add('tips', 'Tips a Hive', tips.length ? 'pass' : 'warn',
      tips.length ? `${tips.length} Hive tip${tips.length === 1 ? '' : 's'}, the first at ${tips[0].toFixed(1)} s.` : 'No Hive tipped. Not enough weight went into an up Cell.', tips.length ? tips[0] : null);
  }

  const statuses = checks.map((c) => c.status);
  const verdict = statuses.includes('fail') ? 'failed' : statuses.includes('warn') ? 'attention' : 'clean';
  const needLook = statuses.filter((s) => s !== 'pass').length;
  // "Run failed" is only for an auto that crashed or did not finish; a failed check is "Problems found"
  const crashed = checks.find((c) => c.id === 'ran').status === 'fail';
  const verdictText = verdict === 'clean' ? 'Clean run' : verdict === 'attention' ? 'Needs a look' : crashed ? 'Run failed' : 'Problems found';

  return {
    verdict, verdictText, needLook, checks,
    numbers: {
      used: run.used, distance, rotationDeg: deg(rotation), peakSpeed, peakSpeedAt, peakAccel, peakAccelAt,
      avgMovingSpeed: (t1 - t0 - idleTotal) > 0 ? distance / (t1 - t0 - idleTotal) : 0,
      movingTime: t1 - t0 - idleTotal, idleTotal, idleTail,
      contacts: contacts.length, setupContacts: setupContacts.length,
      odoMax, odoMaxAt, odoMean: odoCount ? odoSum / odoCount : NaN, odoFinal, headingMax, headingMaxAt, headingFinal,
      maxX, midline, frames: frames.length,
      pickups: pickups.length, shots: shots.length, shotsIn: shotsIn.length, hiveTips: tips.length, heldAtEnd,
      simMs: run.meta ? run.meta.simMs : null,
      speedup: run.meta && run.meta.simMs ? (t1 - t0) / (run.meta.simMs / 1000) : null,
    },
    hasBalls: hasBalls && (pickups.length + shots.length + tips.length) > 0,
    series: { speed: frames.map((f, i) => [f[0], speed[i]]), odo, heading, held },
    contacts, setupContacts, contactsByName, idle, decisions, decisionTime,
    duration: t1,
  };
}
