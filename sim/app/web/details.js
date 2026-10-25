// The details drawer: what the robot was doing at the playhead. Recorded data only (pose, decide(), holding, balls,
// telemetry, nearby events). There are no motor, shooter, sensor or log sections because the sim does not record them.

import { h } from './rail.js';
import { icon } from './icons.js';
import { eventText } from './stats.js';
import { eventKind } from './timeline.js';

const wrapDeg = (rad) => { const d = (rad * 180) / Math.PI; return ((d + 180) % 360 + 360) % 360 - 180; };
const fmtPose = (x, y, th) => `${x.toFixed(0)}, ${y.toFixed(0)} · ${wrapDeg(th).toFixed(1)}°`;
const inches = (mm) => (mm / 25.4).toFixed(1);
const fmtPoseIn = (x, y, th) => `${inches(x)}, ${inches(y)} in · ${wrapDeg(th).toFixed(1)}°`;

/** "gap 38 mm · 2.8°" between the true and odometry pose, or null when odometry is unknown or the gap is warnMm or less. */
export function gapLabel(pose, warnMm = 0) {
  const gap = Math.hypot(pose.x - pose.ox, pose.y - pose.oy);
  if (!Number.isFinite(gap) || gap <= warnMm) return null;
  const heading = Math.abs(wrapDeg(pose.theta - pose.otheta));
  return `gap ${gap.toFixed(0)} mm · ${heading.toFixed(1)}°`;
}

/** What decide() said at time t: { move, since, was } (was = the move before, or null). */
export function decideAt(decisions, t) {
  let i = -1;
  for (let k = 0; k < decisions.length; k++) if (decisions[k].from <= t) i = k;
  if (i < 0) return null;
  return { move: decisions[i].move, since: decisions[i].from, was: i > 0 ? decisions[i - 1].move : null };
}

function accordion(id, iconName, title, summaryEl, bodyEl, open = false) {
  const head = h('button', { class: 'acc-head', 'aria-expanded': String(open), 'aria-controls': `acc-${id}` },
    h('span', { class: 'ic' }, icon(iconName)), h('b', { text: title }), summaryEl, h('span', { class: 'chev' }, icon('chevron')));
  const body = h('div', { class: 'acc-body', id: `acc-${id}`, hidden: !open }, bodyEl);
  head.addEventListener('click', () => {
    const next = head.getAttribute('aria-expanded') !== 'true';
    head.setAttribute('aria-expanded', String(next)); body.hidden = !next;
  });
  return h('section', { class: 'acc' }, head, body);
}

/**
 * Builds the drawer inside root. update(t, pose, run, a, ctx) is cheap: it only touches text when the frame changed.
 * ctx = { balls: {label: count} | null, odoWarnMm }.
 */
export function mountDetails(root, { onClose }) {
  const title = h('b', { class: 'dt-title m', text: 'At 0.00 s' });
  const close = h('button', { class: 'dt-close', 'aria-label': 'Close details', onclick: onClose }, '✕');
  const decide = { move: h('b', { class: 'm big', text: '—' }), since: h('span', { class: 'sub', text: '' }) };
  const decideBox = h('div', { class: 'decidebox' }, h('span', { class: 'h', text: 'decide()' }), decide.move, decide.since);

  const poseTrue = h('b', { class: 'm' }), poseOdo = h('b', { class: 'm' }), poseSummary = h('span', { class: 'summary warn m' });
  const poseBody = [h('div', { class: 'kv' }, h('span', { text: 'True' }), poseTrue), h('div', { class: 'kv' }, h('span', { text: 'Odometry' }), poseOdo)];
  const holding = h('span', { class: 'summary m' }), holdingBody = h('p', { class: 'dim', text: 'Balls the robot carries right now.' });
  const ballsSummary = h('span', { class: 'summary m' }), ballsList = h('div');
  const telSummary = h('span', { class: 'summary m' }), telList = h('div', { class: 'tel' });
  const evSummary = h('span', { class: 'summary m' }), evList = h('div');

  const accs = [
    accordion('pose', 'pose', 'Pose', poseSummary, poseBody, true),
    accordion('holding', 'intake', 'Holding', holding, holdingBody, true),
    accordion('balls', 'ball', 'Balls', ballsSummary, ballsList),
    accordion('tel', 'telemetry', 'Telemetry', telSummary, telList),
    accordion('ev', 'contact', 'Events near here', evSummary, evList, true),
  ];
  root.replaceChildren(h('div', { class: 'dt-head' }, title, close), decideBox, ...accs);

  let shownFrame = -2, shownTel = -2, shownEvents = '', run = null;
  const reset = () => { shownFrame = -2; shownTel = -2; shownEvents = ''; };

  return {
    setRun(next) { run = next; reset(); },
    update(t, pose, runNow, a, ctx = {}) {
      run = runNow;
      const frameKey = pose.index;
      title.textContent = `At ${t.toFixed(2)} s`;
      const evKey = nearIdx(run, t).join();
      if (frameKey !== shownFrame) {
        shownFrame = frameKey;
        const d = decideAt(a.decisions, t);
        decide.move.textContent = d ? d.move : 'not reported';
        decide.since.textContent = d ? `since ${d.since.toFixed(2)} s${d.was ? ` · was ${d.was}` : ''}` : 'This auto does not report decide().';
        poseTrue.textContent = fmtPoseIn(pose.x, pose.y, pose.theta);
        const odoKnown = Number.isFinite(pose.ox);
        poseOdo.textContent = odoKnown ? fmtPoseIn(pose.ox, pose.oy, pose.otheta) : '—';
        const gap = gapLabel(pose, 0);
        poseSummary.textContent = gap || (odoKnown ? '' : 'no odometry');
        poseSummary.classList.toggle('warn', Boolean(gap) && Math.hypot(pose.x - pose.ox, pose.y - pose.oy) > (ctx.odoWarnMm ?? 30));
        const held = heldAt(run, t);
        holding.textContent = `${held} of 4`;
        if (ctx.balls) {
          ballsSummary.textContent = `${Object.values(ctx.balls)[0] ?? ''}`;
          ballsList.replaceChildren(...Object.entries(ctx.balls).map(([k, v]) => h('div', { class: 'kv' }, h('span', { text: k }), h('b', { class: 'm', text: String(v) }))));
        }
      }
      const telIdx = telemetryIndex(run, t);
      if (telIdx !== shownTel) {
        shownTel = telIdx;
        const items = telIdx >= 0 ? run.telemetry[telIdx].items : {};
        const keys = Object.keys(items);
        telSummary.textContent = `${keys.length} keys`;
        telList.replaceChildren(...(keys.length ? keys.slice(0, 40).map((k) => h('div', { class: 'kv' }, h('span', { text: k }), h('b', { class: 'm', text: items[k], title: items[k] }))) : [h('p', { class: 'dim', text: 'none yet' })]));
      }
      if (evKey !== shownEvents) {
        shownEvents = evKey;
        const near = nearIdx(run, t).map((i) => run.events[i]);
        evSummary.textContent = near.length ? `${near.length}` : '';
        evList.replaceChildren(...(near.length ? near.map((e) => h('div', { class: 'evrow' }, h('span', { class: 'm t', text: e.t.toFixed(2) }), h('span', { class: `kdot k-${eventKind(e.text)}` }), h('span', { text: eventText(e.text) }))) : [h('p', { class: 'dim', text: 'Nothing within a second.' })]));
      }
    },
  };
}

function nearIdx(run, t) {
  const out = [];
  run.events.forEach((e, i) => { if (e.t >= 0 && Math.abs(e.t - t) <= 1) out.push(i); });
  return out;
}

function heldAt(run, t) {
  const list = run.balls ? run.balls.held : [];
  let v = 0;
  for (const [ht, count] of list) { if (ht <= t) v = count; else break; }
  return v;
}

function telemetryIndex(run, t) {
  const list = run.telemetry;
  let index = -1;
  for (let lo = 0, hi = list.length - 1; lo <= hi;) {
    const mid = (lo + hi) >> 1;
    if (list[mid].t <= t) { index = mid; lo = mid + 1; } else hi = mid - 1;
  }
  return index;
}
