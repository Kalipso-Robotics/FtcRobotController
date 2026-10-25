// The Kit icon set. Status badges are drawn on a 12 px grid, the rest on 16 px. Paths copied from the Kit board.
const NS = 'http://www.w3.org/2000/svg';

const S12 = {
  pass: '<path d="M2.6 6.3l2.2 2.2 4.6-4.8"/>',
  warn: '<path d="M6 2.8v3.9M6 9.2v.01"/>',
  fail: '<path d="M3.5 3.5l5 5M8.5 3.5l-5 5"/>',
  none: '<path d="M3 6h6"/>',
  stale: '<path d="M6 3.2v3l2 1.3"/>',
  offline: '<path d="M2.5 6h2M7.5 6h2M5 4.5l2 3"/>',
  paused: '<path d="M4.5 3v6M7.5 3v6"/>',
  blocked: '<circle cx="6" cy="6" r="3.8"/><path d="M3.4 8.6l5.2-5.2"/>',
  play: '<path d="M3.5 2 L10 6 L3.5 10 Z" fill="currentColor" stroke="none"/>',
  pausebars: '<path d="M4 2.5v7M8 2.5v7" stroke-width="2"/>',
  back: '<path d="M7.5 2.5 L4 6 L7.5 9.5"/>',
  fwd: '<path d="M4.5 2.5 L8 6 L4.5 9.5"/>',
};
const S16 = {
  robot: '<rect x="3" y="3" width="10" height="10" rx="2"/><path d="M13 6.5 L15 8 L13 9.5"/>',
  shooter: '<circle cx="5.5" cy="10.5" r="3.5"/><path d="M8.5 8 C10 4.5 12 3 14.5 3"/><circle cx="13.6" cy="3.4" r="1.2"/>',
  intake: '<circle cx="5" cy="11" r="2.5"/><circle cx="11" cy="11" r="2.5"/><path d="M8 2.5v4M6 4.8L8 6.8l2-2"/>',
  ball: '<circle cx="8" cy="8" r="5"/><path d="M4.5 6.5c2 1.2 5 1.2 7 0"/>',
  hive: '<path d="M3 13V7.5L8 3l5 4.5V13Z"/><path d="M6 13v-3h4v3"/>',
  partner: '<rect x="1.5" y="6" width="6" height="6" rx="1.5"/><rect x="8.5" y="4" width="6" height="6" rx="1.5"/>',
  shots: '<circle cx="8" cy="8" r="5.5"/><circle cx="8" cy="8" r="2"/>',
  contact: '<path d="M2.5 3h11"/><path d="M8 6.5 L10.5 9 L8 11.5 L5.5 9 Z"/>',
  odometry: '<circle cx="8" cy="8" r="2"/><rect x="2.5" y="2.5" width="11" height="11" rx="2" stroke-dasharray="2 2"/>',
  pose: '<circle cx="8" cy="8" r="5.5"/><path d="M8 4.5 L10 9.5 L8 8.5 L6 9.5 Z"/>',
  motors: '<circle cx="8" cy="8" r="5"/><path d="M8 3v2M8 11v2M3 8h2M11 8h2"/><circle cx="8" cy="8" r="1.5"/>',
  sensors: '<circle cx="4" cy="8" r="1.5"/><path d="M7 5.5c1.3 1.4 1.3 3.6 0 5M10 4c2 2.2 2 5.8 0 8"/>',
  telemetry: '<path d="M3 4h10M3 8h7M3 12h9"/>',
  logs: '<rect x="2" y="3" width="12" height="10" rx="2"/><path d="M5 7l2 1.5L5 10M8.5 10.5H11"/>',
  trust: '<path d="M8 2.5 L13 4.5 V8 C13 11 10.5 13 8 14 C5.5 13 3 11 3 8 V4.5 Z"/>',
  checks: '<path d="M3 5l1.5 1.5L7 4M3 11l1.5 1.5L7 10M9.5 5h4M9.5 11h4"/>',
  path: '<path d="M4 2h5l3 3v9H4z"/><path d="M6 11c1.5-3 2.5 0 4-3"/>',
  code: '<path d="M6 5L3 8l3 3M10 5l3 3-3 3"/>',
  numbers: '<path d="M3 13V8M8 13V4M13 13v-6"/>',
  params: '<path d="M2 4h7M12 4h2M2 12h3M8 12h6M9 2.5v3M5 10.5v3"/>',
  chevron: '<path d="M6 3.5 L10.5 8 L6 12.5"/>',
};
const ALIASES = { play: 'play', pause: 'pausebars' };

/** icon('pass') -> an <svg> element. Status icons use the 12 px grid, everything else the 16 px one. */
export function icon(name) {
  const key = ALIASES[name] || name;
  const small = key in S12;
  const body = small ? S12[key] : S16[key];
  const svg = document.createElementNS(NS, 'svg');
  svg.setAttribute('viewBox', small ? '0 0 12 12' : '0 0 16 16');
  svg.setAttribute('aria-hidden', 'true');
  if (!small || !['play'].includes(key)) {
    svg.setAttribute('fill', 'none'); svg.setAttribute('stroke', 'currentColor');
    svg.setAttribute('stroke-width', '1.6'); svg.setAttribute('stroke-linecap', 'round'); svg.setAttribute('stroke-linejoin', 'round');
  }
  svg.innerHTML = body || '';
  return svg;
}

/** A status badge: <span class="sb sb-warn"> with its icon. status: pass | warn | fail | none (or stale/offline/paused/blocked as icon names). */
export function badge(status, iconName = status) {
  const el = document.createElement('span');
  el.className = `sb sb-${['pass', 'warn', 'fail', 'none'].includes(status) ? status : 'none'}`;
  el.append(icon(iconName));
  return el;
}
