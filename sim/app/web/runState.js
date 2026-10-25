// What the run chip in the top bar says, from what the server reports and which run the page has loaded.
// Pure, so the failure states are tested (../runState.test.mjs). The chip marks the picture OLD the moment a build starts,
// not when it ends, so a stale run is never read as the new one.

const clock = (ms) => new Date(ms).toLocaleTimeString('en-GB', { hour12: false });

/**
 * @param state       { phase, runId, watching, since, lastSaveAt, error } from the server
 * @param loadedRun   the run the page shows ({ meta, frames, error }) or null
 * @param now         ms epoch (a parameter so tests do not depend on the clock)
 * @param verdict     'pass' | 'warn' | 'fail' for the loaded run, used for the badge of a fresh run
 * @returns { status: 'pass'|'warn'|'fail'|'none'|'busy', title, sub, action?: 'retry'|'run', old: boolean, extra? }
 */
export function chipFor(state, loadedRun, now, verdict = 'pass') {
  const id = loadedRun && loadedRun.meta ? loadedRun.meta.runId : null;
  const finished = loadedRun && loadedRun.meta && loadedRun.meta.finishedAt ? Date.parse(loadedRun.meta.finishedAt) : 0;
  const have = Boolean(loadedRun);

  if (state.phase === 'offline') return { status: 'fail', title: 'Runner disconnected', sub: '', action: 'retry', old: false };
  if (state.phase === 'compile-error') {
    return { status: 'fail', title: 'Build failed', sub: `${clock(state.since)}${have ? ` · showing old run #${id}` : ''}`, action: 'retry', old: have };
  }
  if (state.phase === 'run-error') {
    const last = have && loadedRun.frames && loadedRun.frames.length ? loadedRun.frames[loadedRun.frames.length - 1][0] : null;
    const partial = have && Boolean(loadedRun.error);
    return { status: 'fail', title: 'Run failed', sub: partial ? `crashed at ${last === null ? '?' : last.toFixed(1)} s` : `no run came out${have ? ` · showing old run #${id}` : ''}`, action: 'retry', old: have && !partial };
  }
  if (state.phase === 'building' || state.phase === 'running') {
    const seconds = Math.max(0, Math.round((now - state.since) / 1000));
    return { status: 'busy', title: state.phase === 'building' ? 'Compiling' : 'Simulating', sub: `${seconds} s`, old: have };
  }
  if (!have) return { status: 'none', title: 'No run yet', sub: '', old: false };
  if (state.lastSaveAt > finished && !state.watching) {
    return { status: 'none', title: `Run #${id} is out of date`, sub: 'press R', action: 'run', old: true };
  }
  const chip = { status: verdict, title: `Run #${id}`, sub: `fresh${finished ? ` · ${clock(finished)}` : ''}`, old: false };
  if (!state.watching) chip.extra = { status: 'none', title: 'Run on save off', sub: 'press R' };
  return chip;
}
