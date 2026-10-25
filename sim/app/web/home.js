// Home: pick an auto, optionally pick a partner path, and run. The field behind it is the last run's real 3D scene;
// before the first run it is a plain tile grid (no fake robot, no fake run).

import { h } from './rail.js';
import { badge, icon } from './icons.js';

const inches = (mm) => Math.round(mm / 25.4);

/**
 * ctx = { opmodes(), selected(), select(name), verdictFor(name): {status, id} | null, run(), watching(), setWatching(bool),
 *         connected(), hasRun(), openLast(),
 *         partner: { mode(), file(), list(): [{file,name,duration,start:[x,y,deg]}], set(mode, file) } }
 * Returns { render(), setCaption(text), setEmpty(bool) }.
 */
export function mountHome(root, stageHost, ctx) {
  const caption = h('div', { class: 'homecap' });
  stageHost.append(caption);

  function autoRow(m) {
    const sel = ctx.selected() === m.name;
    const v = ctx.verdictFor(m.name);
    return h('button', { class: `op${sel ? ' sel' : ''}`, role: 'radio', 'aria-checked': String(sel), onclick: () => { ctx.select(m.name); render(); } },
      h('span', { class: 'ic' }, icon('robot')),
      h('span', { class: 'optxt' },
        h('span', { class: 'opname' }, h('b', { class: 'm', text: m.name }), h('span', { class: `tag ${m.alliance === 'red' ? 'red' : 'blue'}`, text: m.alliance.toUpperCase() })),
        h('span', { class: 'm opstart', text: `start ${inches(m.start.x)}, ${inches(m.start.y)} in · ${m.start.headingDeg}°` })),
      v ? h('span', { class: 'opres' }, badge(v.status), h('span', { class: 'm', text: `#${v.id}` })) : h('span', { class: 'opres dim', text: 'not run yet' }));
  }

  function partnerBlock() {
    const p = ctx.partner;
    const list = p.list();
    const mode = p.mode();
    const seg = h('div', { class: 'segmented wide', role: 'radiogroup', 'aria-label': 'Partner robot' },
      h('button', { role: 'radio', 'aria-pressed': String(mode === 'none'), onclick: () => { p.set('none', null); render(); } }, 'None'),
      h('button', { role: 'radio', 'aria-pressed': String(mode === 'file'), onclick: () => { p.set('file', p.file() || (list[0] && list[0].file) || null); render(); } }, icon('path'), ' Path file'),
      h('button', { disabled: true, class: 'later', title: 'Not built yet' }, 'Draw a path ', h('span', { class: 'tag', text: 'LATER' })));
    let row = null;
    if (mode === 'file') {
      const cur = list.find((x) => x.file === p.file());
      const select = h('select', { 'aria-label': 'Partner path file' }, list.map((x) => h('option', { value: x.file, selected: x.file === p.file() ? '' : undefined, text: `paths/${x.file}` })));
      select.addEventListener('change', () => { p.set('file', select.value); render(); });
      row = h('label', { class: 'partnerrow' }, h('span', { class: 'ic' }, icon('partner')),
        h('span', { class: 'optxt' }, h('b', { class: 'm', text: cur ? `paths/${cur.file}` : 'no path files found' }),
          h('span', { class: 'm opstart', text: cur ? `R2 · start ${inches(cur.start[0])}, ${inches(cur.start[1])} in · runs ${cur.duration.toFixed(1)} s · no shooting` : 'Put a .json in sim/paths/' })),
        h('span', { class: 'chevs' }, icon('chevron')), select);
    }
    return [h('h2', { class: 'h', text: 'Partner robot' }), seg, row];
  }

  function render() {
    const modes = ctx.opmodes();
    const sel = modes.find((m) => m.name === ctx.selected());
    const watching = ctx.watching();
    const toggle = h('input', { type: 'checkbox', id: 'watchHome' }); toggle.checked = watching;
    toggle.addEventListener('change', () => ctx.setWatching(toggle.checked));
    root.replaceChildren(
      h('div', { class: 'brandline' }, h('span', { class: 'mark' }), 'KALIPSO AUTO SIM'),
      h('h1', { text: 'Test your auto before the robot does.' }),
      h('p', { class: 'lede' }, 'Runs your real ', h('code', { class: 'm', text: '@Autonomous' }), ' code on a simulated robot and field.'),
      h('h2', { class: 'h', text: `Your auto · ${modes.length} found in TeamCode` }),
      h('div', { class: 'oplist', role: 'radiogroup', 'aria-label': 'Autonomous' }, modes.map(autoRow)),
      ...partnerBlock().filter(Boolean),
      h('button', { class: 'btn btn-p big runbtn', onclick: () => ctx.run() }, `Run ${sel ? sel.name : ''} `, h('span', { class: 'kbd', text: 'R' })),
      h('div', { class: 'savecard' },
        h('label', { class: 'switch' }, toggle, h('span', { class: 'track' }),
          h('span', { class: 'savetxt' }, h('b', { text: `Run on save is ${watching ? 'on' : 'off'}` }), h('span', { class: 'dim', text: watching ? 'Saving a Java file in Android Studio starts a run.' : 'Press R to run after you save.' }))),
        h('div', { class: 'statusline' },
          h('span', { class: `dotc ${ctx.connected() ? 'ok' : 'off'}` }), h('span', { text: ctx.connected() ? 'Runner connected · physics ready' : 'Runner not reachable' }),
          h('span', { class: 'spacer' }),
          ctx.hasRun() ? h('button', { class: 'linkbtn', onclick: () => ctx.openLast() }, 'Last results →') : null)));
  }

  return { render, setCaption(text) { caption.textContent = text; caption.hidden = !text; }, setEmpty(empty) { stageHost.classList.toggle('nogrid', empty); } };
}
