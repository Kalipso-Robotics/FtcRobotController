// Undo/redo stack of model snapshots (strings). Pure, so it is unit-tested in ../history.test.mjs.
export function createHistory(limit = 50) {
  let past = [], future = [];
  const cap = () => { while (past.length > limit) past.shift(); };
  return {
    push(snap) { past.push(snap); cap(); future = []; },
    undo(current) { if (!past.length) return null; future.push(current); return past.pop(); },
    redo(current) { if (!future.length) return null; past.push(current); cap(); return future.pop(); },
    canUndo: () => past.length > 0,
    canRedo: () => future.length > 0,
    clear() { past = []; future = []; },
  };
}
