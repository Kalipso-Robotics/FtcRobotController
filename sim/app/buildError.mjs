// Turns Gradle/javac output into something the page can show: one error with its file, line and source lines.

/**
 * Gradle prints each compiler error twice, plus a lot of boilerplate. Keep each error once:
 * "file:line: error: message", the source line and the caret, with the path shortened to the repo.
 */
export function cleanBuildOutput(output, root) {
  const lines = output.split('\n');
  const seen = new Set();
  const blocks = [];
  for (let i = 0; i < lines.length; i++) {
    if (!/: error:/.test(lines[i])) continue;
    const head = lines[i].trim().replace(root + '/', '');
    if (seen.has(head)) continue;
    seen.add(head);
    blocks.push([head, lines[i + 1], lines[i + 2]].filter((l) => l !== undefined).map((l, k) => (k === 0 ? l : l.replace(/^\s{0,2}/, ''))).join('\n'));
  }
  if (blocks.length) return blocks.join('\n\n');
  return lines.filter((l) => l.trim() && !/^(warning:|Note:|\d+ warnings?)/.test(l.trim())).slice(0, 40).join('\n');
}

/**
 * The first "path:line: error: message" block as { file, line, col, message, detail, context }.
 * context = the source lines line-1..line+1 (empty when the file cannot be read); the error line carries the caret column.
 */
export function parseCompileError(output, root, readFile) {
  const lines = output.split('\n');
  const at = lines.findIndex((l) => /^\S.*:\d+: error: /.test(l));
  if (at < 0) return null;
  const m = /^(.*?):(\d+): error: (.*)$/.exec(lines[at].trim());
  const file = m[1].startsWith(root + '/') ? m[1].slice(root.length + 1) : m[1];
  const line = Number(m[2]);
  const caretLine = lines[at + 2] ?? '';
  const col = caretLine.includes('^') ? caretLine.indexOf('^') + 1 : null;
  const detail = [];
  for (let i = at + 3; i < lines.length && /^\s+(symbol|location|required|found|reason):/.test(lines[i]); i++) {
    detail.push(lines[i].trim().replace(/:\s+/, ': '));
  }
  let source = null;
  try { source = readFile(m[1]); } catch { /* unreadable */ }
  const context = [];
  if (source !== null && source !== undefined) {
    const all = source.split('\n');
    for (let n = Math.max(1, line - 1); n <= Math.min(all.length, line + 1); n++) {
      context.push(n === line && col ? { n, text: all[n - 1], caret: col } : { n, text: all[n - 1] });
    }
  }
  return { file, line, col, message: m[3], detail: detail.join(' · '), context };
}
