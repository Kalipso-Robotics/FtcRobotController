// POST /api/robot/stl?name=foo.stl[&overwrite=1]: the one place the browser writes a file onto disk. A trust boundary, so:
// basename only, .stl only, size-capped while reading, header sanity-checked, written only into `dir`.
// Codes: 201 saved | 400 bad name/header/empty | 409 file exists (retry with &overwrite=1) | 413 over the cap.
import fs from 'node:fs';
import path from 'node:path';

export const MAX_STL_BYTES = 50 * 1024 * 1024;

/** Error text for a bad upload name, or null. Same name rule as robotValidate's mesh files, plus no "..". */
export function checkStlName(name) {
  if (typeof name !== 'string' || !name) return 'Missing file name.';
  if (name.includes('/') || name.includes('\\') || name.includes('..') || path.basename(name) !== name) return 'File name must not contain a path.';
  if (name.length > 100 || !/^[\w.-]+\.stl$/i.test(name)) return 'File name must be letters, digits, _ . - and end in .stl.';
  return null;
}

/** Error text if the bytes are not plausibly an STL (binary: 84 + 50*n bytes; ASCII: "solid" with facets), or null. */
export function checkStlHeader(buf) {
  if (buf.length < 84) return 'File is too small to be an STL.';
  const n = buf.readUInt32LE(80);
  if (buf.length === 84 + 50 * n) return n > 0 ? null : 'STL has no triangles.';
  // ponytail: ASCII check only samples the first 4 KB; a truncated ASCII file still gets through
  const head = buf.subarray(0, 4096).toString('latin1');
  if (/^\s*solid/i.test(head) && /facet\s+normal/i.test(head) && /vertex/i.test(head)) return null;
  return 'Not a valid STL (binary size does not match its triangle count, and it is not ASCII STL).';
}

export function handleStlUpload(req, res, dir, { max = MAX_STL_BYTES } = {}) {
  const url = new URL(req.url, 'http://localhost');
  const reply = (status, body, after) => {
    res.writeHead(status, { 'content-type': 'application/json; charset=utf-8', connection: 'close' }).end(JSON.stringify(body), after);
  };
  const name = url.searchParams.get('name');
  const bad = checkStlName(name);
  if (bad) return reply(400, { error: bad }, () => req.destroy());
  const tooBig = () => reply(413, { error: `STL is over ${Math.round(max / 1048576)} MB.` }, () => req.destroy());
  if (Number(req.headers['content-length']) > max) return tooBig();
  const chunks = [];
  let size = 0, dead = false;
  req.on('data', (d) => {
    if (dead) return;
    size += d.length;
    if (size > max) { dead = true; chunks.length = 0; tooBig(); return; }
    chunks.push(d);
  });
  req.on('end', () => {
    if (dead) return;
    const buf = Buffer.concat(chunks);
    const problem = checkStlHeader(buf);
    if (problem) return reply(400, { error: problem });
    try {
      fs.mkdirSync(dir, { recursive: true });
      fs.writeFileSync(path.join(dir, name), buf, { flag: url.searchParams.get('overwrite') === '1' ? 'w' : 'wx' });
    } catch (err) {
      if (err.code === 'EEXIST') return reply(409, { error: `${name} already exists.` });
      return reply(500, { error: `Could not save: ${err.message}` });
    }
    reply(201, { ok: true, name, bytes: buf.length });
  });
  req.on('error', () => {});
}
