import test from 'node:test';
import assert from 'node:assert/strict';
import http from 'node:http';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { checkStlName, checkStlHeader, handleStlUpload } from './stlUpload.mjs';

const binaryStl = (n) => { const b = Buffer.alloc(84 + 50 * n); b.writeUInt32LE(n, 80); return b; };
const asciiStl = Buffer.from('solid x\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\nendloop\nendfacet\nendsolid x\n');

test('names: only a plain basename ending in .stl is accepted', () => {
  assert.equal(checkStlName('body.stl'), null);
  assert.equal(checkStlName('My_part-2.STL'), null);
  for (const bad of ['../x.stl', 'a/b.stl', 'a\\b.stl', '..stl', 'a..b.stl', 'x.txt', 'x.stl.txt', '.stl', '', null, undefined, 'x y.stl', 'a'.repeat(120) + '.stl', 'x.stl\0.txt'])
    assert.match(checkStlName(bad) || '', /./, `should reject ${JSON.stringify(bad)}`);
});

test('header: binary size must be 84 + 50n, ASCII must look like facets, garbage is rejected', () => {
  assert.equal(checkStlHeader(binaryStl(3)), null);
  assert.equal(checkStlHeader(asciiStl), null);
  assert.match(checkStlHeader(Buffer.alloc(10)), /too small/i);
  assert.match(checkStlHeader(binaryStl(0)), /no triangles/i);
  assert.match(checkStlHeader(binaryStl(3).subarray(0, 200)), /STL/);
  assert.match(checkStlHeader(Buffer.from('hello world, definitely not an stl file at all, no sir. '.repeat(3))), /STL/);
  assert.match(checkStlHeader(Buffer.from('solid x\nnothing else here in this text file for sure, nope nope nope nope nope nope nope')), /STL/);
});

test('route: status codes, no overwrite without ?overwrite=1, size cap, nothing escapes the folder', async () => {
  const dir = fs.mkdtempSync(path.join(os.tmpdir(), 'stl-'));
  const server = http.createServer((req, res) => handleStlUpload(req, res, dir, { max: 1000 }));
  await new Promise((r) => server.listen(0, '127.0.0.1', r));
  const base = `http://127.0.0.1:${server.address().port}/api/robot/stl`;
  const post = (q, body) => fetch(`${base}?${q}`, { method: 'POST', body }).then(async (r) => [r.status, await r.json().catch(() => ({}))]);
  try {
    assert.equal((await post('name=a.stl', binaryStl(2)))[0], 201);
    assert.ok(fs.existsSync(path.join(dir, 'a.stl')));
    assert.equal((await post('name=a.stl', binaryStl(3)))[0], 409);
    assert.equal(fs.statSync(path.join(dir, 'a.stl')).size, 84 + 100);
    assert.equal((await post('name=a.stl&overwrite=1', binaryStl(3)))[0], 201);
    assert.equal(fs.statSync(path.join(dir, 'a.stl')).size, 84 + 150);
    assert.equal((await post('name=..%2Fx.stl', binaryStl(1)))[0], 400);
    assert.equal((await post('name=x.txt', binaryStl(1)))[0], 400);
    assert.equal((await post('name=g.stl', Buffer.from('garbage garbage garbage garbage garbage garbage garbage garbage garbage garbage garbage')))[0], 400);
    assert.equal((await post('name=big.stl', Buffer.alloc(5000)))[0], 413);
    assert.deepEqual(fs.readdirSync(dir), ['a.stl']);
    assert.deepEqual(fs.readdirSync(path.dirname(dir)).filter((f) => f === 'x.stl'), []);
  } finally { server.close(); server.closeAllConnections(); fs.rmSync(dir, { recursive: true }); }
});
