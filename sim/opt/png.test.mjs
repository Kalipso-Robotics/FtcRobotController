import test from 'node:test';
import assert from 'node:assert/strict';
import zlib from 'node:zlib';
import { Canvas } from './png.mjs';

function decode(buf) {
  const w = buf.readUInt32BE(16), h = buf.readUInt32BE(20);
  const raw = zlib.inflateSync(buf.subarray(buf.indexOf('IDAT') + 4, buf.indexOf('IEND') - 8));
  return { w, h, px: (x, y) => [...raw.subarray(y * (w * 3 + 1) + 1 + x * 3).subarray(0, 3)] };
}

test('encode: signature, IHDR, and a rect pixel survives the round trip', () => {
  const c = new Canvas(20, 10, [255, 255, 255]);
  c.rect(2, 3, 4, 4, [10, 20, 30]);
  const buf = c.encode();
  assert.deepEqual([...buf.subarray(0, 8)], [137, 80, 78, 71, 13, 10, 26, 10]);
  assert.equal(buf.toString('latin1', 12, 16), 'IHDR');
  const img = decode(buf);
  assert.deepEqual([img.w, img.h, buf[24], buf[25]], [20, 10, 8, 2]);   // 8-bit RGB
  assert.deepEqual(img.px(3, 4), [10, 20, 30]);
  assert.deepEqual(img.px(10, 0), [255, 255, 255]);
});

test('line is thick, circle is filled, out of bounds is ignored', () => {
  const c = new Canvas(30, 30, [0, 0, 0]);
  c.line(2, 15, 27, 15, 5, [1, 2, 3]);
  c.circle(15, 5, 4, [9, 9, 9]);
  c.rect(-5, -1, 100, 2, [7, 7, 7]);
  const img = decode(c.encode());
  assert.deepEqual(img.px(15, 17), [1, 2, 3]);
  assert.deepEqual(img.px(15, 20), [0, 0, 0]);
  assert.deepEqual(img.px(15, 5), [9, 9, 9]);
  assert.deepEqual(img.px(0, 0), [7, 7, 7]);
});

test('text draws legible pixels in the given colour', () => {
  const c = new Canvas(40, 12, [255, 255, 255]);
  c.text(1, 1, 'A', [0, 0, 0], 2);
  const img = decode(c.encode());
  let dark = 0;
  for (let y = 0; y < 12; y++) for (let x = 0; x < 40; x++) if (img.px(x, y)[0] === 0) dark++;
  assert.ok(dark > 10);
});
