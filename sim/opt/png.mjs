// Tiny RGB PNG writer: rect, thick line, circle, 3x5 text, encode. Node's zlib only (crc32 needs Node 22+).
import zlib from 'node:zlib';

// 3x5 glyphs, 5 rows of 3 columns ('#' = on). Upper case, digits and the few marks the legend needs.
const ROWS = {
  A: ['.#.', '#.#', '###', '#.#', '#.#'], B: ['##.', '#.#', '##.', '#.#', '##.'], C: ['.##', '#..', '#..', '#..', '.##'],
  D: ['##.', '#.#', '#.#', '#.#', '##.'], E: ['###', '#..', '##.', '#..', '###'], F: ['###', '#..', '##.', '#..', '#..'],
  G: ['.##', '#..', '#.#', '#.#', '.##'], H: ['#.#', '#.#', '###', '#.#', '#.#'], I: ['###', '.#.', '.#.', '.#.', '###'],
  J: ['..#', '..#', '..#', '#.#', '.#.'], K: ['#.#', '#.#', '##.', '#.#', '#.#'], L: ['#..', '#..', '#..', '#..', '###'],
  M: ['#.#', '###', '###', '#.#', '#.#'], N: ['##.', '#.#', '#.#', '#.#', '#.#'], O: ['.#.', '#.#', '#.#', '#.#', '.#.'],
  P: ['##.', '#.#', '##.', '#..', '#..'], Q: ['.#.', '#.#', '#.#', '##.', '.##'], R: ['##.', '#.#', '##.', '#.#', '#.#'],
  S: ['.##', '#..', '.#.', '..#', '##.'], T: ['###', '.#.', '.#.', '.#.', '.#.'], U: ['#.#', '#.#', '#.#', '#.#', '###'],
  V: ['#.#', '#.#', '#.#', '#.#', '.#.'], W: ['#.#', '#.#', '###', '###', '#.#'], X: ['#.#', '#.#', '.#.', '#.#', '#.#'],
  Y: ['#.#', '#.#', '.#.', '.#.', '.#.'], Z: ['###', '..#', '.#.', '#..', '###'],
  0: ['###', '#.#', '#.#', '#.#', '###'], 1: ['.#.', '##.', '.#.', '.#.', '###'], 2: ['##.', '..#', '.#.', '#..', '###'],
  3: ['##.', '..#', '.#.', '..#', '##.'], 4: ['#.#', '#.#', '###', '..#', '..#'], 5: ['###', '#..', '##.', '..#', '##.'],
  6: ['.##', '#..', '###', '#.#', '###'], 7: ['###', '..#', '.#.', '.#.', '.#.'], 8: ['###', '#.#', '###', '#.#', '###'],
  9: ['###', '#.#', '###', '..#', '##.'],
  '.': ['...', '...', '...', '...', '.#.'], ':': ['...', '.#.', '...', '.#.', '...'], '-': ['...', '...', '###', '...', '...'],
  '/': ['..#', '..#', '.#.', '#..', '#..'], '(': ['.#.', '#..', '#..', '#..', '.#.'], ')': ['.#.', '..#', '..#', '..#', '.#.'],
  '+': ['...', '.#.', '###', '.#.', '...'], '=': ['...', '###', '...', '###', '...'], '%': ['#.#', '..#', '.#.', '#..', '#.#'],
  ',': ['...', '...', '...', '.#.', '#..'], '_': ['...', '...', '...', '...', '###'], '>': ['#..', '.#.', '..#', '.#.', '#..'], ' ': ['...', '...', '...', '...', '...'],
};

export class Canvas {
  constructor(w, h, bg = [255, 255, 255]) {
    this.w = w; this.h = h;
    this.data = Buffer.alloc(w * h * 3);
    for (let i = 0; i < w * h; i++) this.data.set(bg, i * 3);
  }
  px(x, y, c) {
    x = Math.round(x); y = Math.round(y);
    if (x >= 0 && y >= 0 && x < this.w && y < this.h) this.data.set(c, (y * this.w + x) * 3);
  }
  rect(x, y, w, h, c) {
    for (let j = Math.round(y); j < Math.round(y + h); j++) for (let i = Math.round(x); i < Math.round(x + w); i++) this.px(i, j, c);
  }
  outline(x, y, w, h, c, t = 1) {
    this.rect(x, y, w, t, c); this.rect(x, y + h - t, w, t, c); this.rect(x, y, t, h, c); this.rect(x + w - t, y, t, h, c);
  }
  circle(cx, cy, r, c) {
    for (let j = -r; j <= r; j++) for (let i = -r; i <= r; i++) if (i * i + j * j <= r * r) this.px(cx + i, cy + j, c);
  }
  // ponytail: stamps a disc at every step instead of a true polygon; fine for lines a few px wide
  line(x0, y0, x1, y1, thick, c) {
    const n = Math.max(1, Math.ceil(Math.hypot(x1 - x0, y1 - y0)));
    for (let i = 0; i <= n; i++) this.circle(x0 + ((x1 - x0) * i) / n, y0 + ((y1 - y0) * i) / n, Math.max(0, Math.round(thick / 2)), c);
  }
  text(x, y, s, c, scale = 2) {
    let cx = x;
    for (const ch of s.toUpperCase()) {
      (ROWS[ch] || ROWS[' ']).forEach((row, j) => [...row].forEach((on, i) => { if (on === '#') this.rect(cx + i * scale, y + j * scale, scale, scale, c); }));
      cx += 4 * scale;
    }
  }
  encode() {
    const stride = this.w * 3;
    const raw = Buffer.alloc((stride + 1) * this.h);
    for (let y = 0; y < this.h; y++) this.data.copy(raw, y * (stride + 1) + 1, y * stride, (y + 1) * stride);
    const chunk = (type, body) => {
      const head = Buffer.alloc(8); head.writeUInt32BE(body.length); head.write(type, 4, 'latin1');
      const crc = Buffer.alloc(4); crc.writeUInt32BE(zlib.crc32(Buffer.concat([head.subarray(4), body])));
      return Buffer.concat([head, body, crc]);
    };
    const ihdr = Buffer.alloc(13); ihdr.writeUInt32BE(this.w); ihdr.writeUInt32BE(this.h, 4); ihdr[8] = 8; ihdr[9] = 2;
    return Buffer.concat([Buffer.from([137, 80, 78, 71, 13, 10, 26, 10]), chunk('IHDR', ihdr), chunk('IDAT', zlib.deflateSync(raw)), chunk('IEND', Buffer.alloc(0))]);
  }
}
