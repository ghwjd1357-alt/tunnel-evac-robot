/* ═══════════════════════════════════════════════════════════════════
   dom_stub.mjs — 브라우저 없이 화면 로직을 돌리기 위한 최소 DOM (2026-09-02)

   실제 브라우저를 못 띄우는 환경(헤드리스 크롬 부재)에서 화면 모듈이
   "무슨 값을 어디에 쓰는지"를 검증한다. index.html 의 id 를 읽어와
   그 id 들만 존재하는 가짜 문서를 만든다 — HTML 과 어긋나면 즉시 드러난다.
   ═══════════════════════════════════════════════════════════════════ */
import fs from 'node:fs';
import path from 'node:path';

const HTML = fs.readFileSync(path.join(import.meta.dirname, '..', 'index.html'), 'utf8');
const IDS = [...HTML.matchAll(/id="([^"]+)"/g)].map(m => m[1]);

class El {
  constructor(id = '') {
    this.id = id; this._text = ''; this.innerHTML = ''; this.className = '';
    this.disabled = false; this.value = ''; this.dataset = {}; this.style = {};
    this.children = []; this._parent = null;
    this.classList = {
      _s: new Set(),
      add: c => this.classList._s.add(c),
      remove: c => this.classList._s.delete(c),
      toggle: (c, on) => { on === undefined ? (this.classList._s.has(c) ? this.classList._s.delete(c) : this.classList._s.add(c))
                                            : (on ? this.classList._s.add(c) : this.classList._s.delete(c)); },
      contains: c => this.classList._s.has(c),
    };
    this.clientWidth = 900; this.clientHeight = 500;
  }
  get parentElement() { return this._parent ??= new El('__parent__'); }
  set textContent(v) { this._text = String(v); }
  get textContent() { return this._text; }
  addEventListener() {}
  getContext() { return new Ctx(); }
  getBoundingClientRect() { return { left: 0, top: 0, width: 900, height: 500 }; }
  querySelectorAll() { return []; }
  appendChild(c) { this.children.push(c); c._parent = this; return c; }
  insertBefore(c, _ref) { this.children.push(c); c._parent = this; return c; }
  removeChild(c) { this.children = this.children.filter(x => x !== c); return c; }
}

/* canvas 2d 컨텍스트 — 호출만 삼킨다 */
class Ctx {
  constructor() { return new Proxy(this, { get: (t, k) => (k in t ? t[k] : () => {}) }); }
  createImageData(w, h) { return { data: new Uint8ClampedArray(w * h * 4) }; }
  putImageData() {} save() {} restore() {}
}

export const elements = new Map(IDS.map(id => [id, new El(id)]));
const missing = new Set();

export const document = {
  getElementById(id) {
    if (!elements.has(id)) { missing.add(id); elements.set(id, new El(id)); }
    return elements.get(id);
  },
  querySelectorAll(sel) {
    /* 진행 막대 step 처럼 동적으로 만들어지는 것은 빈 배열로 둔다 (렌더 경로만 확인) */
    if (sel.includes('[data-menu]')) return [...'main diag record emergency'.split(' ')]
      .map(m => Object.assign(new El(), { dataset: { menu: m } }));
    if (sel.includes('[data-layer]')) return ['scan', 'trail', 'plan', 'cost']
      .map(l => Object.assign(new El(), { dataset: { layer: l } }));
    if (sel.includes('[data-dir]')) return ['fwd', 'back', 'left', 'right']
      .map(d => Object.assign(new El(), { dataset: { dir: d } }));
    if (sel === '.screen') return [];
    return [];
  },
  createElement: () => new El(),
  addEventListener() {}, hidden: false,
  body: new El('__body__'),          // 실제 DOM 에는 항상 있다

  documentElement: { style: {} },
};

const listeners = {};
export const window = {
  addEventListener: (ev, fn) => { (listeners[ev] ||= []).push(fn); },
  dispatchEvent: e => { (listeners[e.type] || []).forEach(f => f(e)); return true; },
  location: { hostname: 'localhost' },
};
export function fireWindow(type, detail) { window.dispatchEvent({ type, detail }); }
export function missingIds() { return [...missing]; }

/* tokens.css 의 색을 읽어 canvas 쪽 getComputedStyle 을 대신한다 */
const TOKENS = fs.readFileSync(path.join(import.meta.dirname, '..', 'css', 'tokens.css'), 'utf8');
export const getComputedStyle = () => ({
  getPropertyValue: n => (TOKENS.match(new RegExp(n.replace('--', '--') + '\\s*:\\s*([^;]+);')) || [, '#000000'])[1].trim(),
});
