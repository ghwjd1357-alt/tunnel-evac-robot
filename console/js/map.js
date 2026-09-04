/* ═══════════════════════════════════════════════════════════════════
   map.js — 지도·위치 뷰 (2026-09-02)

   ── 그리는 순서 (뒤 → 앞) ───────────────────────────────────────
     지도 → costmap(선택) → 지나온 길 → 계획 경로 → 라이다 점군
     → 화재 마커 → 대피자 마커 → 로봇
   앞에 그릴수록 중요한 것이다. 로봇이 항상 맨 위.

   ── 좌표 함정 (07-19 에 밟은 것 — 그대로 유지) ─────────────────
   OccupancyGrid 의 row 0 은 아래쪽(y-)인데 캔버스 y 는 아래로 증가한다.
   → 지도 이미지를 만들 때 상하 반전해서 저장하고, 화면각 = -yaw 로 그린다.

   ── 색 ─────────────────────────────────────────────────────────
   canvas 는 CSS 변수를 직접 못 쓴다 → 시작할 때 한 번 읽어 캐시한다.
   그래야 tokens.css 한 곳만 고쳐도 지도까지 같이 바뀐다.
   ═══════════════════════════════════════════════════════════════════ */

import { state, quiet } from './state.js';
import { getMapOdom } from './ros.js';
import { pushLog } from './log.js';

let canvas, ctx;
let mapImage = null, mapInfo = null, mapBounds = null;   // 지도 렌더 캐시 + 실제 탐사 영역
let costImage = null, costInfo = null;
let view = { scale: 1, ox: 0, oy: 0 };    // 지도px → 화면px
export const layers = { scan: true, trail: true, cost: false, plan: true };

let C = {};   // 색 캐시

/** 디스플레이 모드인가 — body 가 아직 없어도 렌더가 죽지 않게 */
const isDisplay = () => !!document.body?.classList?.contains('display');

function readColors() {
  const cs = getComputedStyle(document.documentElement);
  const g = n => cs.getPropertyValue(n).trim();
  C = {
    bg: g('--bg'), panel: g('--panel'), line: g('--line'),
    dim: g('--dim'), dimmer: g('--dimmer'), text: g('--text'),
    ok: g('--ok'), warn: g('--warn'), alarm: g('--alarm'), accent: g('--accent'),
    /* 지도 3톤 — 무채색. 벽이 가장 밝고, 미탐사가 가장 어둡다 */
    /* 🔴 도면의 주인공은 '벽'이다. (2026-09-03 재설계)
       자유공간을 밝게 칠하면 라이다가 문틈으로 새어 본 부채꼴이 같이 밝아져
       화면이 얼룩덜룩해진다. 그 조각들은 실제 데이터라 지울 수 없으므로,
       '지우는' 대신 '덜 강조한다' — 정보는 그대로 두고 시각 위계만 바꾼다.
         자유공간 = 배경보다 아주 조금만 밝게 (있다는 것만 보이게)
         벽       = 확실히 밝게 (이것만 눈에 들어오게) */
    free: '#1c2226', wall: '#e6ebee',
  };
}

/* ═══ 지도 정리 ═══════════════════════════════════════════════════
   SLAM 원본을 그대로 그리면 관제 화면이 아니라 센서 덤프가 된다.
   다만 **벽의 모양은 한 픽셀도 건드리지 않는다** — 문 오목부·기둥·요철은
   로봇이 실제로 그린 구조이고, 그것이 이 지도의 증거력이다.
   지우는 것은 "벽이 아닌 것"뿐이다.

     ① 미탐사(-1) → 배경색. 정보가 0인 영역이 화면의 절반을 먹지 않게.
     ② 고립 점유셀 제거 — 이웃 점유가 1개 이하인 점. 반사 노이즈다.
        문 오목부는 최소 수십 셀이라 이 기준에 걸리지 않는다.
     ③ 자유공간의 '새어나간 조각' 제거 — 라이다가 문틈·유리로 밖을 본 부채꼴.
        침식(2셀) → 연결성분 → 큰 것만 남김 → 팽창(2셀) 복원.
        복도 폭은 2.35 m = 47셀이라 2셀 침식으로는 끊기지 않는다.
     ④ 벽 1셀 팽창 — 선이 끊겨 보이지 않게. 문 열린 폭(18셀)은 안 메워진다.
   ═══════════════════════════════════════════════════════════════ */

const OCC_MIN_NEIGHBORS = 2;   // 이웃 점유가 이보다 적으면 노이즈
const FREE_ERODE        = 2;   // 새어나간 조각을 끊는 침식 반경(셀)
const FREE_MIN_AREA     = 400; // 남길 자유공간 덩어리 최소 넓이(셀) = 1 m²
/* 🔴 복도의 정의 (2026-09-03 사용자 기준):
   **양쪽 벽이 다 있는 구간까지가 복도다.**
   복도 끝으로 가면 한쪽 벽이 끊기는 지점이 나오는데, 그 바깥은 상정한 복도가 아니다.

   판정: 자유공간 셀에서 상·하 **양쪽 모두**에 벽이 있거나,
         좌·우 **양쪽 모두**에 벽이 있으면 복도 안이다. 한쪽만 있으면 복도 밖.
   → 가로 복도는 상/하로, 세로 통로는 좌/우로 걸린다. 축 방향을 몰라도 된다.

   탐색 거리: 복도 폭 2.35 m = 47셀. 한쪽 벽에 붙은 셀도 반대쪽까지 닿아야 하므로
   여유를 둬 55셀(2.75 m). 연결통로 1.60 m = 32셀은 넉넉히 들어온다.

   ⚠ 벽을 새로 그리지는 않는다 — 관측한 적 없는 구조를 만들면 관제가 거짓말을 한다.
      여기서 하는 것은 **복도 밖을 안 그리는** 것뿐이다. */
const CORRIDOR_SPAN = 55;
/* 🔴 '복도 판정에 쓸 벽'은 구조물이어야 한다.
   부채꼴 안에 흩뿌려진 점 몇 개가 벽으로 세어지면 거리가 가깝게 나와 판정이 무력해진다
   (첫 시도에서 4.4% 만 잘렸다). 연결성분이 이 크기 미만인 점 무리는 판정에서 뺀다.
   실터널 벽 한 줄: 위 복도 20.75 m = 415셀 · 연결통로 8.25 m = 165셀 → 40 은 넉넉히 안전.
   ⚠ 2026-09-03: 처음엔 판정에서만 뺐는데 화면이 난장판이라 **그리기에서도 뺀다.**
      지워지는 것은 벽에 안 붙은 2 m 미만의 점 무리 = 반사·유리 너머·동적 장애물.
      문 요철·기둥은 벽에 붙어 있어 한 덩어리이므로 이 기준에 걸리지 않는다. */
const OCC_STRUCT_MIN_AREA = 40;
const FREE_ISLAND_MIN_AREA = 2000;  // 5 m² 미만의 떨어진 자유공간 조각 = 복도가 아니다
/* 🔴 교차점 보정.
   위·아래 복도와 연결통로가 만나는 T자 지점은 **어느 축으로도 양쪽 벽이 없다**
   (위로는 복도 벽이 있지만 아래로는 통로가 열려 있다) → 판정에서 탈락해 구멍이 뚫린다.
   닫힘 연산(팽창 후 침식)으로 메운다. 사방이 복도로 둘러싸인 구멍만 채워지고,
   복도 끝처럼 한쪽만 열린 경계는 팽창한 만큼 침식으로 되돌아와 넓어지지 않는다.
   연결통로 폭 1.60 m = 32셀 → 18 이면 양쪽에서 36셀을 이어 붙여 메운다. */
const JUNCTION_CLOSE = 18;

/* 🔴 임무 운용 구역 (map 좌표 m) — 이 밖은 그리지 않는다.
   근거 = src/mission_manager/config/waypoints_real_H.yaml `corridor_graph`:
     up_west (0.50, -0.08) · up_junc (8.95, -0.08) · up_east (12.99, -0.10)
     low_west(0.50,-10.65) · low_junc(8.95,-10.65) · low_east(12.45,-10.97)
     화재 (12.50, -0.10) · 집결 (10.50, -0.08) · 탈출 (0.50, -10.65)
   즉 임무가 정의한 범위는 x 0.50~12.99 · y -10.97~-0.08 이다. 여유 1.5 m.

   ⚠ 지도 동쪽 끝(x 18 m 근처)에는 실제로 양쪽 벽이 다 있지만 **임무 구역이 아니다.**
      벽이 있다/없다가 아니라 **운용 구역인가**로 자르는 것이므로 기준을 여기 명시한다.
      운용 구역이 바뀌면 waypoints 와 이 값을 같이 고친다. */
const OPERATION_AREA = { x0: -1.0, x1: 14.5, y0: -12.5, y1: 1.4 };
const WALL_DILATE       = 0;   // 🔴 0. 셀 하나가 화면에서 3~4px 이라
                               //    팽창하면 벽이 통나무가 되고 문 요철이 뭉갠다

/** 분리형 침식/팽창 — 반경 r 의 정사각 구조요소. O(w*h*r*2) */
function morph(src, w, h, r, dilate) {
  const tmp = new Uint8Array(w * h), out = new Uint8Array(w * h);
  const pick = dilate ? (a, b) => a | b : (a, b) => a & b;
  for (let y = 0; y < h; y++) {                       // 가로
    for (let x = 0; x < w; x++) {
      let v = dilate ? 0 : 1;
      for (let d = -r; d <= r; d++) {
        const xx = x + d;
        v = pick(v, (xx < 0 || xx >= w) ? (dilate ? 0 : 1) : src[y * w + xx]);
      }
      tmp[y * w + x] = v;
    }
  }
  for (let x = 0; x < w; x++) {                       // 세로
    for (let y = 0; y < h; y++) {
      let v = dilate ? 0 : 1;
      for (let d = -r; d <= r; d++) {
        const yy = y + d;
        v = pick(v, (yy < 0 || yy >= h) ? (dilate ? 0 : 1) : tmp[yy * w + x]);
      }
      out[y * w + x] = v;
    }
  }
  return out;
}

/**
 * "양쪽 벽 사이에 있는 셀"을 고른다.
 *
 * 각 셀에서 좌/우/상/하 네 방향으로 가장 가까운 벽까지의 칸 수를 구한 뒤,
 *   (위·아래 둘 다 span 이내)  또는  (좌·우 둘 다 span 이내)
 * 이면 복도 안으로 본다. 한쪽만 있으면 — 즉 복도 끝에서 벽 하나가 끊기면 — 밖이다.
 *
 * 네 방향 거리는 행/열마다 선형으로 두 번씩만 훑으면 되므로 O(w*h) 다.
 */
function betweenWalls(occ, w, h, span) {
  const n = w * h, INF = 1 << 24;
  const L = new Int32Array(n), R = new Int32Array(n),
        U = new Int32Array(n), D = new Int32Array(n);
  for (let y = 0; y < h; y++) {                    // 좌 → 우
    let d = INF;
    for (let x = 0; x < w; x++) { const i = y * w + x; d = occ[i] ? 0 : d + 1; L[i] = d; }
    d = INF;
    for (let x = w - 1; x >= 0; x--) { const i = y * w + x; d = occ[i] ? 0 : d + 1; R[i] = d; }
  }
  for (let x = 0; x < w; x++) {                    // 상 → 하
    let d = INF;
    for (let y = 0; y < h; y++) { const i = y * w + x; d = occ[i] ? 0 : d + 1; U[i] = d; }
    d = INF;
    for (let y = h - 1; y >= 0; y--) { const i = y * w + x; d = occ[i] ? 0 : d + 1; D[i] = d; }
  }
  const out = new Uint8Array(n);
  for (let i = 0; i < n; i++) {
    out[i] = ((U[i] <= span && D[i] <= span) || (L[i] <= span && R[i] <= span)) ? 1 : 0;
  }
  return out;
}

/** 연결성분(4-이웃)에서 minArea 미만인 덩어리를 지운다 */
function keepLarge(mask, w, h, minArea) {
  const seen = new Uint8Array(w * h), out = new Uint8Array(w * h);
  const stack = new Int32Array(w * h);
  const comp  = new Int32Array(w * h);
  for (let i = 0; i < w * h; i++) {
    if (!mask[i] || seen[i]) continue;
    let sp = 0, n = 0;
    stack[sp++] = i; seen[i] = 1;
    while (sp) {
      const c = stack[--sp]; comp[n++] = c;
      const cx = c % w, cy = (c / w) | 0;
      if (cx > 0     && mask[c - 1] && !seen[c - 1]) { seen[c - 1] = 1; stack[sp++] = c - 1; }
      if (cx < w - 1 && mask[c + 1] && !seen[c + 1]) { seen[c + 1] = 1; stack[sp++] = c + 1; }
      if (cy > 0     && mask[c - w] && !seen[c - w]) { seen[c - w] = 1; stack[sp++] = c - w; }
      if (cy < h - 1 && mask[c + w] && !seen[c + w]) { seen[c + w] = 1; stack[sp++] = c + w; }
    }
    if (n >= minArea) for (let k = 0; k < n; k++) out[comp[k]] = 1;
  }
  return out;
}

/* ── OccupancyGrid → 오프스크린 캔버스 (수신할 때만. 매 프레임 재렌더는 비싸다)
   동시에 실제로 탐사된 칸의 경계를 잰다 — 689x382 중 복도는 절반도 안 된다. ── */
function renderGrid(msg, palette, measure = false) {
  const w = msg.info.width, h = msg.info.height, n = w * h;
  const off = document.createElement('canvas');
  off.width = w; off.height = h;
  const octx = off.getContext('2d');
  const img = octx.createImageData(w, h);

  /* ⓪ 임무 운용 구역 밖은 아예 읽지 않는다 (위 OPERATION_AREA 주석 참조) */
  const res = msg.info.resolution, ox = msg.info.origin.position.x, oy = msg.info.origin.position.y;
  const c0 = Math.max(0,     Math.floor((OPERATION_AREA.x0 - ox) / res));
  const c1 = Math.min(w - 1, Math.ceil ((OPERATION_AREA.x1 - ox) / res));
  const r0 = Math.max(0,     Math.floor((OPERATION_AREA.y0 - oy) / res));
  const r1 = Math.min(h - 1, Math.ceil ((OPERATION_AREA.y1 - oy) / res));

  /* ① 원본을 세 갈래로 나눈다 */
  let occ = new Uint8Array(n), free = new Uint8Array(n);
  for (let i = 0; i < n; i++) {
    const v = msg.data[i];
    if (v === -1 || v === 255) continue;              // 미탐사 — 어느 쪽도 아니다
    if (measure) {                                    // 정리 경로에서만 구역을 적용 (costmap 은 그대로)
      const col = i % w, row = (i / w) | 0;
      if (col < c0 || col > c1 || row < r0 || row > r1) continue;
    }
    if (v >= 65) occ[i] = 1; else free[i] = 1;
  }

  if (measure) {
    /* ② 고립 점유셀 제거 — 벽의 모양은 그대로 두고 '떠 있는 점'만 없앤다 */
    const occClean = new Uint8Array(n);
    for (let i = 0; i < n; i++) {
      if (!occ[i]) continue;
      const x = i % w, y = (i / w) | 0;
      let k = 0;
      for (let dy = -1; dy <= 1; dy++) for (let dx = -1; dx <= 1; dx++) {
        if (!dx && !dy) continue;
        const xx = x + dx, yy = y + dy;
        if (xx >= 0 && xx < w && yy >= 0 && yy < h && occ[yy * w + xx]) k++;
      }
      if (k >= OCC_MIN_NEIGHBORS) occClean[i] = 1;
    }
    occ = occClean;

    /* ③ 자유공간: 침식 → 큰 덩어리만 → 팽창 (원래 자유공간 밖으로는 안 번지게) */
    const eroded = morph(free, w, h, FREE_ERODE, false);
    const kept   = keepLarge(eroded, w, h, FREE_MIN_AREA);
    const back   = morph(kept, w, h, FREE_ERODE, true);
    for (let i = 0; i < n; i++) free[i] = (back[i] && free[i]) ? 1 : 0;

    /* ③-b 구조물 벽만 남긴다 (판정 기준이자 그리는 대상) */
    occ = keepLarge(occ, w, h, OCC_STRUCT_MIN_AREA);

    /* ③-c 양쪽 벽 사이에 있는 자유공간만 남긴다 = 복도 끝에서 벽이 끊기면 그 바깥은 버린다 */
    let inside = betweenWalls(occ, w, h, CORRIDOR_SPAN);
    /* T자 교차점 메우기 (위 주석 참조) */
    inside = morph(morph(inside, w, h, JUNCTION_CLOSE, true), w, h, JUNCTION_CLOSE, false);
    for (let i = 0; i < n; i++) if (!inside[i]) free[i] = 0;

    /* ③-d 복도에서 떨어져 나간 조각 제거. 복도 본체는 5만 셀이 넘어 안 걸린다. */
    free = keepLarge(free, w, h, FREE_ISLAND_MIN_AREA);

    /* ④-b 남은 복도에 닿지 않는 벽 조각도 지운다 —
       복도 밖의 점은 사용자가 상정한 구조가 아니다. */
    const near = morph(free, w, h, 4, true);
    for (let i = 0; i < n; i++) if (occ[i] && !near[i]) occ[i] = 0;

    /* ④ 벽 팽창 (WALL_DILATE=0 이면 건너뛴다) */
    if (WALL_DILATE > 0) occ = morph(occ, w, h, WALL_DILATE, true);
  }

  /* ⑤ 칠하기 (상하 반전: grid row 0 = 아래쪽) + 탐사 경계 측정 */
  let cMin = w, cMax = -1, rMin = h, rMax = -1;
  for (let i = 0; i < n; i++) {
    const row = (i / w) | 0, col = i % w;
    const j = ((h - 1 - row) * w + col) * 4;
    const known = occ[i] || free[i];
    if (measure && known) {
      if (col < cMin) cMin = col; if (col > cMax) cMax = col;
      if (row < rMin) rMin = row; if (row > rMax) rMax = row;
    }
    const c = palette(occ[i] ? 100 : free[i] ? 0 : -1);
    if (!c) { img.data[j + 3] = 0; continue; }
    img.data[j] = c[0]; img.data[j + 1] = c[1]; img.data[j + 2] = c[2]; img.data[j + 3] = c[3] ?? 255;
  }
  octx.putImageData(img, 0, 0);

  if (measure) {
    const M = 8;
    mapBounds = cMax < 0 ? null : {
      x0: Math.max(0, cMin - M), x1: Math.min(w - 1, cMax + M),
      y0: Math.max(0, h - 1 - rMax - M), y1: Math.min(h - 1, h - 1 - rMin + M),
    };
  }
  return off;
}

const hex2rgb = h => [parseInt(h.slice(1, 3), 16), parseInt(h.slice(3, 5), 16), parseInt(h.slice(5, 7), 16)];

const MAP_PALETTE = v => {
  if (v === -1) return null;                 // 미탐사 = 투명 (배경이 비친다)
  if (v >= 65)  return hex2rgb(C.wall);      // 벽
  return hex2rgb(C.free);                    // 주행 가능
};

/* costmap 은 "부풀린 장애물" = 로봇이 못 가는 영역. 겹쳐 놓되 아주 흐리게. */
const COST_PALETTE = v => {
  if (v < 65) return null;                       // 투명 (정리 경로에서 free 는 0 으로 온다)
  const [r, g, b] = hex2rgb(C.warn);
  return [r, g, b, 110];
};

/* ── 좌표 변환 ────────────────────────────────────────────────── */
/** 화면을 '탐사된 영역'에 맞춘다. 경계를 못 쟀으면 지도 전체에 맞춘다. */
function fitView() {
  if (!mapInfo) return;
  const cw = canvas.width, ch = canvas.height;
  const b = mapBounds ?? { x0: 0, y0: 0, x1: mapInfo.width - 1, y1: mapInfo.height - 1 };
  const bw = b.x1 - b.x0 + 1, bh = b.y1 - b.y0 + 1;
  const s = Math.min(cw / bw, ch / bh);
  view = { scale: s,
           ox: (cw - bw * s) / 2 - b.x0 * s,
           oy: (ch - bh * s) / 2 - b.y0 * s };
}

/** map 미터 → 화면 픽셀 */
function m2p(x, y) {
  const cx = (x - mapInfo.origin.position.x) / mapInfo.resolution;
  const cy = (y - mapInfo.origin.position.y) / mapInfo.resolution;
  return [view.ox + cx * view.scale, view.oy + (mapInfo.height - cy) * view.scale];
}

/** 화면 픽셀 → map 미터 */
function p2m(px, py) {
  const cx = (px - view.ox) / view.scale;
  const cy = mapInfo.height - (py - view.oy) / view.scale;
  return [mapInfo.origin.position.x + cx * mapInfo.resolution,
          mapInfo.origin.position.y + cy * mapInfo.resolution];
}

/* ── 그리기 ───────────────────────────────────────────────────── */
export function draw() {
  if (!canvas) return;
  const host = canvas.parentElement;
  if (canvas.width !== host.clientWidth || canvas.height !== host.clientHeight) {
    canvas.width = host.clientWidth; canvas.height = host.clientHeight;
    fitView();
  }
  ctx.fillStyle = C.bg;
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  if (!mapImage || !mapInfo) {
    ctx.fillStyle = C.dimmer; ctx.font = '13px sans-serif'; ctx.textAlign = 'center';
    ctx.fillText('지도 수신 대기', canvas.width / 2, canvas.height / 2);
    return;
  }

  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(mapImage, view.ox, view.oy,
                mapInfo.width * view.scale, mapInfo.height * view.scale);

  /* costmap (선택)
     🔴 로컬 costmap 의 frame_id 는 **odom** 이다 (nav2 global_frame: odom · rolling_window).
        map 좌표로 알고 그렸더니 SLAM 이 보정할 때마다 어긋났고, 연결통로처럼 보정이
        큰 구간에서 눈에 띄게 밀리고 비틀렸다 (2026-09-04 확인).
        → map←odom 변환으로 원점을 옮기고, 회전 성분은 캔버스를 돌려서 반영한다.
        origin 은 격자의 **좌하단**이고, 이미지는 상하 반전해 저장돼 있으므로
        좌하단을 기준점으로 잡고 위로 hpx 만큼 올려 그린다. */
  if (layers.cost && costImage && costInfo) {
    const T = getMapOdom();
    if (T) {
      const o = costInfo.origin.position;
      const c = Math.cos(T.yaw), sn = Math.sin(T.yaw);
      const ox = T.x + c * o.x - sn * o.y;      // odom → map
      const oy = T.y + sn * o.x + c * o.y;
      const wpx = costInfo.width  * costInfo.resolution / mapInfo.resolution * view.scale;
      const hpx = costInfo.height * costInfo.resolution / mapInfo.resolution * view.scale;
      const [px, py] = m2p(ox, oy);
      ctx.save();
      ctx.translate(px, py);
      ctx.rotate(-T.yaw);                        // 화면 y 가 반전이라 부호가 반대
      ctx.drawImage(costImage, 0, -hpx, wpx, hpx);
      ctx.restore();
    }
  }

  /* 지나온 길 — 로봇이 실제로 지난 자취. 흐린 무채색으로 뒤에 깐다 */
  if (layers.trail && state.trail.length > 1) {
    /* 🔴 1 m 넘게 튄 곳에서는 선을 끊는다.
       bag 되감기나 재측위로 로봇 위치가 순간이동하면 화면을 가로지르는
       직선이 그려져 실제로 지나간 길처럼 보인다 (2026-09-03 화면에서 확인). */
    ctx.strokeStyle = C.dimmer; ctx.lineWidth = 1.5; ctx.beginPath();
    let prev = null;
    for (const [x, y] of state.trail) {
      const [px, py] = m2p(x, y);
      if (prev && Math.hypot(x - prev[0], y - prev[1]) > 1.0) ctx.moveTo(px, py);
      else if (prev) ctx.lineTo(px, py);
      else ctx.moveTo(px, py);
      prev = [x, y];
    }
    ctx.stroke();
  }

  /* 계획 경로 — 앞으로 갈 길 */
  if (layers.plan && state.planPts.length > 1) {
    ctx.strokeStyle = C.accent; ctx.lineWidth = 2; ctx.setLineDash([6, 4]); ctx.beginPath();
    state.planPts.forEach(([x, y], i) => { const [px, py] = m2p(x, y); i ? ctx.lineTo(px, py) : ctx.moveTo(px, py); });
    ctx.stroke(); ctx.setLineDash([]);
  }

  /* 라이다 점군 — 로봇이 '지금' 보고 있는 것.
     🔴 지도와 같은 임무 구역으로 자른다. (2026-09-03)
        라이다는 16 m 까지 보므로 복도 축으로 나간 먼 반사가 잘라낸 지도 바깥
        검은 공간에 찍힌다. 좌표는 맞지만, 제자리 회전 중에 그 점들이 레이더 빔처럼
        쓸고 지나가 '점군 전체가 돈다'로 보인다. 실측: 180° 회전 두 시점의
        점군 최근접거리 0.072 m — 변환 자체는 정상이었다. */
  if (layers.scan && state.scanPts.length) {
    /* 영상에서 보이려면 점이 벽보다 확실히 튀어야 한다 (첫 확인에서 거의 안 보였다) */
    ctx.fillStyle = C.accent; ctx.globalAlpha = 0.85;
    const r = Math.max(2, view.scale * 1.1);
    const A = OPERATION_AREA;
    for (const [x, y] of state.scanPts) {
      if (x < A.x0 || x > A.x1 || y < A.y0 || y > A.y1) continue;   // 구역 밖은 안 그린다
      const [px, py] = m2p(x, y);
      ctx.fillRect(px - r / 2, py - r / 2, r, r);
    }
    ctx.globalAlpha = 1;
  }

  /* 화재 — 마름모 (이모지 쓰지 않는다) */
  if (state.fireXY) marker(state.fireXY.x, state.fireXY.y, C.alarm, 'diamond', '화재');
  /* 대피자 신고 지점 */
  if (state.victim)  marker(state.victim.x,  state.victim.y,  C.warn,  'square',  '대피자');

  /* 로봇 — 항상 맨 위. 진행방향 삼각형 (화면각 = -yaw) */
  if (state.robot) {
    const [px, py] = m2p(state.robot.x, state.robot.y);
    ctx.save(); ctx.translate(px, py); ctx.rotate(-state.robot.yaw);
    const K = isDisplay() ? 1.8 : 1;
    ctx.fillStyle = C.text;
    ctx.beginPath(); ctx.moveTo(11*K, 0); ctx.lineTo(-7*K, 6.5*K); ctx.lineTo(-7*K, -6.5*K);
    ctx.closePath(); ctx.fill();
    ctx.restore();
    /* 로봇 둘레 얇은 원 = 눈이 즉시 찾을 수 있게 */
    ctx.strokeStyle = C.accent; ctx.lineWidth = K;
    ctx.beginPath(); ctx.arc(px, py, 13*K, 0, Math.PI * 2); ctx.stroke();
  }
}

function marker(x, y, color, shape, label) {
  /* 디스플레이(7인치)에서는 마커가 안 보인다 — 원안대로 1.8배 (feature/display) */
  const K = isDisplay() ? 1.8 : 1;
  const [px, py] = m2p(x, y);
  ctx.fillStyle = color; ctx.strokeStyle = color; ctx.lineWidth = 1.5;
  ctx.beginPath();
  if (shape === 'diamond') {
    ctx.moveTo(px, py - 8*K); ctx.lineTo(px + 8*K, py); ctx.lineTo(px, py + 8*K); ctx.lineTo(px - 8*K, py);
  } else {
    ctx.rect(px - 6*K, py - 6*K, 12*K, 12*K);
  }
  ctx.closePath(); ctx.fill();
  ctx.font = `${Math.round(11*K)}px sans-serif`;
  ctx.textAlign = 'center'; ctx.fillText(label, px, py - 13*K);
}

/* ── 배선 ─────────────────────────────────────────────────────── */
export function setupMap() {
  readColors();
  canvas = document.getElementById('map-canvas');
  ctx = canvas.getContext('2d');

  /* /map 은 2초마다 온다. 정리 연산(침식·연결성분)이 무거우므로 3초에 한 번만 다시 만든다.
     지도는 천천히 자라므로 이 정도로 충분하다. */
  let lastGridAt = 0;
  window.addEventListener('map:grid', e => {
    const first = !mapInfo;
    /* 🔴 mapInfo(좌표계) 와 mapImage(그림) 는 **반드시 같이** 갱신한다.
       SLAM 이 지도를 넓히면 origin·크기가 바뀐다. 예전엔 좌표만 매번 갱신하고
       그림은 3초에 한 번만 다시 그려서, 배경은 3초 전 것인데 로봇·경로·라이다는
       최신 좌표로 찍혀 어긋났다 (2026-09-03 '라이다가 지도랑 안 맞는다'의 원인).
       회전 중에는 지도가 빨리 자라 어긋남이 특히 크게 보였다. */
    if (!first && Date.now() - lastGridAt < 3000) return;
    lastGridAt = Date.now();
    mapInfo = e.detail.info;
    mapImage = renderGrid(e.detail, MAP_PALETTE, true);   // true = 정리 + 경계 측정
    fitView();
  });
  window.addEventListener('map:costmap', e => {
    costInfo = e.detail.info;
    costImage = renderGrid(e.detail, COST_PALETTE);
  });

  /* 지도 클릭 → 화재 좌표 후보. 즉시 발행하지 않는다(오클릭 방지 — 07-19 결정) */
  canvas.addEventListener('click', ev => {
    if (!mapInfo) return;
    const r = canvas.getBoundingClientRect();
    const [x, y] = p2m(ev.clientX - r.left, ev.clientY - r.top);
    quiet({ pickXY: { x, y } });
    document.getElementById('fire-x').value = x.toFixed(2);
    document.getElementById('fire-y').value = y.toFixed(2);
    pushLog('PICK', '화재 좌표 지정', `${x.toFixed(2)}  ${y.toFixed(2)}`, 'ctrl');
  });

  /* 마우스를 올린 지점의 좌표를 오른쪽 아래에 표시 */
  canvas.addEventListener('mousemove', ev => {
    if (!mapInfo) return;
    const r = canvas.getBoundingClientRect();
    const [x, y] = p2m(ev.clientX - r.left, ev.clientY - r.top);
    document.getElementById('map-coord').textContent = `${x.toFixed(2)}  ${y.toFixed(2)}`;
  });

  /* 레이어 켜고 끄기 */
  for (const btn of document.querySelectorAll('#map-toggles button')) {
    btn.onclick = () => {
      const k = btn.dataset.layer;
      layers[k] = !layers[k];
      btn.classList.toggle('on', layers[k]);
    };
    btn.classList.toggle('on', layers[btn.dataset.layer]);
  }

  window.addEventListener('resize', fitView);
}
