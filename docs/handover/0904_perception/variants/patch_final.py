#!/usr/bin/env python3
"""최종 구성 pn_FINAL.py — 원본 미변경. TRT + 직접추적 + 디버그확대제거 + 회전 파라미터화."""
import shutil, re
SRC = "/home/hanhan/percep_ws/install/vision_pipeline/lib/python3.10/site-packages/vision_pipeline/perception_node.py"
DST = "/tmp/perftest/pn_FINAL.py"
shutil.copy(SRC, DST)
s = open(DST).read()

# ── 회전 파라미터화 ───────────────────────────────────────────────
s = s.replace("""            color_np = cv2.rotate(color_np, cv2.ROTATE_180)
            depth_np = cv2.rotate(depth_np, cv2.ROTATE_180)""",
"""            if self._rot180:
                color_np = cv2.rotate(color_np, cv2.ROTATE_180)
                depth_np = cv2.rotate(depth_np, cv2.ROTATE_180)""")
s = s.replace("        self.frames_published = 0",
              """        self.declare_parameter('rotate_180', True)
        self._rot180 = bool(self.get_parameter('rotate_180').value)
        self.frames_published = 0""", 1)

# ── fire: TensorRT 엔진 ───────────────────────────────────────────
s = s.replace("""        from rfdetr import RFDETRSmall
        self.fire_model = RFDETRSmall(pretrain_weights=fire_path,
                                      num_classes=int(g('fire_num_classes')))""",
"""        import os as _os
        from polygraphy.backend.trt import EngineFromBytes, TrtRunner
        from polygraphy.backend.common import BytesFromPath
        _eng = _os.environ.get('FIRE_ENGINE',
                               '/home/hanhan/fire_trt_export/rfdetr-small.engine')
        self.get_logger().info('Loading TRT engine %s ...' % _eng)
        self._trt = TrtRunner(EngineFromBytes(BytesFromPath(_eng)))
        self._trt.activate(); self._trt_res = 512
        self.fire_model = None""")
for m in re.finditer(r".*optimize_for_inference.*\n", s):
    s = s.replace(m.group(0), "")
    break

i = s.index("    def _infer_fire(self, frame)")
j = s.index("    def _infer_pose", i)
s = s[:i] + '''    def _infer_fire(self, frame) -> List[Tuple[str, float, Tuple]]:
        """TensorRT 엔진 추론. 🔴 후처리 정확도 미검증 — 속도 측정용."""
        import numpy as _np
        H, W = frame.shape[:2]; R = self._trt_res
        img = cv2.resize(frame, (R, R), interpolation=cv2.INTER_LINEAR)
        if not hasattr(self, '_pre_s'):
            _m = _np.array([0.485, 0.456, 0.406], _np.float32).reshape(3, 1, 1)
            _sd = _np.array([0.229, 0.224, 0.225], _np.float32).reshape(3, 1, 1)
            self._pre_s = (1.0 / 255.0) / _sd; self._pre_b = -_m / _sd
        x = img.transpose(2, 0, 1).astype(_np.float32)
        x = _np.ascontiguousarray((x * self._pre_s + self._pre_b)[None])
        o = self._trt.infer({'input': x})
        dets = o['dets'][0]; logits = o['labels'][0]
        prob = 1.0 / (1.0 + _np.exp(-logits))
        cls = prob.argmax(1); conf = prob.max(1)
        out = []
        for k in _np.nonzero(conf >= self.fire_conf)[0]:
            name = RFDETR_CLASS_NAMES.get(int(cls[k]))
            if name is None:
                continue
            cx, cy, bw, bh = dets[k]
            out.append((name, float(conf[k]),
                        (float((cx - bw / 2) * W), float((cy - bh / 2) * H),
                         float((cx + bw / 2) * W), float((cy + bh / 2) * H))))
        return out

''' + s[j:]

# ── pose: 직접 추적기 ─────────────────────────────────────────────
TRACKER = '''

class CentroidTracker:
    """중심점 그리디 매칭 + 유실 버퍼. ByteTrack 대체 (09-04 검증)."""
    def __init__(self, max_dist_frac=0.25, lost_frames=30):
        self.tracks = {}; self.next_id = 1
        self.max_dist_frac = max_dist_frac; self.lost_frames = lost_frames

    def update(self, boxes, img_w, img_h):
        import math
        gate = self.max_dist_frac * math.hypot(img_w, img_h)
        cents = [((b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0) for b in boxes]
        pairs = []
        for di, (cx, cy) in enumerate(cents):
            for tid, t in self.tracks.items():
                d = math.hypot(cx - t['cx'], cy - t['cy'])
                if d <= gate:
                    pairs.append((d, di, tid))
        pairs.sort()
        assigned, used = {}, set()
        for d, di, tid in pairs:
            if di in assigned or tid in used:
                continue
            assigned[di] = tid; used.add(tid)
        out = []
        for di, (cx, cy) in enumerate(cents):
            tid = assigned.get(di)
            if tid is None:
                tid = self.next_id; self.next_id += 1
            b = boxes[di]
            self.tracks[tid] = {'cx': cx, 'cy': cy, 'miss': 0}
            out.append(tid)
        for tid in list(self.tracks):
            if tid not in out:
                self.tracks[tid]['miss'] += 1
                if self.tracks[tid]['miss'] > self.lost_frames:
                    del self.tracks[tid]
        return out
'''
s = s.replace("class PerceptionNode(Node):", TRACKER + "\n\nclass PerceptionNode(Node):", 1)
s = s.replace("""        res = self.pose_model.track(frame, persist=True, conf=self.pose_conf,
                                    classes=[0], verbose=False)[0]""",
"""        if not hasattr(self, '_ctrk'):
            self._ctrk = CentroidTracker()
        res = self.pose_model.predict(frame, conf=self.pose_conf,
                                      classes=[0], verbose=False)[0]""")
s = s.replace("""        ids = (boxes.id.int().cpu().tolist() if boxes.id is not None
               else list(range(len(boxes))))
        xyxy = boxes.xyxy.cpu().numpy()""",
"""        xyxy = boxes.xyxy.cpu().numpy()
        ids = self._ctrk.update([tuple(b) for b in xyxy],
                                frame.shape[1], frame.shape[0])""")

# ── 디버그 화면 1.5배 확대 제거 ───────────────────────────────────
m = re.search(r"^\s*debug_img = cv2\.resize\(debug_img.*?\n(\s*interpolation=.*?\)\n)?", s, re.M | re.S)
if m:
    s = s[:m.start()] + "            # [개선] 1.5배 확대 제거\n" + s[m.end():]
open(DST, "w").write(s)
print("pn_FINAL.py 생성")
