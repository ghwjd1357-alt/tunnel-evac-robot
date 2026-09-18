#!/usr/bin/env python3
"""
TRT 실험 패치 — /tmp/perftest/pn_TRT.py 만 만든다. 원본은 안 건드린다.
  A' : fire 를 TensorRT 엔진으로 교체 (파이토치 모델은 아예 안 올린다)
  C  : fire 격프레임 (FIRE_EVERY, 기본 2)
  B  : 적용 안 함 — 디버그 화면은 촬영 대상이라 켜 둔다
🔴 후처리는 이 실험에서 새로 쓴 것이다. **정확도 미검증** — 속도 측정 전용.
"""
import shutil, sys

SRC = "/home/hanhan/percep_ws/install/vision_pipeline/lib/python3.10/site-packages/vision_pipeline/perception_node.py"
DST = "/tmp/perftest/pn_TRT.py"
shutil.copy(SRC, DST)
s = open(DST).read()

# ── 1. fire 모델 로딩을 TRT 러너로 교체 ───────────────────────────
old_load = """        from rfdetr import RFDETRSmall
        self.fire_model = RFDETRSmall(pretrain_weights=fire_path,
                                      num_classes=int(g('fire_num_classes')))"""
assert old_load in s, "fire 로딩부 못 찾음"
new_load = """        # [실험] TensorRT 엔진으로 교체 — 파이토치 모델은 올리지 않는다
        import os as _os
        from polygraphy.backend.trt import EngineFromBytes, TrtRunner
        from polygraphy.backend.common import BytesFromPath
        _eng = _os.environ.get('FIRE_ENGINE',
                               '/home/hanhan/fire_trt_export/rfdetr-small.engine')
        self.get_logger().info('Loading TRT engine %s ...' % _eng)
        self._trt = TrtRunner(EngineFromBytes(BytesFromPath(_eng)))
        self._trt.activate()
        self._trt_res = 512
        self.fire_model = None"""
s = s.replace(old_load, new_load)

# ── 2. _infer_fire 를 TRT 추론으로 교체 ───────────────────────────
old_sig = "    def _infer_fire(self, frame) -> List[Tuple[str, float, Tuple]]:"
i = s.index(old_sig)
j = s.index("    def _infer_pose", i)
new_fire = '''    def _infer_fire(self, frame) -> List[Tuple[str, float, Tuple]]:
        """[실험] TensorRT 엔진 추론. 🔴 후처리 정확도 미검증 — 속도 측정용."""
        import numpy as _np
        H, W = frame.shape[:2]
        R = self._trt_res
        img = cv2.resize(frame, (R, R), interpolation=cv2.INTER_LINEAR)
        x = img.astype(_np.float32) / 255.0
        x = (x - _np.array([0.485, 0.456, 0.406], _np.float32)) / \\
            _np.array([0.229, 0.224, 0.225], _np.float32)
        x = _np.ascontiguousarray(x.transpose(2, 0, 1)[None])

        o = self._trt.infer({'input': x})
        dets = o['dets'][0]            # (300, 4) cxcywh 정규화
        logits = o['labels'][0]        # (300, 4) 클래스 로짓

        prob = 1.0 / (1.0 + _np.exp(-logits))
        cls = prob.argmax(1)
        conf = prob.max(1)
        keep = conf >= self.fire_conf

        out = []
        for k in _np.nonzero(keep)[0]:
            name = RFDETR_CLASS_NAMES.get(int(cls[k]))
            if name is None:                       # 0=car (N4)
                continue
            cx, cy, bw, bh = dets[k]
            x1 = (cx - bw / 2) * W; y1 = (cy - bh / 2) * H
            x2 = (cx + bw / 2) * W; y2 = (cy + bh / 2) * H
            out.append((name, float(conf[k]), (float(x1), float(y1),
                                               float(x2), float(y2))))
        return out

'''
s = s[:i] + new_fire + s[j:]

# ── 3. C : fire 격프레임 ──────────────────────────────────────────
L = s.split("\n")
st = next(k for k, x in enumerate(L) if x.strip() == "try:" and "_infer_fire" in L[k + 1])
assert L[st + 4].strip() == "return", L[st + 4]
C = [
    "        # [실험 C] fire 격프레임",
    "        self._fc = getattr(self, '_fc', 0) + 1",
    "        _every = int(os.environ.get('FIRE_EVERY', '2'))",
    "        if (self._fc % _every == 1) or (not hasattr(self, '_fire_cache')):",
    "            try:",
    "                fire_dets = self._infer_fire(color_np)",
    "                self._fire_cache = fire_dets",
    "            except Exception as e:",
    "                self._fail_frame('fire', str(e))",
    "                return",
    "        else:",
    "            fire_dets = self._fire_cache",
]
L = L[:st] + C + L[st + 5:]
open(DST, "w").write("\n".join(L))
print("pn_TRT.py 생성 완료")
