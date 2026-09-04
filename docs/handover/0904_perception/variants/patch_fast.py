#!/usr/bin/env python3
"""빠른 개선 A+B 를 pn_FAST.py 에 적용. 정확도에 영향 없는 것만."""
import shutil
SRC = "/tmp/perftest/pn_TRT.py"
DST = "/tmp/perftest/pn_FAST.py"
shutil.copy(SRC, DST)
s = open(DST).read()

# ── A : 디버그 화면 1.5배 확대 제거 (640x480 그대로 발행) ─────────
import re
m = re.search(r"^\s*debug_img = cv2\.resize\(debug_img.*?\n(\s*interpolation=.*?\)\n)?", s, re.M | re.S)
assert m, "확대 줄 못 찾음"
s = s[:m.start()] + "            # [빠른개선 A] 1.5배 확대 제거 — 뷰어에서 확대하면 된다\n" + s[m.end():]

# ── B : fire 전처리 1패스 (미리 계산한 scale/bias 로 곱셈 한 번) ──
old = """        x = img.astype(_np.float32) / 255.0
        x = (x - _np.array([0.485, 0.456, 0.406], _np.float32)) / \\
            _np.array([0.229, 0.224, 0.225], _np.float32)
        x = _np.ascontiguousarray(x.transpose(2, 0, 1)[None])"""
assert old in s, "전처리부 못 찾음"
new = """        # [빠른개선 B] 4번 훑던 것을 곱셈 1번으로. 결과값은 수학적으로 동일하다.
        if not hasattr(self, '_pre_s'):
            _m = _np.array([0.485, 0.456, 0.406], _np.float32).reshape(3, 1, 1)
            _sd = _np.array([0.229, 0.224, 0.225], _np.float32).reshape(3, 1, 1)
            self._pre_s = (1.0 / 255.0) / _sd
            self._pre_b = -_m / _sd
        x = img.transpose(2, 0, 1).astype(_np.float32)
        x = _np.ascontiguousarray((x * self._pre_s + self._pre_b)[None])"""
s = s.replace(old, new)
open(DST, "w").write(s)
print("A+B 적용 완료")
