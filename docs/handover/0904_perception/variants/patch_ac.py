#!/usr/bin/env python3
"""ABC 실험 패치 — 원본은 안 건드리고 /tmp/perftest/pn_ABC.py 만 만든다."""
import shutil, sys

SRC = "/home/hanhan/percep_ws/install/vision_pipeline/lib/python3.10/site-packages/vision_pipeline/perception_node.py"
DST = "/tmp/perftest/pn_AC.py"
shutil.copy(SRC, DST)
L = open(DST).read().split("\n")

# ── A : JIT trace 켜기 ────────────────────────────────────────────
for i, x in enumerate(L):
    if "optimize_for_inference(compile=False" in x:
        L[i] = x.replace("compile=False", "compile=True")
        break
else:
    sys.exit("A 실패: optimize_for_inference 못 찾음")

# ── C : fire 격프레임 ─────────────────────────────────────────────
start = next(i for i, x in enumerate(L)
             if x.strip() == "try:" and "_infer_fire" in L[i + 1])
end = start + 4                       # try / infer / except / fail / return
assert L[end].strip() == "return", L[end]

C = [
    "        # [실험 C] fire 격프레임 — 화재는 프레임 단위로 움직이지 않는다.",
    "        #   bbox 는 직전 결과를 재사용하고 depth 는 현재 프레임으로 다시 잰다.",
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
L = L[:start] + C + L[end + 1:]
s = "\n".join(L)

# B 는 적용하지 않는다 — 디버그 화면이 곧 촬영 대상이다

open(DST, "w").write(s)
print("ABC 생성 완료")
