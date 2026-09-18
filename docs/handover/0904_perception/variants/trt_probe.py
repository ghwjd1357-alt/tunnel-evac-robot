#!/usr/bin/env python3
"""엔진이 로드되고 실제로 추론이 도는지 + 몇 ms 인지. 노드는 안 건드린다."""
import time, sys, numpy as np

ENGINE = sys.argv[1] if len(sys.argv) > 1 else "/home/hanhan/fire_trt_export/rfdetr-small.engine"

from polygraphy.backend.trt import EngineFromBytes, TrtRunner
from polygraphy.backend.common import BytesFromPath

print("엔진:", ENGINE)
engine = EngineFromBytes(BytesFromPath(ENGINE))

with TrtRunner(engine) as runner:
    meta = runner.get_input_metadata()
    print("\n=== 입력 ===")
    for name, (dtype, shape) in meta.items():
        print(f"  {name:20} {dtype}  {shape}")

    feed = {}
    for name, (dtype, shape) in meta.items():
        s = [1 if (isinstance(d, str) or d < 0) else d for d in shape]
        feed[name] = np.random.rand(*s).astype(dtype)

    out = runner.infer(feed)                      # 워밍업
    print("\n=== 출력 ===")
    for k, v in out.items():
        print(f"  {k:20} {v.dtype}  {v.shape}")

    N = 30
    ts = []
    for _ in range(N):
        t0 = time.perf_counter()
        runner.infer(feed)
        ts.append((time.perf_counter() - t0) * 1000)
    ts.sort()
    print(f"\n=== 추론 시간 ({N}회) ===")
    print(f"  중앙값 {ts[N//2]:6.1f} ms")
    print(f"  최소   {ts[0]:6.1f} ms   최대 {ts[-1]:6.1f} ms")
    print(f"\n  현재 파이토치 FP16(JIT) fire = 약 120 ms")
    print(f"  → 배율 약 {120/ts[N//2]:.1f}x")
