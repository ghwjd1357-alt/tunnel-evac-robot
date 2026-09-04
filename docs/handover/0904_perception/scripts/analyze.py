#!/usr/bin/env python3
"""추적 결과 분석 — track_id 연속성과 판정 분포."""
import json, sys
from collections import Counter

path = sys.argv[1]
L = [json.loads(x) for x in open(path)]
ids = set()
for r in L:
    ids.update(r['ids'])
c = Counter(d[0] for r in L for d in r['det'])
withp = sum(1 for r in L if r['ids'])

print(f"  프레임 {len(L)}")
print(f"  사람 잡힌 프레임 {withp}/{len(L)}  ({withp/max(1,len(L))*100:.0f}%)")
print(f"  고유 track_id {len(ids)}: {sorted(ids)[:15]}")
print(f"  판정 {dict(c)}")

# ID 연속성 — 가장 오래 유지된 id 와 조각남 정도
runs = {}
for r in L:
    for i in r['ids']:
        runs[i] = runs.get(i, 0) + 1
top = sorted(runs.items(), key=lambda kv: -kv[1])[:6]
print(f"  id별 등장 프레임수 {top}")
if runs:
    print(f"  최장 유지 {max(runs.values())} 프레임 · 조각 수 {len(runs)}")
