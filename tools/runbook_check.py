#!/usr/bin/env python3
"""외부 런북 지문 대조 — 커밋이 근거로 든 문서가 그때 그 문서인가 (08-21 §82.10).

사용:
    python3 tools/runbook_check.py            # 대조 (rc=1 이면 어긋남)
    python3 tools/runbook_check.py --update   # 지문 갱신 (의도적으로만)

왜 이 도구가 있나
-----------------
커밋 본문이 `~/Desktop/0822_촬영_명령묶음.md` 같은 **저장소 밖 문서**를 완료 근거로
인용한다. 그런데 그 파일은 `git show` 의 blob 이 아니다. 그래서 다음 세션이나
검토자가 *"그 해시 시점의 문서가 지금 이 문서와 같은가"* 를 **독립 대조할
앵커가 없다.** 실제로 08-21 검토가 이 자리를 지적했다(§82.10):

    "현재 촬영 명령 묶음은 여러 take 사이 mission reset 을 적으면서
     adapter 재무장은 적지 않아 §82.5 를 드러냈다"

즉 런북과 코드가 갈렸는데 저장소만 봐서는 알 수 없었다.

🔴 본문을 저장소에 넣지 않는 이유
   `tunnel-evac-robot` 은 **공개** 저장소다(`AGENTS.md §5`). 런북에는 계정·경로·
   현장 절차가 섞여 있다. 그래서 **본문 대신 지문과 필수 관문 문장만** 둔다.
   문서 백업은 별도 **비공개** 저장소 `tunnel-evac-docs` 가 맡는다.

무엇을 보는가
    ① 파일이 있는가
    ② sha256 이 manifest 와 같은가 (다르면 "바뀌었다" — 실패가 아니라 신고)
    ③ **필수 관문 문장**이 들어 있는가 — 이게 진짜 계약이다.
       해시는 오타 하나에도 바뀌지만, 관문 문장은 *의미*가 사라졌을 때만 사라진다.
"""

import argparse
import hashlib
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
MANIFEST = os.path.join(HERE, 'runbook_manifest.json')

# 🔴 비밀번호·개인정보 냄새가 나는 문자열은 manifest 에 넣지 않는다(§82.10 요구).
FORBIDDEN = ('password', 'passwd', '비밀번호', 'ssh-rsa', 'BEGIN RSA',
             'api_key', 'apikey', 'token=')


def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(65536), b''):
            h.update(chunk)
    return h.hexdigest()


def load():
    with open(MANIFEST, encoding='utf-8') as f:
        return json.load(f)


def check_manifest_hygiene(man):
    """manifest 자체에 개인정보가 섞이지 않았는가."""
    blob = json.dumps(man, ensure_ascii=False).lower()
    return [w for w in FORBIDDEN if w.lower() in blob]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--update', action='store_true',
                    help='지문을 현재 파일로 갱신한다 (의도적으로만)')
    a = ap.parse_args()

    man = load()
    home = os.path.expanduser('~')
    bad = 0

    dirty = check_manifest_hygiene(man)
    if dirty:
        print(f'🔴 manifest 에 넣으면 안 되는 문자열: {dirty}')
        return 1

    for e in man['runbooks']:
        path = os.path.join(home, e['path'])
        name = e['path']
        if not os.path.isfile(path):
            print(f'🔴 없음  {name}')
            bad += 1
            continue
        text = open(path, encoding='utf-8').read()
        missing = [g for g in e.get('required_gates', []) if g not in text]
        digest = sha256(path)
        size = os.path.getsize(path)

        if a.update:
            e['sha256'], e['bytes'] = digest, size
            if missing:
                print(f'🔴 {name} — 필수 관문 문장이 없다: {missing}')
                print('   🔴 지문만 갱신하고 넘어가지 않는다. 문서를 먼저 고칠 것.')
                bad += 1
            else:
                print(f'✏  {name} — 지문 갱신 ({size:,} bytes)')
            continue

        if missing:
            print(f'🔴 {name} — 필수 관문 문장 누락 {len(missing)}건:')
            for m in missing:
                print(f'     · "{m}"')
            bad += 1
        elif digest != e['sha256']:
            print(f'🔶 {name} — 내용이 바뀌었다 (관문 문장은 살아 있다)')
            print(f'     manifest {e["sha256"][:16]}…  현재 {digest[:16]}…')
            print('     의도한 변경이면: python3 tools/runbook_check.py --update')
            bad += 1
        else:
            print(f'🟢 {name} — 지문·관문 일치 ({size:,} bytes)')

    if a.update:
        with open(MANIFEST, 'w', encoding='utf-8') as f:
            json.dump(man, f, ensure_ascii=False, indent=2)
            f.write('\n')
        print(f'\nmanifest 저장 — {MANIFEST}')
        return 1 if bad else 0

    print()
    if bad:
        print(f'🔴 {bad} 건 어긋남 — 커밋이 인용한 런북과 현재 문서가 같지 않다.')
        return 1
    print('🟢 인용된 런북 전량이 커밋 시점과 같다.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
