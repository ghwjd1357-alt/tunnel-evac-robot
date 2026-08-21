# -*- coding: utf-8 -*-
"""runbook_check 부정 회귀 (08-21, Codex §82.10).

한 글자 변경 · 파일 누락 · 관문 문장 삭제가 각각 잡히는지, 그리고 정상 사본이
통과하는지 본다. 실제 `~/Desktop` 을 건드리지 않도록 임시 manifest 로 돌린다.
"""

import json
import os
import subprocess
import sys
import tempfile

TOOL = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'runbook_check.py')
MAN = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'runbook_manifest.json')


def run(manifest_path):
    """runbook_check 를 임시 manifest 로 돌린다 (MANIFEST 상수를 주입)."""
    code = (
        'import runpy, sys, os;'
        f'sys.path.insert(0, {os.path.dirname(TOOL)!r});'
        'import runbook_check as R;'
        f'R.MANIFEST = {manifest_path!r};'
        'sys.argv = ["runbook_check.py"];'
        'sys.exit(R.main())'
    )
    return subprocess.run([sys.executable, '-c', code],
                          capture_output=True, text=True)


def make(tmp, text, gates, name='rb.md'):
    """임시 런북 + 그것을 가리키는 manifest 를 만든다. 반환 (manifest 경로, 런북 경로)."""
    home = os.path.expanduser('~')
    d = tempfile.mkdtemp(dir=tmp)
    rb = os.path.join(d, name)
    with open(rb, 'w', encoding='utf-8') as f:
        f.write(text)
    import hashlib
    h = hashlib.sha256(text.encode('utf-8')).hexdigest()
    man = {'_why': 't', 'runbooks': [{
        'path': os.path.relpath(rb, home), 'role': 't',
        'sha256': h, 'bytes': len(text.encode('utf-8')),
        'required_gates': gates}]}
    mp = os.path.join(d, 'man.json')
    with open(mp, 'w', encoding='utf-8') as f:
        json.dump(man, f, ensure_ascii=False)
    return mp, rb


BODY = '# 런북\n\n무장 확인 (z: 2.0)\n어댑터 재무장\n'
GATES = ['무장 확인 (z: 2.0)', '어댑터 재무장']


def test_unchanged_copy_passes(tmp_path):
    mp, _ = make(str(tmp_path), BODY, GATES)
    r = run(mp)
    assert r.returncode == 0, r.stdout + r.stderr


def test_one_character_change_is_caught(tmp_path):
    """🔴 한 글자만 바꿔도 지문이 어긋나야 한다."""
    mp, rb = make(str(tmp_path), BODY, GATES)
    with open(rb, 'a', encoding='utf-8') as f:
        f.write('.')
    r = run(mp)
    assert r.returncode == 1
    assert '내용이 바뀌었다' in r.stdout, r.stdout


def test_missing_file_is_caught(tmp_path):
    mp, rb = make(str(tmp_path), BODY, GATES)
    os.unlink(rb)
    r = run(mp)
    assert r.returncode == 1 and '없음' in r.stdout, r.stdout


def test_deleted_gate_sentence_is_caught(tmp_path):
    """🔴 지문보다 중요한 것 — 관문 문장이 사라지면 의미가 사라진 것이다."""
    mp, rb = make(str(tmp_path), BODY, GATES)
    with open(rb, 'w', encoding='utf-8') as f:
        f.write(BODY.replace('어댑터 재무장', ''))
    r = run(mp)
    assert r.returncode == 1
    assert '필수 관문 문장 누락' in r.stdout, r.stdout


def test_gate_check_wins_over_hash_message(tmp_path):
    """관문이 없으면 '바뀌었다' 가 아니라 '누락' 으로 보고해야 한다 — 더 센 신고다."""
    mp, rb = make(str(tmp_path), BODY, GATES)
    with open(rb, 'w', encoding='utf-8') as f:
        f.write('# 완전히 다른 문서\n')
    r = run(mp)
    assert '필수 관문 문장 누락' in r.stdout, r.stdout


def test_real_manifest_has_no_secrets():
    """🔴 공개 저장소다 — manifest 에 비밀번호·키가 들어가면 안 된다."""
    sys.path.insert(0, os.path.dirname(TOOL))
    import runbook_check as R
    man = json.load(open(MAN, encoding='utf-8'))
    assert R.check_manifest_hygiene(man) == []


def test_real_manifest_covers_the_shoot_runbook():
    """촬영 런북은 반드시 anchor 돼 있어야 한다 (§82.10 의 대상)."""
    man = json.load(open(MAN, encoding='utf-8'))
    paths = [e['path'] for e in man['runbooks']]
    assert any('0822_촬영_명령묶음' in p for p in paths), paths
    shoot = next(e for e in man['runbooks'] if '0822_촬영_명령묶음' in e['path'])
    joined = ' '.join(shoot['required_gates']).lower()
    assert 'rearm' in joined, shoot['required_gates']
    assert 'yaw' in joined, shoot['required_gates']
    # §83.4 — 전달 방식까지 관문이다. --once 는 discovery 전에 유실된다.
    assert '--times' in joined, shoot['required_gates']
