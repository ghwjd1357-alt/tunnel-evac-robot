# -*- coding: utf-8 -*-
"""apply_measurements 전용 fault-injection 회귀 (08-21, Codex §82.8·§83.8).

[무엇을 잡나]
  ① §82.8 — 구판은 **쓰고 나서 검증**했다. `--normal-speed nan` 이면 원본이 nan 이
     되고, 이어서 한 번 더 실행하면 `.bak` 까지 nan 으로 덮였다. 도구가 안내하는
     복구 명령(`cp .bak`)이 오염본을 되돌리는 상태였다.
  ② §83.8 — 그걸 고친 뒤에도 **mode 가 0644 → 0600 으로 바뀌었다**(`mkstemp` 기본).
     같은 사용자면 티가 안 나지만 다른 service user 가 읽는 배포에서는 정상 적용이
     권한 실패를 만든다.

⚠ 전원 단절 자체는 시험할 수 없다. 여기서는 **replace 직전 예외**로 대신하고,
  디렉터리 fsync 는 호출 여부로만 본다 — 그 이상은 이 층에서 증명할 수 없다.
"""

import hashlib
import os
import shutil
import stat
import subprocess
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TOOL = os.path.join(ROOT, 'tools', 'apply_measurements.py')
SRC = os.path.join(ROOT, 'src', 'mission_manager', 'config', 'waypoints_real_H.yaml')

pytest.importorskip('yaml')


def sha(path):
    return hashlib.sha256(open(path, 'rb').read()).hexdigest()


def run(target, *args):
    return subprocess.run([sys.executable, TOOL, '--file', target, *args],
                          capture_output=True, text=True, cwd=ROOT)


@pytest.fixture
def yml(tmp_path):
    t = str(tmp_path / 'w.yaml')
    shutil.copy(SRC, t)
    os.chmod(t, 0o644)
    return t


BAD = [
    ('--normal-speed', 'nan'), ('--normal-speed', 'inf'),
    # ⚠ 음수는 `--flag=값` 으로 준다 — argparse 가 `-inf` 를 옵션으로 읽는다.
    ('--normal-speed=-inf', None), ('--normal-speed=-0.1', None),
    ('--guide-speed', 'nan'), ('--guide-speed', '0'),
    ('--cluster-max-width=-1', None), ('--detect-range', 'inf'),
    ('--min-points', '0'), ('--min-points=-2', None),
    ('--low-west', '1.0'), ('--low-west', 'a,b'), ('--low-west', 'nan,-10.6'),
    ('--normal-speed', '0.5'),          # 검증 실패 (상한 0.12)
]


@pytest.mark.parametrize('flag,val', BAD)
def test_bad_input_never_touches_original_or_backup(yml, flag, val):
    """🔴 §82.8 재현본 — 어떤 거부 경로에서도 바이트가 안 변해야 한다."""
    bak = yml + '.bak'
    shutil.copy(yml, bak)
    h0, b0 = sha(yml), sha(bak)
    r = run(yml, flag) if val is None else run(yml, flag, val)
    assert r.returncode == 1, r.stdout
    assert sha(yml) == h0, f'원본이 바뀌었다 ({flag} {val})'
    assert sha(bak) == b0, f'백업이 바뀌었다 ({flag} {val})'


def test_two_consecutive_failures_keep_last_good(yml):
    """실패를 두 번 이어서 해도 last-good 이 살아 있어야 한다 (구판이 여기서 죽었다)."""
    bak = yml + '.bak'
    shutil.copy(yml, bak)
    g = sha(bak)
    run(yml, '--normal-speed', 'nan')
    run(yml, '--normal-speed', 'inf')
    assert sha(bak) == g
    assert run(yml, '--guide-speed', '0.09').returncode == 0


def test_mode_is_preserved(yml):
    """🔴 §83.8 재현본 — 0644 가 0600 이 되면 다른 사용자가 못 읽는다."""
    os.chmod(yml, 0o644)
    assert run(yml, '--guide-speed', '0.09').returncode == 0
    assert stat.S_IMODE(os.stat(yml).st_mode) == 0o644
    assert stat.S_IMODE(os.stat(yml + '.bak').st_mode) == 0o644


def test_mode_640_is_preserved_too(yml):
    os.chmod(yml, 0o640)
    assert run(yml, '--guide-speed', '0.09').returncode == 0
    assert stat.S_IMODE(os.stat(yml).st_mode) == 0o640


def test_no_temp_files_left_behind(yml):
    """실패 경로가 `.apply_*.tmp` 를 남기면 다음 사람이 그걸 정본으로 오해한다."""
    run(yml, '--normal-speed', 'nan')
    d = os.path.dirname(yml)
    assert [f for f in os.listdir(d) if f.startswith('.apply_')] == []


def test_exception_before_replace_leaves_original(yml, monkeypatch):
    """replace 직전 예외 — 원본이 온전하고 임시파일도 안 남아야 한다."""
    sys.path.insert(0, os.path.join(ROOT, 'tools'))
    import apply_measurements as A
    h0 = sha(yml)
    real = os.replace

    def boom(a, b):
        raise OSError('injected before replace')

    monkeypatch.setattr(os, 'replace', boom)
    with pytest.raises(OSError):
        A.atomic_write(yml, 'ruined', 0o644)
    monkeypatch.setattr(os, 'replace', real)
    assert sha(yml) == h0
    d = os.path.dirname(yml)
    assert [f for f in os.listdir(d) if f.startswith('.apply_')] == []


def test_directory_is_fsynced(yml, monkeypatch):
    """§83.8 — rename 의 디렉터리 엔트리까지 durable 해야 주장이 성립한다."""
    sys.path.insert(0, os.path.join(ROOT, 'tools'))
    import apply_measurements as A
    seen = []
    real = os.fsync
    monkeypatch.setattr(os, 'fsync', lambda fd: (seen.append(fd), real(fd))[1])
    A.atomic_write(yml, open(yml, encoding='utf-8').read(), 0o644)
    assert len(seen) >= 2, '파일 fsync 만 하고 디렉터리는 안 했다'


def test_good_change_round_trips_values_and_keeps_comments(yml):
    """🟢 정상 6키 변경은 주석을 보존하고 값이 왕복해야 한다."""
    import yaml as Y
    before = open(yml, encoding='utf-8').read()
    assert run(yml, '--guide-speed', '0.08', '--normal-speed', '0.09',
               '--cluster-max-width', '0.62', '--detect-range', '1.80',
               '--min-points', '4', '--low-west', '0.42,-10.61').returncode == 0
    after = open(yml, encoding='utf-8').read()
    assert '# 유도 저속' in after and '평시' in after, '주석이 날아갔다'
    assert run(yml, '--guide-speed', '0.10', '--normal-speed', '0.10',
               '--cluster-max-width', '0.80', '--detect-range', '1.50',
               '--min-points', '3', '--low-west', '0.50,-10.65').returncode == 0
    assert Y.safe_load(open(yml, encoding='utf-8')) == Y.safe_load(before)
