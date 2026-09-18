"""예약 66 판정기 회귀 — 넷 중 하나만 무너져도 FAIL 이어야 한다 (2026-09-18)."""
import importlib.util, pathlib, sys

_p = pathlib.Path(__file__).with_name('usb_cam_lidar_test.py')
_spec = importlib.util.spec_from_file_location('usb_cam_lidar_test', _p)
m = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(m)

GOOD = dict(duration=200.0, scan_max_gap=0.19, usb_events=[], moving_ratio=0.8, scan_count=2000)


def _v(**patch):
    return m.verdict(**{**GOOD, **patch})


def test_all_good_passes():
    ok, why = _v()
    assert ok and why == []


def test_short_run_fails():
    ok, why = _v(duration=179.9)
    assert not ok and any('180' in w for w in why)


def test_duration_boundary_passes():
    assert _v(duration=180.0)[0]


def test_scan_gap_at_limit_fails():
    assert not _v(scan_max_gap=1.0)[0]           # 경계 = drive_watch SCAN_TIMEOUT 과 같은 1.0s
    assert _v(scan_max_gap=0.99)[0]


def test_one_usb_disconnect_fails():
    ok, why = _v(usb_events=['usb 2-2: USB disconnect, device number 7'])
    assert not ok and 'USB 이상 1건' in why[0]


def test_stationary_run_is_not_evidence():
    ok, why = _v(moving_ratio=0.59)
    assert not ok and '정지 시험' in why[0]
    assert _v(moving_ratio=0.60)[0]


def test_no_scan_at_all_fails_even_if_everything_else_looks_fine():
    ok, why = _v(scan_count=0, scan_max_gap=0.0)
    assert not ok and '/scan' in why[0]


def test_usb_regex_catches_the_0822_signatures():
    for line in ['kernel: cp210x ttyUSB0: usb_serial_generic_read_bulk_callback - urb stopped: -32',
                 'kernel: usb 2-2: USB disconnect, device number 7',
                 'kernel: usb 2-2: reset high-speed USB device number 8 using tegra-xusb',
                 'kernel: usb usb2-port2: over-current condition']:
        assert m.USB_BAD.search(line), line
    assert not m.USB_BAD.search('kernel: usb 1-2.3: new full-speed USB device number 5 using tegra-xusb')
