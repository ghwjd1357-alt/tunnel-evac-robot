#!/usr/bin/env python3
"""어댑터↔미션 **교차 패키지 계약** 회귀 — 2026-08-22 (독립 검토 §88.4).

🔴 **한 패키지 안에서는 못 보는 결함이 있다.** 미션의 `scan_dwell_sec` 가 어댑터의
`person_confirm_sec_leave` 보다 짧으면, 훑기의 한 방향에서 `ok`·`none` 이 **확정될
수 없다** — 방향이 바뀌면 frame verdict 가 바뀌어 streak 가 재시작하기 때문이다.
그러면 360° 를 돌아도 `unknown` 만 모으고 끝난다.

두 값이 **다른 패키지에 있어서** 각자의 회귀는 이것을 못 본다. 여기서 본다.
"""
import glob
import os
import re
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ADAPTER = os.path.join(ROOT, 'src/perception_adapter/perception_adapter/adapter_node.py')
WP_GLOB = os.path.join(ROOT, 'src/mission_manager/config/waypoints*.yaml')


def adapter_default(name):
    src = open(ADAPTER, encoding='utf-8').read()
    m = re.search(rf"declare_parameter\('{name}',\s*([0-9.]+)\)", src)
    assert m, f'어댑터에서 {name} 기본값을 못 찾았다'
    return float(m.group(1))


def mission_configs():
    import yaml
    out = {}
    for f in sorted(glob.glob(WP_GLOB)):
        with open(f, encoding='utf-8') as fh:
            out[os.path.basename(f)] = yaml.safe_load(fh)
    return out


class PersonChainContractTest(unittest.TestCase):

    def test_c1_dwell_can_produce_a_leave_verdict(self):
        """🔴 dwell < leave 면 훑기가 `unknown` 만 모은다 — 360° 가 무의미해진다."""
        leave = adapter_default('person_confirm_sec_leave')
        for name, wp in mission_configs().items():
            dwell = float(wp.get('scan_dwell_sec', 0.0))
            self.assertGreater(
                dwell, leave,
                f'{name}: dwell {dwell} ≤ leave {leave} — 그 방향에서 ok·none 이 '
                f'확정될 수 없다')

    def test_c2_dwell_also_covers_the_fallen_verdict(self):
        """🔵 역회귀 — fallen 은 더 짧으므로 당연히 들어가야 한다."""
        fallen = adapter_default('person_confirm_sec_fallen')
        for name, wp in mission_configs().items():
            self.assertGreater(float(wp.get('scan_dwell_sec', 0.0)), fallen, name)

    def test_c3_the_mission_freshness_guard_is_looser_than_the_adapter_publish(self):
        """미션의 신선도 가드는 어댑터의 **10 Hz 상시 발행**을 전제한다.

        어댑터가 살아 있는 한 `/person_status` 는 0.1초마다 나간다. 가드가 그보다
        빡빡하면 정상 동작이 `stale` 로 읽힌다.
        """
        for name, wp in mission_configs().items():
            self.assertGreater(
                float(wp.get('person_status_timeout_sec', 0.0)), 0.5,
                f'{name}: 어댑터 10 Hz 발행보다 가드가 빡빡하다')

    def test_c4_leaving_stays_more_conservative_than_reporting(self):
        """🔴 비대칭이 두 패키지에 걸쳐 유지되는가 — 떠나는 판정이 더 신중해야 한다."""
        self.assertGreater(adapter_default('person_confirm_sec_leave'),
                           adapter_default('person_confirm_sec_fallen'))


if __name__ == '__main__':
    unittest.main()
