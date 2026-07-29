#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
런치 파일들이 공유하는 '조건 기동' 배선 도구.

[왜 런치 파일이 아니라 여기 있나]
  `launch/` 안의 파일들은 서로 import 할 수 없다. ros2 launch 는 그 파일을 모듈이
  아니라 스크립트로 읽어 들이고, share 폴더는 파이썬 경로에 없기 때문이다.
  반면 이 파일은 패키지의 정식 파이썬 모듈이라 `from tunnel_bringup.launch_util import ...`
  로 어느 런치에서든 쓸 수 있다 (setup.bash 가 경로를 잡아 준다).

[하는 일 두 가지]
  make_gate()  : readiness_gate 노드 하나를 파라미터와 함께 만든다.
  when_ready() : "그 게이트가 성공하면 다음 단계를, 실패하면 런치 전체 종료"를 배선한다.

★ 이 두 함수가 실차 런치에서 **고정 시간 지연을 대체한다.** 시뮬은 "8초 뒤 Nav2,
  12초 뒤 미션"처럼 시계로 순서를 맞추지만, 실차는 부팅·USB 인식·micro-ROS 연결
  타이밍이 매번 달라 그 숫자가 성립하지 않는다. 시간 대신 **관측된 조건**으로 넘어간다.
"""

from launch.actions import LogInfo, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def make_gate(label, timeout, topics=(), transient_local=(), tf=(), tf_fresh=0.0,
              lifecycle=(), actions=()):
    """
    조건 기동 게이트 노드 하나를 만든다.

    label      : 로그 식별자 겸 노드 이름 접미사 (게이트마다 달라야 한다)
    timeout    : 이 시간 안에 조건을 못 채우면 게이트가 실패로 종료한다 (초)
    topics     : 흐르고 있어야 할 토픽 목록
    transient_local : 그중 래치 토픽(/map 등 — 수신 1건이면 충족)
    tf         : "부모:자식" TF 목록
    tf_fresh   : 0 이면 연결만, >0 이면 그 초 안에 갱신됐는지까지 본다
                 (URDF 고정 joint 같은 정적 TF 에는 반드시 0 — 한 번만 발행된다)
    lifecycle  : ACTIVE 여야 할 lifecycle 노드 이름 목록
    actions    : 서버가 떠 있어야 할 액션 이름 목록

    빈 리스트를 그대로 넘기면 rclpy 가 파라미터 타입을 정하지 못하므로 [''] 를 넣고,
    readiness_gate 쪽에서 빈 문자열을 걷어낸다.
    """
    return Node(
        package='tunnel_bringup',
        executable='readiness_gate',
        name=f'readiness_gate_{label}',
        output='screen',
        parameters=[{
            # ★ 실차는 실제 시계로 판정한다 (시뮬 전용 시계를 쓰지 않는다).
            'use_sim_time': False,
            'label': label,
            'require_topics': list(topics) or [''],
            'transient_local_topics': list(transient_local) or [''],
            'require_tf': list(tf) or [''],
            'tf_fresh_sec': float(tf_fresh),
            'require_lifecycle_active': list(lifecycle) or [''],
            'require_actions': list(actions) or [''],
            'timeout_sec': float(timeout),
        }],
    )


def when_ready(gate, next_actions, what):
    """
    게이트가 0 으로 끝나면 다음 단계를 띄우고, 아니면 런치 전체를 내린다.

    ★ fail-closed 인 이유: 조건을 못 채웠다는 것은 센서·구동부·Nav2 중 무언가가
      죽었다는 뜻이다. 그 상태로 다음 단계를 띄우면 '반쪽 스택'이 만들어지고,
      최악의 경우 라이다 없이(=장애물을 못 보고) 주행 명령을 받는 구성이 된다.
      그래서 다음 단계를 건너뛰는 것이 아니라 **런치 전체를 종료**한다.
    """
    def on_exit(event, context):
        if event.returncode == 0:
            return next_actions
        return [
            LogInfo(msg=f'❌ 준비 조건 미충족 — {what} 를 기동하지 않고 런치를 종료한다 '
                        f'(반쪽 스택 주행 금지). 위 게이트 로그의 "미충족" 목록을 볼 것.'),
            Shutdown(reason=f'readiness gate failed before {what}'),
        ]
    return RegisterEventHandler(OnProcessExit(target_action=gate, on_exit=on_exit))
