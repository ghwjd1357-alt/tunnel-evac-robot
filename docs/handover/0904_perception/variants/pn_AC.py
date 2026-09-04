#!/usr/bin/env python3
"""
perception_node.py — `/detections` 발행 노드 (단일 노드)

기존 `rf_detr_depth_publisher.py` 를 기반으로 `/detections` V1 계약
(2026-08-17 합의 + 08-18 실측 반영)을 적용한 통합 노드.

기존에서 유지한 것
    - message_filters ApproximateTimeSynchronizer (color+depth 동기)
    - RFDETRSmall 로드 방식, get_median_depth 정책 (C5)
    - header.stamp = 촬영시각 (A3), frame_id = color optical frame (A4)
    - mm -> m 변환 (A5), depth 실패 시 해당 탐지 제외 (A9-1)

이번에 바뀐 것
    N1-b  화재 + 사람자세를 한 노드에서 처리. 두 모델 모두 성공해야 발행
    N1    실패 시 미발행 + /diagnostics ERROR (빈 배열로 위장하지 않음)
    C2-b  Fire-Smoke 의 human 은 발행하지 않고 카운터로만 집계
    C2-c  사람은 person_fallen / person_ok / person_unknown 3분기
    N2    class_name 열거값 5개로 제한
    N5    모델 일치 카운터 3종 -> /diagnostics
    N7    unknown 기권 + valid>=4 투표
    N8    중력축 기준 (미회신이므로 폴백) + 각도 상한 방어
    A7    QoS RELIABLE / VOLATILE / KEEP_LAST 5
    좌표  position 을 optical frame 규약(x=우, y=하, z=전방)으로 채움
          기존 코드는 거리를 x 에 넣고 y,z=0 이었다 -> 역할 A 가 오독한다
"""

from __future__ import annotations

import array
import os
import time
from typing import List, Optional, Tuple

import torch

import message_filters
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo, Image, RegionOfInterest
from tunnel_interfaces.msg import Detection3D, Detection3DArray

from vision_pipeline.posture_judge import (ALLOWED_CLASS_NAMES,
                                           ModelAgreementCounters,
                                           PostureConfig, PostureJudge,
                                           is_allowed_class,
                                           normalize_class_name)

# RF-DETR 클래스 매핑. 0=car 는 계약에서 제외(N4), 2=human 은 미발행(C2-b)
RFDETR_CLASS_NAMES = {1: 'fire', 2: 'human', 3: 'smoke'}
FIRE_PUBLISH = {'fire', 'smoke'}          # 실제로 /detections 에 싣는 것

L_SHOULDER, R_SHOULDER, L_HIP, R_HIP = 5, 6, 11, 12


class InferenceFailure(RuntimeError):
    """추론 실패. 이 프레임은 발행하지 않는다 (N1)."""


class PerceptionNode(Node):

    def __init__(self):
        super().__init__('perception_node')
        # 최근 person bbox 기억 (fire 오탐 억제용 시간적 보강 — 08-28)
        # pose가 특정 자세(팔로 얼굴 가림 등)에서 간헐적으로 사람을 놓치는
        # 프레임이 있고, 그 틈에 fire 억제(겹침 비교)가 비교 대상 person이
        # 없어 무력화되는 문제를 막는다. 최근 0.5초 이내 person 위치를
        # 계속 들고 있다가 억제 판단에 사용한다.
        self._recent_person_bboxes = []
        self._recent_person_time = 0.0
        self._PERSON_MEMORY_SEC = 0.5

        pkg_dir = os.path.dirname(__file__)

        # -- 파라미터 ------------------------------------------------------
        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('info_topic', '/camera/color/camera_info')
        self.declare_parameter(
            'fire_model_path',
            os.path.join(pkg_dir, 'models', 'fire_smoke_v9_rfdetr_small.pt'))
        self.declare_parameter('fire_conf', 0.38)
        self.declare_parameter('fire_num_classes', 3)
        self.declare_parameter('pose_model_path', '/home/hanhan/yolo11n-pose_meta.engine')
        self.declare_parameter('pose_conf', 0.5)
        self.declare_parameter('min_conf', 0.25)        # C4 publisher 하한
        self.declare_parameter('use_gravity', False)    # N8: M2 회신 후 True
        self.declare_parameter('debug_pose', True)
        self.declare_parameter('depth_min_m', 0.3)
        self.declare_parameter('depth_max_m', 8.0)
        self.declare_parameter('depth_min_px', 30)
        self.declare_parameter('depth_min_valid_frac', 0.5)
        self.declare_parameter('sync_slop', 0.1)
        self.declare_parameter('sync_queue_size', 10)

        g = lambda n: self.get_parameter(n).value   # noqa: E731
        self.fire_conf = float(g('fire_conf'))
        self.pose_conf = float(g('pose_conf'))
        self.min_conf = float(g('min_conf'))
        self.use_gravity = bool(g('use_gravity'))
        self.debug_pose = bool(g('debug_pose'))
        self.depth_min_m = float(g('depth_min_m'))
        self.depth_max_m = float(g('depth_max_m'))
        self.depth_min_px = int(g('depth_min_px'))
        self.depth_min_frac = float(g('depth_min_valid_frac'))

        # -- 모델 (N1-b: 한 노드 안에 둘 다) --------------------------------
        fire_path = str(g('fire_model_path'))
        self.get_logger().info(f'Loading RF-DETR from {fire_path} ...')
        from rfdetr import RFDETRSmall
        self.fire_model = RFDETRSmall(pretrain_weights=fire_path,
                                      num_classes=int(g('fire_num_classes')))
        self.get_logger().info('Optimizing fire model for FP16 inference...')
        self.fire_model.optimize_for_inference(compile=True, dtype=torch.float16)

        pose_path = str(g('pose_model_path'))
        self.get_logger().info(f'Loading pose model from {pose_path} ...')
        from ultralytics import YOLO
        self.pose_model = YOLO(pose_path)

        self.judge = PostureJudge(PostureConfig())
        self.counters = ModelAgreementCounters()

        # -- 카메라 내부 파라미터 (좌표 역투영용) ---------------------------
        self.fx = self.fy = self.cx = self.cy = None
        self.create_subscription(CameraInfo, str(g('info_topic')),
                                 self._on_info, 5)

        # -- 발행 (A7) ------------------------------------------------------
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST,
                         depth=5)
        self.pub = self.create_publisher(Detection3DArray, 'detections', qos)
        self.debug_img_pub = self.create_publisher(Image, '/camera/debug_image', 1)
        self.diag_pub = self.create_publisher(DiagnosticArray,
                                              '/diagnostics', qos)

        # -- 구독 (color + depth 동기) --------------------------------------
        color_sub = message_filters.Subscriber(self, Image, str(g('color_topic')))
        depth_sub = message_filters.Subscriber(self, Image, str(g('depth_topic')))
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub],
            queue_size=int(g('sync_queue_size')),
            slop=float(g('sync_slop')))
        self.ts.registerCallback(self.image_callback)

        self.frames_published = 0
        self.frames_dropped = 0
        self.get_logger().info(
            'perception_node ready (fire+pose, gravity=%s)' % self.use_gravity)

        # -- Y1 워치독 (2026-08-19) ---------------------------------
        # depth 스트림이 안 올라온 채 기동하면 동기화 콜백이 한 번도
        # 불리지 않는다. 그 상태에서 노드는 살아 있고 node list 에도
        # 나오므로, 콜백 안에서만 도는 _publish_diag 는 침묵한다.
        # 침묵이 곧 고장인데 침묵은 아무도 보지 못한다.
        # 따라서 콜백과 독립적으로 도는 타이머가 필요하다.
        self.declare_parameter('watchdog_hz', 1.0)
        self.declare_parameter('watchdog_startup_grace_sec', 10.0)
        self.declare_parameter('watchdog_stale_sec', 2.0)

        self._wd_stale = float(g('watchdog_stale_sec'))
        self._wd_grace = float(g('watchdog_startup_grace_sec'))
        self._wd_start = time.time()
        self._wd_last_cb = 0.0          # 0.0 = 콜백 한 번도 안 불림
        self._wd_last_state = None
        self._color_topic = str(g('color_topic'))
        self._depth_topic = str(g('depth_topic'))
        self._wd_last_depth = 0.0    # depth 프레임 마지막 도착 시각
        self._wd_depth_count = 0
        # depth 를 직접 구독한다. count_publishers 는 '발행자 등록'만 보므로
        # 토픽은 있는데 프레임이 안 흐르는 상태(2026-08-20 3회 관측)를
        # 구분하지 못한다. 이미지는 건드리지 않고 도착 시각만 기록한다.
        self.create_subscription(
            Image, self._depth_topic, self._wd_depth_probe, 1)
        self.create_timer(1.0 / float(g('watchdog_hz')), self._watchdog)


    # ======================================================================
    def _on_info(self, msg: CameraInfo) -> None:
        """camera_info 는 한 번만 받으면 된다 (C1)."""
        if self.fx is None:
            self.fx, self.fy = float(msg.k[0]), float(msg.k[4])
            self.cx, self.cy = float(msg.k[2]), float(msg.k[5])
            self.get_logger().info(
                'camera_info: fx=%.1f fy=%.1f cx=%.1f cy=%.1f'
                % (self.fx, self.fy, self.cx, self.cy))

    # ======================================================================
    def image_callback(self, color_msg: Image, depth_msg: Image) -> None:
        self._wd_last_cb = time.time()
        """
        한 프레임의 전 과정.
        N1: 어느 단계든 실패하면 발행하지 않고 /diagnostics 에만 남긴다.
        """
        _t0 = time.time()
        try:
            color_np = np.frombuffer(color_msg.data, dtype=np.uint8).reshape(
                color_msg.height, color_msg.width, 3)
            depth_np = np.frombuffer(depth_msg.data, dtype=np.uint16).reshape(
                depth_msg.height, depth_msg.width)
            # 카메라가 물리적으로 180도 뒤집혀 장착된 상태 보정 (08-22)
            color_np = cv2.rotate(color_np, cv2.ROTATE_180)
            depth_np = cv2.rotate(depth_np, cv2.ROTATE_180)
        except Exception as e:                                  # noqa: BLE001
            self._fail_frame('decode', str(e))
            return
        _t1 = time.time()

        # ---- 두 모델 추론 (N1) ------------------------------------------
        # [실험 C] fire 격프레임 — 화재는 프레임 단위로 움직이지 않는다.
        #   bbox 는 직전 결과를 재사용하고 depth 는 현재 프레임으로 다시 잰다.
        self._fc = getattr(self, '_fc', 0) + 1
        _every = int(os.environ.get('FIRE_EVERY', '2'))
        if (self._fc % _every == 1) or (not hasattr(self, '_fire_cache')):
            try:
                fire_dets = self._infer_fire(color_np)
                self._fire_cache = fire_dets
            except Exception as e:
                self._fail_frame('fire', str(e))
                return
        else:
            fire_dets = self._fire_cache
        _t2 = time.time()

        try:
            pose_dets = self._infer_pose(color_np)
        except Exception as e:                                  # noqa: BLE001
            self._fail_frame('pose', str(e))
            return
        _t3 = time.time()

        # 여기 도달 = 두 모델 모두 정상. 빈 결과는 "정상 미탐지"다 (B1).

        # ---- N5 모델 일치 카운터 (C2-b) ----------------------------------
        human_count = sum(1 for d in fire_dets if d[0] == 'human')
        self.counters.update(pose_count=len(pose_dets), human_count=human_count)

        # ---- 결과 조립 ---------------------------------------------------
        now = self.get_clock().now().nanoseconds / 1e9
        gravity = self._gravity_axis()
        items: List[Tuple[str, float, Tuple[int, int, int, int], Tuple]] = []
        persistent_unknown = 0
        implausible_now = 0

        # 사람 (pose 단일 소스)
        for tid, conf, bbox, kpts, kconf in pose_dets:
            r = self.judge.update(tid, kpts, kconf, bbox,
                                  gravity=gravity, now=now)
            if r['persistent_unknown']:
                persistent_unknown += 1
            if r['implausible_angle']:
                implausible_now += 1
            if self.debug_pose:
                self.get_logger().info(
                    '  #%s ang=%s ar=%.2f v=%d f=%d thr=%.0f' % (
                        tid,
                        '--' if r['angle'] is None else '%.0f' % r['angle'],
                        r['aspect_ratio'], r['valid'], r['fallen'],
                        r['angle_thr']))
            roi = self._keypoint_roi(kpts, kconf) or self._center_roi(bbox)
            pos = self._to_3d(depth_np, roi)
            if pos is None:
                continue                                   # A9-1
            items.append((r['class_name'], conf, bbox, pos))

        # 사람 bbox 목록 (fire 오탐 억제용 — 08-25, 카모/피부 텍스처가 fire로 오탐되는
        # 문제 대응. fire bbox 대부분이 person bbox 안에 포함될 때만 억제한다.
        # 사람 옆에 별도로 존재하는 실제 화재(핵심 임무 시나리오)는 겹치는 비율이
        # 낮으므로 그대로 통과시킨다.)
        _person_bboxes = [bbox for _cn, _c, bbox, _p in items]
        # 시간적 보강: 이번 프레임에 person이 잡히면 기억을 갱신하고,
        # 이번 프레임에 person이 하나도 없으면 최근 기억(0.5초 이내)을
        # 대신 사용한다 — pose가 간헐적으로 사람을 놓치는 틈에 fire 억제가
        # 무력화되는 것을 막는다 (08-28).
        if _person_bboxes:
            self._recent_person_bboxes = list(_person_bboxes)
            self._recent_person_time = now
        elif now - self._recent_person_time <= self._PERSON_MEMORY_SEC:
            _person_bboxes = list(self._recent_person_bboxes)

        def _fire_person_overlap_and_arearatio(fire_bbox, person_bboxes):
            """반환: (최대 겹침비율, 그때의 fire/person 면적비율) 또는 (0, 0)"""
            fx1, fy1, fx2, fy2 = fire_bbox
            fire_area = max(0, fx2 - fx1) * max(0, fy2 - fy1)
            if fire_area <= 0:
                return 0.0, 0.0
            best_overlap = 0.0
            best_arearatio = 0.0
            for px1, py1, px2, py2 in person_bboxes:
                person_area = max(0, px2 - px1) * max(0, py2 - py1)
                if person_area <= 0:
                    continue
                ix1, iy1 = max(fx1, px1), max(fy1, py1)
                ix2, iy2 = min(fx2, px2), min(fy2, py2)
                inter = max(0, ix2 - ix1) * max(0, iy2 - iy1)
                overlap = inter / fire_area
                if overlap > best_overlap:
                    best_overlap = overlap
                    best_arearatio = fire_area / person_area
            return best_overlap, best_arearatio

        # 화재/연기 (human 은 싣지 않는다 — C2-b)
        for name, conf, bbox in fire_dets:
            if name not in FIRE_PUBLISH:
                continue
            if name == 'fire':
                overlap, arearatio = _fire_person_overlap_and_arearatio(bbox, _person_bboxes)
                if overlap >= 0.6:
                    # 겹침은 있으나 fire 영역이 person 대비 작으면(소품 가능성)
                    # 억제하지 않고 그대로 통과시킨다. 몸 전체를 덮는 텍스처
                    # 오탐만 잡는다 (08-25 실측: 오탐은 arearatio 0.7~1.1대,
                    # 소품/직접 든 fire는 이보다 뚜렷이 작을 것으로 추정 — 검증 필요).
                    if arearatio < 0.5 and conf >= 0.6:
                        # 면적비가 작고(=몸 전체를 덮지 않음) confidence도 높은 경우만
                        # '들고 있는 소품' 가능성으로 보고 통과시킨다. 08-25 실측:
                        # 손에 든 실제 fire는 conf 0.8~0.9대. 몸 일부의 저confidence
                        # 오탐(conf 0.4~0.6대, arearatio 작음)은 이 조건으로 걸러진다.
                        self.get_logger().info(
                            'fire person 겹침(overlap=%.2f) 이나 면적비 작고 conf 높음(ratio=%.2f) — 통과 (conf=%.2f)'
                            % (overlap, arearatio, conf))
                    else:
                        self.get_logger().warn(
                            'fire 오탐 의심 — person bbox 내부 포함 억제 '
                            '(conf=%.2f overlap=%.2f arearatio=%.2f)' % (conf, overlap, arearatio))
                        continue
            pos = self._to_3d(depth_np, self._center_roi(bbox))
            if pos is None:
                continue
            items.append((name, conf, bbox, pos))

        # ---- 정규화 + 열거값 검사 + 하한 (N2/N3, C4) ----------------------
        msg = Detection3DArray()
        msg.header.stamp = color_msg.header.stamp          # A3 촬영시각
        msg.header.frame_id = color_msg.header.frame_id    # A4 optical frame

        for name, conf, bbox, pos in items:
            name = normalize_class_name(name)              # N3
            if not is_allowed_class(name):                 # N2/N4
                self.get_logger().warn('열거값 밖 class_name 차단: %s' % name)
                continue
            if conf < self.min_conf:                       # C4
                continue

            det = Detection3D()
            det.class_name = name
            det.confidence = float(conf)

            roi = RegionOfInterest()
            x1, y1, x2, y2 = [int(v) for v in bbox]
            roi.x_offset = max(0, x1)
            roi.y_offset = max(0, y1)
            roi.width = max(0, x2 - x1)
            roi.height = max(0, y2 - y1)
            det.bbox = roi

            p = Point()
            p.x, p.y, p.z = float(pos[0]), float(pos[1]), float(pos[2])
            det.position = p

            msg.detections.append(det)

        _t4 = time.time()
        self.pub.publish(msg)
        self.frames_published += 1
        _t5 = time.time()

        # ---- 시연용 디버그 화면 (박스+라벨+거리) --------------------------
        try:
            _d0 = time.time()
            debug_img = color_np.copy()
            _d1 = time.time()
            for det in msg.detections:
                x1 = det.bbox.x_offset
                y1 = det.bbox.y_offset
                x2 = x1 + det.bbox.width
                y2 = y1 + det.bbox.height
                if det.class_name == 'fire':
                    color = (0, 0, 255)
                elif det.class_name == 'person_fallen':
                    color = (0, 165, 255)
                elif det.class_name == 'person_ok':
                    color = (0, 255, 0)
                else:
                    color = (0, 255, 255)
                cv2.rectangle(debug_img, (x1, y1), (x2, y2), color, 2)
                label = '%s %.2f %.1fm' % (
                    det.class_name, det.confidence, det.position.z)
                cv2.putText(debug_img, label, (x1, max(0, y1 - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            _d2 = time.time()
            # 시연 화면 확대 (원본 640x480은 뷰어에서 작게 보임 — 08-28, 3배)
            debug_img = cv2.resize(debug_img, (int(debug_img.shape[1] * 1.5), int(debug_img.shape[0] * 1.5)),
                                    interpolation=cv2.INTER_LINEAR)
            debug_msg = Image()
            debug_msg.height = debug_img.shape[0]
            debug_msg.width = debug_img.shape[1]
            debug_msg.encoding = 'rgb8'
            debug_msg.is_bigendian = 0
            debug_msg.step = debug_img.shape[1] * debug_img.shape[2]
            debug_msg.data = array.array('B', debug_img.tobytes())
            debug_msg.header = msg.header
            _d3 = time.time()
            self.debug_img_pub.publish(debug_msg)
            _d4 = time.time()
            if self._timing_count % 10 == 0:
                self.get_logger().info(
                    'TIMING3 copy=%.1fms draw=%.1fms tobytes=%.1fms pub=%.1fms'
                    % ((_d1-_d0)*1000, (_d2-_d1)*1000, (_d3-_d2)*1000, (_d4-_d3)*1000))
        except Exception as e:
            self.get_logger().warn('debug_image 발행 실패: %s' % str(e))
        _t6 = time.time()
        if not hasattr(self, '_timing_count'):
            self._timing_count = 0
        self._timing_count += 1
        if self._timing_count % 10 == 0:
            self.get_logger().info(
                'TIMING2 fire=%.1fms pose=%.1fms judge=%.1fms publish=%.1fms debug_img=%.1fms TOTAL=%.1fms'
                % ((_t2-_t1)*1000, (_t3-_t2)*1000, (_t4-_t3)*1000,
                   (_t5-_t4)*1000, (_t6-_t5)*1000, (_t6-_t0)*1000))

        if implausible_now:
            self.get_logger().warn(
                'implausible torso angle x%d — 카메라 장착 방향 또는 TF 확인 필요'
                % implausible_now)

        self._publish_diag(ok=True, extra={
            'detections': len(msg.detections),
            'persistent_unknown': persistent_unknown,
            'implausible_now': implausible_now,
        })

    # ======================================================================
    # 추론
    # ======================================================================
    def _infer_fire(self, frame) -> List[Tuple[str, float, Tuple]]:
        """반환 [(class_name, conf, (x1,y1,x2,y2)), ...] — 정규화 전 원본 라벨."""
        d = self.fire_model.predict(frame, threshold=self.fire_conf)
        out = []
        if d.is_empty():
            return out
        for i in range(len(d.xyxy)):
            cid = int(d.class_id[i])
            name = RFDETR_CLASS_NAMES.get(cid)
            if name is None:                    # 0=car 등 (N4)
                continue
            out.append((name, float(d.confidence[i]),
                        tuple(float(v) for v in d.xyxy[i])))
        return out

    def _infer_pose(self, frame):
        """반환 [(track_id, conf, bbox, keypoints, kpt_conf), ...]"""
        res = self.pose_model.track(frame, persist=True, conf=self.pose_conf,
                                    classes=[0], verbose=False)[0]
        out = []
        boxes, kps = res.boxes, res.keypoints
        if boxes is None or len(boxes) == 0 or kps is None:
            return out
        ids = (boxes.id.int().cpu().tolist() if boxes.id is not None
               else list(range(len(boxes))))
        xyxy = boxes.xyxy.cpu().numpy()
        confs = boxes.conf.cpu().numpy()
        kxy = kps.xy.cpu().numpy()
        kcf = kps.conf.cpu().numpy() if kps.conf is not None else None
        for i, tid in enumerate(ids):
            out.append((
                int(tid), float(confs[i]),
                tuple(float(v) for v in xyxy[i]),
                kxy[i].tolist(),
                kcf[i].tolist() if kcf is not None else [1.0] * 17,
            ))
        return out

    # ======================================================================
    # depth -> 3D (C5)
    # ======================================================================
    def _to_3d(self, depth_np, roi) -> Optional[Tuple[float, float, float]]:
        """
        optical frame 규약: x=우, y=하, z=전방(거리).
        기존 코드는 거리를 x 에 넣고 y,z=0 이었다. 역할 A 가 오독하므로 정정.
        camera_info 미수신 시에는 z 만 채우고 x,y=0 (역투영 불가).
        """
        if roi is None:
            return None
        h, w = depth_np.shape[:2]
        x1, y1, x2, y2 = roi
        x1 = max(0, min(w - 1, int(x1))); x2 = max(0, min(w, int(x2)))
        y1 = max(0, min(h - 1, int(y1))); y2 = max(0, min(h, int(y2)))
        if x2 <= x1 or y2 <= y1:
            return None

        patch = depth_np[y1:y2, x1:x2].astype(np.float32) / 1000.0   # A5
        valid = patch[(patch >= self.depth_min_m) & (patch <= self.depth_max_m)]
        if valid.size < self.depth_min_px:
            return None
        if valid.size < patch.size * self.depth_min_frac:
            return None

        z = float(np.median(valid))
        u = (x1 + x2) / 2.0
        v = (y1 + y2) / 2.0
        if self.fx is None:
            return (0.0, 0.0, z)
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        return (x, y, z)

    def _keypoint_roi(self, kpts, kconf):
        """C5 추가 제안 — 몸통 keypoint 사각형. 신뢰도 미달이면 None(폴백)."""
        idx = (L_SHOULDER, R_SHOULDER, L_HIP, R_HIP)
        try:
            if any(kconf[i] < 0.5 for i in idx):
                return None
            xs = [kpts[i][0] for i in idx]
            ys = [kpts[i][1] for i in idx]
        except (IndexError, TypeError):
            return None
        return (min(xs), min(ys), max(xs), max(ys))

    @staticmethod
    def _center_roi(bbox):
        """bbox 외곽 각 25% 제외한 중앙 영역 (B2)."""
        x1, y1, x2, y2 = bbox
        bw, bh = x2 - x1, y2 - y1
        return (x1 + bw * 0.25, y1 + bh * 0.25, x2 - bw * 0.25, y2 - bh * 0.25)

    # ======================================================================
    # N8 중력축 — M2 회신 전까지 폴백
    # ======================================================================
    def _gravity_axis(self):
        """
        TODO(M2): 역할 A 회신 후 tf2 로
                  <중력 정렬 프레임> -> camera optical frame 조회하여
                  카메라 좌표계 중력 벡터를 이미지 평면에 투영한다.
        반환 None = 폴백 (이미지 y축 + 임계 70도)
        """
        if not self.use_gravity:
            return None
        return (0.0, 1.0)

    # ======================================================================
    # 실패 처리 (N1) / 진단
    # ======================================================================
    def _fail_frame(self, stage: str, detail: str) -> None:
        self.frames_dropped += 1
        self.get_logger().error('frame dropped [%s]: %s' % (stage, detail))
        self._publish_diag(ok=False, extra={'failed_stage': stage,
                                            'detail': detail[:200]})

    def _wd_depth_probe(self, msg: Image) -> None:
        self._wd_last_depth = time.time()
        self._wd_depth_count += 1

    def _watchdog(self) -> None:
        """콜백과 독립적으로 입력 스트림 생존을 감시한다 (Y1)."""
        now = time.time()
        n_color = self.count_publishers(self._color_topic)
        n_depth = self.count_publishers(self._depth_topic)
        since_start = now - self._wd_start
        cb_age = (now - self._wd_last_cb) if self._wd_last_cb else None

        depth_age = (now - self._wd_last_depth) if self._wd_last_depth else None

        if n_depth == 0:
            state, msg = 'FATAL', 'depth_stream_missing'
        elif n_color == 0:
            state, msg = 'FATAL', 'color_stream_missing'
        elif depth_age is None and since_start > self._wd_grace:
            # 발행자는 등록됐는데 프레임이 한 장도 안 왔다.
            state, msg = 'FATAL', 'depth_publisher_silent'
        elif depth_age is not None and depth_age > self._wd_stale:
            state, msg = 'FATAL', 'depth_frames_stale'
        elif cb_age is None:
            if since_start > self._wd_grace:
                # depth 도 color 도 흐르는데 짝이 안 맞는다 = 동기화 실패
                state, msg = 'FATAL', 'sync_failed_no_callback'
            else:
                state, msg = 'STARTING', 'waiting_first_frame'
        elif cb_age > self._wd_stale:
            state, msg = 'ERROR', 'callback_stale'
        else:
            state, msg = 'OK', 'ok'

        detail = ('%s (color_pub=%d depth_pub=%d depth_frames=%d '
                  'depth_age=%s cb_age=%s uptime=%.0fs)'
                  % (msg, n_color, n_depth, self._wd_depth_count,
                     '--' if depth_age is None else '%.1fs' % depth_age,
                     '--' if cb_age is None else '%.1fs' % cb_age,
                     since_start))

        # 상태가 바뀔 때만 로그. FATAL 은 10초마다 반복해서 알린다.
        changed = state != self._wd_last_state
        repeat = state == 'FATAL' and int(since_start) % 10 == 0
        if changed or repeat:
            if state == 'FATAL':
                self.get_logger().error('WATCHDOG %s' % detail)
            elif state == 'ERROR':
                self.get_logger().warn('WATCHDOG %s' % detail)
            elif changed:
                self.get_logger().info('WATCHDOG %s' % detail)
        self._wd_last_state = state

        # 콜백이 안 도는 동안에도 /diagnostics 를 계속 내보낸다.
        if state in ('FATAL', 'ERROR'):
            self._publish_diag(ok=False, extra={
                'watchdog': state,
                'watchdog_reason': msg,
                'color_publishers': n_color,
                'depth_publishers': n_depth,
                'callback_age_sec': '--' if cb_age is None else round(cb_age, 2),
                'depth_frames_total': self._wd_depth_count,
                'depth_age_sec': '--' if depth_age is None else round(depth_age, 2),
                'uptime_sec': round(since_start, 1),
            })

    def _publish_diag(self, ok: bool, extra: Optional[dict] = None) -> None:
        st = DiagnosticStatus()
        st.name = 'perception_node'
        st.hardware_id = 'jetson'
        st.level = DiagnosticStatus.OK if ok else DiagnosticStatus.ERROR
        st.message = 'ok' if ok else 'frame not published'

        kv = dict(self.counters.as_dict())
        kv.update({
            'frames_published': self.frames_published,
            'frames_dropped': self.frames_dropped,
            'implausible_angle_frames': self.judge.implausible_frames,
            'camera_info': 'yes' if self.fx is not None else 'MISSING',
        })
        if extra:
            kv.update(extra)
        st.values = [KeyValue(key=str(k), value=str(v)) for k, v in kv.items()]

        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [st]
        self.diag_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
