#!/usr/bin/env python3
"""
검증용 변형 2종 생성. 원본은 안 건드린다.
  pn_val_bytetrack.py : 현행 ByteTrack (기준값)
  pn_val_mytrack.py   : 중심점 매칭 직접 구현

공통 적용
  · rotate_180 파라미터화 (기본 true) — 이 bag 은 카메라 정상 방향이라 false 로 돌린다
  · 프레임마다 판정을 /tmp/perftest/<name>.jsonl 로 기록
"""
import shutil, sys

SRC = "/home/hanhan/percep_ws/install/vision_pipeline/lib/python3.10/site-packages/vision_pipeline/perception_node.py"

TRACKER = '''

# ─────────────────────────────────────────────────────────────────
# [실험] 중심점 매칭 추적기 — ByteTrack 대체 후보
#   대피자 1~3명 · 서로 분리 · 프레임 간격 75~100ms 를 전제로 한다.
#   ByteTrack 은 수십 명·가림·ReID 용이라 프레임당 50ms 를 쓴다(09-04 실측).
#   여기서는 중심점 그리디 매칭 + 유실 버퍼만으로 같은 일을 한다.
# ─────────────────────────────────────────────────────────────────
class CentroidTracker:
    def __init__(self, max_dist_frac=0.25, lost_frames=30):
        self.tracks = {}        # id -> dict(cx, cy, w, h, miss)
        self.next_id = 1
        self.max_dist_frac = max_dist_frac
        self.lost_frames = lost_frames

    def update(self, boxes, img_w, img_h):
        """boxes = [(x1,y1,x2,y2), ...] -> [track_id, ...] (같은 순서)"""
        import math
        gate = self.max_dist_frac * math.hypot(img_w, img_h)
        cents = [((b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0) for b in boxes]

        # 후보 쌍을 거리순으로 정렬해 그리디 매칭
        pairs = []
        for di, (cx, cy) in enumerate(cents):
            for tid, t in self.tracks.items():
                d = math.hypot(cx - t['cx'], cy - t['cy'])
                if d <= gate:
                    pairs.append((d, di, tid))
        pairs.sort()

        assigned, used_t = {}, set()
        for d, di, tid in pairs:
            if di in assigned or tid in used_t:
                continue
            assigned[di] = tid
            used_t.add(tid)

        out = []
        for di, (cx, cy) in enumerate(cents):
            tid = assigned.get(di)
            if tid is None:                       # 새 대상
                tid = self.next_id
                self.next_id += 1
            b = boxes[di]
            self.tracks[tid] = {'cx': cx, 'cy': cy,
                                'w': b[2] - b[0], 'h': b[3] - b[1], 'miss': 0}
            out.append(tid)

        # 이번 프레임에 못 본 트랙은 유실 카운트. 버퍼를 넘으면 버린다.
        for tid in list(self.tracks):
            if tid not in used_t and tid not in out:
                self.tracks[tid]['miss'] += 1
                if self.tracks[tid]['miss'] > self.lost_frames:
                    del self.tracks[tid]
        return out
'''

def build(dst, use_mytrack):
    shutil.copy(SRC, dst)
    s = open(dst).read()

    # ── 공통 1: rotate_180 파라미터화 ─────────────────────────────
    old_rot = """            color_np = cv2.rotate(color_np, cv2.ROTATE_180)
            depth_np = cv2.rotate(depth_np, cv2.ROTATE_180)"""
    assert old_rot in s, "회전부 못 찾음"
    s = s.replace(old_rot, """            if self._rot180:
                color_np = cv2.rotate(color_np, cv2.ROTATE_180)
                depth_np = cv2.rotate(depth_np, cv2.ROTATE_180)""")
    s = s.replace("        self.frames_published = 0",
                  """        self.declare_parameter('rotate_180', True)
        self._rot180 = bool(self.get_parameter('rotate_180').value)
        self._vlog = open(os.environ.get('VLOG', '/tmp/perftest/val.jsonl'), 'w')
        self._vframe = 0
        self.frames_published = 0""", 1)

    # ── 공통 2: 프레임별 판정 기록 ────────────────────────────────
    s = s.replace("        self.pub.publish(msg)\n        self.frames_published += 1",
                  """        self.pub.publish(msg)
        self.frames_published += 1
        try:
            import json as _json
            self._vframe += 1
            self._vlog.write(_json.dumps({
                'f': self._vframe,
                'stamp': color_msg.header.stamp.sec + color_msg.header.stamp.nanosec / 1e9,
                'ids': _vids,
                'det': [[d.class_name, round(float(d.confidence), 3),
                         int(d.bbox.x_offset), int(d.bbox.y_offset),
                         int(d.bbox.width), int(d.bbox.height)] for d in msg.detections],
            }) + '\\n')
            self._vlog.flush()
        except Exception:
            pass""", 1)

    # 사람 루프에서 tid 를 모은다
    s = s.replace("        # 사람 (pose 단일 소스)\n        for tid, conf, bbox, kpts, kconf in pose_dets:",
                  "        # 사람 (pose 단일 소스)\n        _vids = []\n"
                  "        for tid, conf, bbox, kpts, kconf in pose_dets:\n"
                  "            _vids.append(int(tid))", 1)

    # ── 추적기 교체 ───────────────────────────────────────────────
    if use_mytrack:
        s = s.replace("class PerceptionNode(Node):", TRACKER + "\n\nclass PerceptionNode(Node):", 1)
        old = """        res = self.pose_model.track(frame, persist=True, conf=self.pose_conf,
                                    classes=[0], verbose=False)[0]"""
        assert old in s, "track 호출 못 찾음"
        s = s.replace(old, """        # [실험] ByteTrack 대신 predict + 중심점 매칭
        if not hasattr(self, '_ctrk'):
            self._ctrk = CentroidTracker()
        res = self.pose_model.predict(frame, conf=self.pose_conf,
                                      classes=[0], verbose=False)[0]""")
        s = s.replace("""        ids = (boxes.id.int().cpu().tolist() if boxes.id is not None
               else list(range(len(boxes))))
        xyxy = boxes.xyxy.cpu().numpy()""",
                      """        xyxy = boxes.xyxy.cpu().numpy()
        ids = self._ctrk.update([tuple(b) for b in xyxy],
                                frame.shape[1], frame.shape[0])""")
    open(dst, "w").write(s)
    print("생성:", dst)

build("/tmp/perftest/pn_val_bytetrack.py", False)
build("/tmp/perftest/pn_val_mytrack.py", True)
