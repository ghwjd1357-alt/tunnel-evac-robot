#!/usr/bin/env python3
"""bag 프레임 한 장에 pose 모델을 직접 물려 무엇이 나오는지 본다."""
import numpy as np, cv2, rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from ultralytics import YOLO

r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri='/home/hanhan/person_bag_0904', storage_id='sqlite3'),
       rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/camera/color/image_raw']))

t0 = None; frame = None
while r.has_next():
    _, d, t = r.read_next()
    if t0 is None: t0 = t
    if (t - t0) / 1e9 < 6: continue
    m = deserialize_message(d, Image)
    frame = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3).copy()
    break
print('프레임', frame.shape)

for path in ['/home/hanhan/pose_finetuned_meta.engine',
             '/home/hanhan/yolo11n-pose_meta.engine',
             '/home/hanhan/yolo11n-pose.pt']:
    try:
        mdl = YOLO(path)
        print(f'\n── {path}')
        print('   names:', getattr(mdl, "names", None))
        for conf in (0.5, 0.25, 0.1):
            res = mdl.predict(frame, conf=conf, verbose=False)[0]
            n = 0 if res.boxes is None else len(res.boxes)
            cls = [] if n == 0 else res.boxes.cls.int().cpu().tolist()
            cf = [] if n == 0 else [round(float(x), 2) for x in res.boxes.conf.cpu().tolist()]
            print(f'   conf={conf}: {n} 개  cls={cls[:5]}  conf={cf[:5]}')
    except Exception as e:
        print(f'\n── {path}\n   실패: {e}')
