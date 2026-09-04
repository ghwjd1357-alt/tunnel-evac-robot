import os

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

OUT = '/tmp/vidframes'


def to_bgr(msg):
    """cv_bridge 를 쓰지 않고 rgb8 Image 를 BGR numpy 로 푼다.

    yolo_env 의 numpy 와 cv_bridge 가 빌드된 numpy 의 ABI 가 달라서
    imgmsg_to_cv2 가 _ARRAY_API not found 로 죽는다.

    토픽 인코딩은 rgb8 인데 cv2 는 BGR 을 기대하므로 채널을 뒤집어야 한다.
    (이걸 빼먹어서 0904 첫 영상이 파랗게 나왔다)
    """
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    rgb = buf.reshape(msg.height, msg.step // 3, 3)[:, :msg.width]
    return np.ascontiguousarray(rgb[:, :, ::-1])


class Dump(Node):
    def __init__(self):
        super().__init__('frame_dumper')
        self.n = 0
        self.create_subscription(Image, '/camera/debug_image', self.cb, 10)

    def cb(self, msg):
        cv2.imwrite('%s/%06d.jpg' % (OUT, self.n), to_bgr(msg),
                    [cv2.IMWRITE_JPEG_QUALITY, 92])
        self.n += 1
        if self.n % 60 == 0:
            print('frames=%d' % self.n, flush=True)


def main():
    os.makedirs(OUT, exist_ok=True)
    rclpy.init()
    node = Dump()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    print('TOTAL=%d' % node.n, flush=True)


main()
