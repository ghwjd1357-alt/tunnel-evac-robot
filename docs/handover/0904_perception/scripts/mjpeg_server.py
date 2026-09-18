"""/camera/debug_image 를 브라우저에서 실시간으로 보기 위한 MJPEG 스트리머.

노트북과 젯슨이 아이폰 핫스팟(172.20.10.x)에 물려 있어 DDS 멀티캐스트가
넘어오지 않는다. 그래서 ROS 토픽을 노트북으로 끌어오는 대신, 젯슨에서
JPEG 로 감싸 HTTP 로 밀어준다. 브라우저 주소창에 http://<젯슨IP>:8081 만
치면 보인다 (플러그인·설치 불필요).
"""
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

PORT = 8081
JPEG_QUALITY = 80

_latest = {'jpg': None}
_lock = threading.Lock()


class Grab(Node):
    def __init__(self):
        super().__init__('mjpeg_grab')
        self.n = 0
        self.create_subscription(Image, '/camera/debug_image', self.cb, 1)

    def cb(self, msg):
        # cv_bridge 는 numpy ABI 충돌로 못 쓴다 — bgr8 을 직접 reshape
        # 토픽은 rgb8, cv2.imencode 는 BGR 을 기대한다 → 채널을 뒤집는다
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        rgb = buf.reshape(msg.height, msg.step // 3, 3)[:, :msg.width]
        img = np.ascontiguousarray(rgb[:, :, ::-1])
        ok, buf = cv2.imencode('.jpg', img,
                               [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if ok:
            with _lock:
                _latest['jpg'] = buf.tobytes()
        self.n += 1
        if self.n % 120 == 0:
            print('streamed=%d' % self.n, flush=True)


PAGE = b"""<!doctype html><meta charset=utf-8><title>robot view</title>
<style>body{margin:0;background:#141414;display:flex;align-items:center;
justify-content:center;height:100vh}img{max-width:100%;max-height:100vh}</style>
<img src="/stream">"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path != '/stream':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(PAGE)
            return

        self.send_response(200)
        self.send_header('Content-Type',
                         'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        last = None
        try:
            while True:
                with _lock:
                    jpg = _latest['jpg']
                if jpg is None or jpg is last:
                    # 새 프레임이 아직 없다 — CPU 를 태우지 않도록 잠깐 쉰다
                    threading.Event().wait(0.01)
                    continue
                last = jpg
                self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n'
                                 b'Content-Length: %d\r\n\r\n' % len(jpg))
                self.wfile.write(jpg)
                self.wfile.write(b'\r\n')
        except (BrokenPipeError, ConnectionResetError):
            pass


def main():
    srv = ThreadingHTTPServer(('0.0.0.0', PORT), Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print('MJPEG http://0.0.0.0:%d' % PORT, flush=True)

    rclpy.init()
    node = Grab()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


main()
