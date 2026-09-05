#!/usr/bin/env python3
"""관제 정적 웹서버 — 캐시를 절대 남기지 않는다.

왜 `python3 -m http.server` 를 그대로 쓰지 않는가 (2026-09-04):
    브라우저가 ES 모듈을 캐시한다. `demo.js` 에 함수를 추가하고 `main.js` 에서
    그것을 import 하도록 바꾼 직후, Firefox 가 **낡은 demo.js + 새 main.js** 를
    섞어 물었다. 없는 export 를 import 하니 모듈 그래프 전체가 링크에 실패하고,
    그러면 `connect()` 까지 가지 못해 화면이 영원히 "연결 대기" 로 남는다.
    화면에는 아무 오류도 안 뜬다 — 조용한 실패다.

    개발 중에는 Ctrl+Shift+R 로 넘길 수 있지만, 시연 당일 그 화면을 보면
    원인을 찾을 시간이 없다. 서버가 처음부터 캐시를 금지하는 편이 안전하다.
    정적 파일 몇 개짜리라 캐시가 없어도 느려지지 않는다.
"""
import sys
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer


class NoCacheHandler(SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header('Cache-Control', 'no-store, must-revalidate')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Expires', '0')
        super().end_headers()

    def log_message(self, fmt, *a):
        # bag 로그를 덮지 않도록 화면 대신 파일로 남긴다.
        # "브라우저가 파일을 받아 갔는가" 는 조용한 실패를 가를 때 유일한 증거다.
        try:
            with open('/tmp/console_access.log', 'a') as f:
                f.write('%s %s\n' % (self.log_date_time_string(), fmt % a))
        except OSError:
            pass


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8000
    directory = sys.argv[2] if len(sys.argv) > 2 else '.'
    handler = partial(NoCacheHandler, directory=directory)
    ThreadingHTTPServer(('0.0.0.0', port), handler).serve_forever()


main()
