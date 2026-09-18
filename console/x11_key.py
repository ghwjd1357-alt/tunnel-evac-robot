#!/usr/bin/env python3
"""x11_key.py — X 서버에 키 입력을 흉내 낸다 (xdotool 대체 · 2026-09-18)

    python3 console/x11_key.py Escape          # GNOME "Activities" 화면 닫기
    python3 console/x11_key.py F11             # 예: 전체화면 토글

왜 있나: 젯슨은 자동 로그인 직후 GNOME 이 "Activities"(앱 목록) 화면을 띄운 채로 두고,
나중에 뜬 kiosk 파이어폭스가 그것을 닫지 못한다(09-18 패널 캡처). xdotool 이 없고
sudo 없이 깔 수도 없어서, 이미 있는 libX11/libXtst 를 ctypes 로 직접 부른다.
DISPLAY 환경변수가 가리키는 X 서버에 Escape 한 번 = 사람이 키보드를 누른 것과 같다.
"""
import ctypes, os, sys, time

def press(keysym_name: str) -> None:
    x11 = ctypes.CDLL('libX11.so.6')
    xtst = ctypes.CDLL('libXtst.so.6')
    x11.XOpenDisplay.restype = ctypes.c_void_p
    x11.XStringToKeysym.restype = ctypes.c_ulong
    x11.XKeysymToKeycode.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
    x11.XFlush.argtypes = [ctypes.c_void_p]
    x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
    xtst.XTestFakeKeyEvent.argtypes = [ctypes.c_void_p, ctypes.c_uint, ctypes.c_int, ctypes.c_ulong]

    dpy = x11.XOpenDisplay(None)
    if not dpy:
        sys.exit(f'X 서버를 못 열었다 (DISPLAY={os.environ.get("DISPLAY")})')
    keysym = x11.XStringToKeysym(keysym_name.encode())
    if keysym == 0:
        sys.exit(f'모르는 키 이름: {keysym_name}')
    code = x11.XKeysymToKeycode(dpy, keysym)
    xtst.XTestFakeKeyEvent(dpy, code, 1, 0)
    x11.XFlush(dpy)
    time.sleep(0.05)
    xtst.XTestFakeKeyEvent(dpy, code, 0, 0)
    x11.XFlush(dpy)
    x11.XCloseDisplay(dpy)


if __name__ == '__main__':
    press(sys.argv[1] if len(sys.argv) > 1 else 'Escape')
