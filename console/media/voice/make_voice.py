#!/usr/bin/env python3
"""안내 음성 mp3 생성 — 디스플레이 스피커용 (2026-09-18)

문구는 여기 적지 않는다. `console/js/i18n.js` 의 DISPLAY_KO(화면 문구)를 그대로 읽어
"주 문구. 보조 문구." 로 합쳐 읽는다 — 화면과 소리가 갈라질 자리를 없앤다.

    사전 1회:  sudo apt install python3-pip && python3 -m pip install --user edge-tts
    실행:      python3 console/media/voice/make_voice.py          (인터넷 필요 — 생성할 때만)
    확인:      node console/test/test_audio.mjs

음성 = Microsoft edge-tts `ko-KR-SunHiNeural`. 만든 mp3 는 저장소에 동봉하므로
로봇(젯슨)은 인터넷 없이 재생한다. 문구를 바꾸면 이 스크립트를 다시 돌린다.
"""
import asyncio, pathlib, re, sys

HERE = pathlib.Path(__file__).resolve().parent
I18N = HERE.parent.parent / 'js' / 'i18n.js'
VOICE = 'ko-KR-SunHiNeural'
RATE = '-5%'
# js/audio.js VOICE_STATES 와 같아야 한다 — FAULT·BLOCKED 는 일부러 없다 (소리로 "정지" 를 외치지 않는다)
STATES = ['PATROL', 'APPROACH', 'SCAN_AREA', 'GATHER', 'GUIDE', 'HOLD',
          'SEARCH_BACK', 'RESCUE', 'NO_VICTIM', 'ESCAPED']


def read_display_ko():
    src = I18N.read_text(encoding='utf-8')
    block = re.search(r'export const DISPLAY_KO = \{(.*?)\n\};', src, re.S)
    if not block:
        sys.exit('DISPLAY_KO 를 i18n.js 에서 못 찾았다')
    out = {}
    for m in re.finditer(r"(\w+):\s*\['([^']*)',\s*'([^']*)'\]", block.group(1)):
        out[m.group(1)] = (m.group(2), m.group(3))
    return out


async def main():
    try:
        import edge_tts
    except ImportError:
        sys.exit('edge-tts 없음:  python3 -m pip install --user edge-tts')
    ko = read_display_ko()
    missing = [s for s in STATES if s not in ko]
    if missing:
        sys.exit(f'i18n.js DISPLAY_KO 에 없는 상태: {missing}')
    for st in STATES:
        main_t, sub_t = ko[st]
        text = f'{main_t}. {sub_t}.'
        out = HERE / f'{st}.mp3'
        await edge_tts.Communicate(text, VOICE, rate=RATE).save(str(out))
        print(f'{st:12s} {out.stat().st_size:6d} B  "{text}"')


if __name__ == '__main__':
    asyncio.run(main())
