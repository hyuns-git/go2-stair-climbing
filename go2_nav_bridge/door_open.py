#!/usr/bin/env python3
"""door_open — 옥상 자동문 열기 (URL에 HTTP GET 접속만 하면 열림)."""
import os

import requests

from go2_nav_bridge.email_sender import _load_env

# 인증 없이 접속만 하면 문이 열리는 주소라 코드/저장소에 넣지 않고 .env에서 읽는다.
DOOR_URL = os.environ.get('MISSION_DOOR_URL') or _load_env().get('MISSION_DOOR_URL')


def open_door(url=DOOR_URL, timeout=10.0):
    """반환: (ok, 설명). 실패해도 예외를 던지지 않는다."""
    if not url:
        return False, '.env에 MISSION_DOOR_URL이 없음'
    try:
        resp = requests.get(url, timeout=timeout)
    except requests.RequestException as e:
        return False, f'문 열기 요청 실패: {e}'
    if not 200 <= resp.status_code < 300:
        return False, f'문 열기 응답 이상: HTTP {resp.status_code}'
    return True, f'HTTP {resp.status_code} ({resp.text.strip()[:80]!r})'


if __name__ == '__main__':
    print(open_door())
