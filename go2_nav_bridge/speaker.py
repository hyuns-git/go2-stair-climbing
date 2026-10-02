#!/usr/bin/env python3
"""speaker — Go2 내장 스피커로 wav 재생 (audiohub API: 업로드 2001, 목록 1001, 재생 1002)."""
import base64
import hashlib
import json
import os
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from unitree_api.msg import Request, Response

API_GET_LIST = 1001
API_PLAY = 1002
API_UPLOAD = 2001
BLOCK = 4096
ASSET = Path(__file__).resolve().parent / 'assets' / 'mission_complete_16k.wav'


class Speaker(Node):
    def __init__(self):
        super().__init__('speaker')
        self.pub = self.create_publisher(Request, '/api/audiohub/request', 10)
        self.res = {}
        self.create_subscription(
            Response, '/api/audiohub/response',
            lambda m: self.res.update({m.header.identity.id: m}), 10)
        time.sleep(1.5)  # DDS 연결 대기

    def call(self, api, param, timeout=4.0):
        r = Request()
        i = time.time_ns()
        r.header.identity.id = i
        r.header.identity.api_id = api
        r.parameter = param if isinstance(param, str) else json.dumps(param)
        self.pub.publish(r)
        t = time.time()
        while time.time() - t < timeout and i not in self.res:
            rclpy.spin_once(self, timeout_sec=0.05)
        m = self.res.get(i)
        return (m.header.status.code, m.data) if m else (-1, '')

    def _find(self, name):
        code, data = self.call(API_GET_LIST, '')
        if code != 0 or not data:
            return None
        for a in json.loads(data).get('audio_list', []):
            a = {k.lower(): v for k, v in a.items()}  # 응답 키가 대문자로 옴
            if a.get('custom_name') == name:
                return a.get('unique_id')
        return None

    def play_wav(self, wav_path=ASSET):
        wav_path = Path(wav_path)
        name = wav_path.stem
        uid = self._find(name)
        if uid is None:
            raw = wav_path.read_bytes()
            md5 = hashlib.md5(raw).hexdigest()
            total = (len(raw) + BLOCK - 1) // BLOCK
            for idx in range(total):
                chunk = raw[idx * BLOCK:(idx + 1) * BLOCK]
                code, _ = self.call(API_UPLOAD, {
                    'file_name': name, 'file_type': 'wav', 'file_size': len(raw),
                    'current_block_index': idx + 1, 'total_block_number': total,
                    'block_content': base64.b64encode(chunk).decode(),
                    'current_block_size': len(chunk), 'file_md5': md5,
                    'create_time': int(time.time() * 1000)})
                if code != 0:
                    return False, f'업로드 실패 (블록 {idx + 1}/{total}, code={code})'
            for _ in range(5):
                uid = self._find(name)
                if uid:
                    break
                time.sleep(0.5)
            if uid is None:
                return False, '업로드 후 목록에서 파일을 찾지 못함'
        code, _ = self.call(API_PLAY, {'unique_id': uid})
        return code == 0, f'재생 요청 code={code}'


def say_mission_complete():
    """rclpy가 init된 상태에서 호출. 실패해도 예외를 던지지 않는다."""
    node = None
    try:
        node = Speaker()
        return node.play_wav()
    except Exception as e:  # 음성 실패가 미션을 깨면 안 됨
        return False, f'음성 재생 예외: {e}'
    finally:
        if node is not None:
            node.destroy_node()


def main():
    rclpy.init()
    try:
        print(say_mission_complete())
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
