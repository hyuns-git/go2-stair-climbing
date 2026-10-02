#!/usr/bin/env python3
"""photo_shooter — Go2 videohub(VideoClient) JPEG 촬영 노드 (뼈대, 실기 미검증)

HANDOFF.md 1항 확인 결과 (2026-09-11):
  `import unitree_sdk2py` -> ModuleNotFoundError. 이 시스템에 unitree_sdk2py
  자체가 설치돼 있지 않아 VideoClient/GetImageSample()을 직접 호출할 수 없음.
  대신 stair_traverse_node.py가 이미 쓰고 있는 unitree_api Request/Response
  ROS2 브릿지 방식(`/api/sport/request` 등)을 그대로 재사용한다.

  서비스명 "videohub"와 api_id=1001(GetImageSample)은 Unitree 공식 프로토콜
  문서 기준값이며, unitree_ros2/example의 다른 클라이언트들
  (`/api/sport/request`, `/api/motion_switcher/request`, `/api/arm/request`,
  `/api/voice/request` — 전부 "/api/<service>/request" 패턴)과 명명 규칙이
  일치해 유추한 값이다. **이 저장소/이 시스템에는 videohub 클라이언트 예제가
  없어 토픽명·api_id·응답에 실제 JPEG가 담기는지 전부 미검증** — 로봇 연결 후
  `ros2 topic list | grep videohub`로 토픽 존재를 먼저 확인하고, 이 노드
  실행 후 로그(특히 sport API 응답 code, binary 길이)를 반드시 확인할 것.
  안 되면 `/frontvideostream`(Go2FrontVideoData, H.264) 경로로 전환 검토.

사용법 (직접 실행):
  ros2 run go2_nav_bridge photo_shooter --ros-args -p burst_count:=5

미션 코드에서 재사용:
  from go2_nav_bridge.photo_shooter import PhotoShooterNode
  node = PhotoShooterNode()   # rclpy.init()은 호출자가 미리 해뒀다고 가정
  paths = node.capture_burst()
  node.destroy_node()
"""
import json
import os
import time

import rclpy
from rclpy.node import Node
from unitree_api.msg import Request, Response

API_VIDEO_GET_IMAGE_SAMPLE = 1001


class PhotoShooterNode(Node):
    def __init__(self):
        super().__init__('photo_shooter')

        self.declare_parameter('output_dir', os.path.expanduser('~/photos'))
        self.declare_parameter('burst_count', 5)
        self.declare_parameter('burst_interval_sec', 0.5)
        # SIDE_ROOFTOP.md 권장: 정지 후 보행 진동 가라앉을 때까지 대기
        self.declare_parameter('post_stop_delay_sec', 2.0)
        self.declare_parameter('response_timeout_sec', 3.0)

        p = self.get_parameter
        self.output_dir = os.path.expanduser(p('output_dir').value)
        self.burst_count = p('burst_count').value
        self.burst_interval_sec = p('burst_interval_sec').value
        self.post_stop_delay_sec = p('post_stop_delay_sec').value
        self.response_timeout_sec = p('response_timeout_sec').value

        self.video_req_pub = self.create_publisher(
            Request, '/api/videohub/request', 10)
        self.video_resp_sub = self.create_subscription(
            Response, '/api/videohub/response', self._on_video_response, 10)
        self._pending_id = None
        self._pending_result = None

    def _on_video_response(self, msg: Response):
        if self._pending_id is not None and msg.header.identity.id == self._pending_id:
            self._pending_result = msg

    def call_video_api(self, api_id, parameter=None):
        req = Request()
        self._pending_id = time.time_ns()
        req.header.identity.id = self._pending_id
        req.header.identity.api_id = api_id
        req.parameter = json.dumps(parameter) if parameter is not None else ''
        self._pending_result = None
        self.video_req_pub.publish(req)

        start = time.time()
        while time.time() - start < self.response_timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._pending_result is not None:
                break
        if self._pending_result is None:
            self.get_logger().warn(f'videohub API {api_id} 응답 타임아웃')
            return -1, None
        code = self._pending_result.header.status.code
        return code, self._pending_result.binary

    def capture_burst(self, burst_count=None, output_dir=None):
        """정지 후 대기 -> 연사 촬영 -> 저장된 파일 경로 리스트 반환.

        실패한 프레임은 건너뛰고 계속 진행 (한 장이라도 건지는 게 목표,
        HANDOFF.md 원칙대로 여기서 새 재시도/복구 로직을 설계하지 않음 —
        burst_count만큼 시도하고 끝).
        """
        burst_count = self.burst_count if burst_count is None else burst_count
        output_dir = self.output_dir if output_dir is None else os.path.expanduser(output_dir)
        os.makedirs(output_dir, exist_ok=True)

        self.get_logger().info(
            f'촬영 대기 {self.post_stop_delay_sec:.1f}s (보행 진동 가라앉힘)')
        time.sleep(self.post_stop_delay_sec)

        saved_paths = []
        for i in range(burst_count):
            code, binary = self.call_video_api(API_VIDEO_GET_IMAGE_SAMPLE)
            if code != 0 or not binary:
                self.get_logger().warn(
                    f'{i+1}/{burst_count}번째 촬영 실패 (code={code}, '
                    f'binary_len={len(binary) if binary else 0})')
                continue

            ts = time.strftime('%Y%m%d_%H%M%S')
            path = os.path.join(output_dir, f'rooftop_{ts}_{i}.jpg')
            with open(path, 'wb') as f:
                f.write(bytes(binary))
            self.get_logger().info(f'저장: {path} ({len(binary)} bytes)')
            saved_paths.append(path)

            if i < burst_count - 1:
                time.sleep(self.burst_interval_sec)

        if not saved_paths:
            self.get_logger().error('연사 전체 실패 - 사진 0장')
        else:
            self.get_logger().info(f'촬영 완료: {len(saved_paths)}/{burst_count}장 성공')
        return saved_paths


def main():
    rclpy.init()
    node = PhotoShooterNode()
    try:
        paths = node.capture_burst()
        node.get_logger().info(f'결과: {paths}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
