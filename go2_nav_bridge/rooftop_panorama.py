#!/usr/bin/env python3
"""rooftop_panorama — 제자리 회전 5방향 촬영 + 회전 중 동영상 녹화 + 이메일.

정면(0deg)을 기준으로 왼쪽 90도 -> 오른쪽으로 45도씩 돌며 5장(-90,-45,0,+45,+90)
을 찍고, 회전하는 내내 프레임을 연속 저장해 mp4로 만든다. 평지에서도 단독
실행 가능 (계단/옥상 불필요). 로봇은 제자리 회전만 하고 전진하지 않는다.

  ros2 run go2_nav_bridge rooftop_panorama            # 촬영+영상+이메일
  ros2 run go2_nav_bridge rooftop_panorama --no-email
  ros2 run go2_nav_bridge rooftop_panorama --no-video --no-email
"""
import json
import math
import os
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from unitree_api.msg import Request, Response

from go2_nav_bridge import email_sender
from go2_nav_bridge.photo_shooter import API_VIDEO_GET_IMAGE_SAMPLE
from go2_nav_bridge.stair_traverse_node import StairTraverseNode


def _env_float(name, default):
    try:
        return float(os.environ.get(name, default))
    except ValueError:
        return default


# 회전 명령각에서 뺄 오버슈트 보정 (실측: 90도 명령 -> +2.4~4.2도, 87도 -> +0.5도)
TURN_SPEED_RAD_S = _env_float('PANO_TURN_SPEED', 0.6)
# 정지 지연 ~0.08s 가정, 속도에 비례해 보정 (실측 오버슈트: 0.35rad/s 약 2.5도, 0.8rad/s 약 3.6도)
OVERSHOOT_COMP_DEG = _env_float(
    'PANO_OVERSHOOT_COMP_DEG', math.degrees(TURN_SPEED_RAD_S * 0.08))
SETTLE_SEC = _env_float('PANO_SETTLE_SEC', 1.0)
VIDEO_FPS = _env_float('PANO_VIDEO_FPS', 15.0)
VIDEO_MAX_MB = _env_float('PANO_VIDEO_MAX_MB', 18.0)
OUTPUT_DIR = os.path.expanduser(os.environ.get('PANO_OUTPUT_DIR', '~/photos'))

# (오른쪽 회전 기준 상대 이동각) - 왼쪽 90도로 시작해 45도씩 오른쪽으로
# 라벨은 정면 기준 각도 (음수=왼쪽)
SWEEP = [(-90, 'L90'), (-45, 'L45'), (0, 'C0'), (45, 'R45'), (90, 'R90')]


class FrameGrabber(Node):
    """videohub JPEG 프레임 요청. 별도 스레드 executor가 응답 콜백을 처리."""

    def __init__(self):
        super().__init__('pano_frame_grabber')
        self.pub = self.create_publisher(Request, '/api/videohub/request', 10)
        self.create_subscription(
            Response, '/api/videohub/response', self._on_resp, 10)
        self._lock = threading.Lock()
        self._event = threading.Event()
        self._pending_id = None
        self._result = None

    def _on_resp(self, msg):
        if self._pending_id is not None and msg.header.identity.id == self._pending_id:
            self._result = msg
            self._event.set()

    def grab(self, timeout=3.0):
        """JPEG bytes 또는 None."""
        with self._lock:
            req = Request()
            self._pending_id = time.time_ns()
            req.header.identity.id = self._pending_id
            req.header.identity.api_id = API_VIDEO_GET_IMAGE_SAMPLE
            req.parameter = ''
            self._result = None
            self._event.clear()
            self.pub.publish(req)
            if not self._event.wait(timeout):
                return None
            res = self._result
            if res.header.status.code != 0 or not res.binary:
                return None
            return bytes(res.binary)


class VideoRecorder:
    def __init__(self, grabber):
        self.grabber = grabber
        self.frames = []
        self._stop = threading.Event()
        self._thread = None
        self.t_start = None
        self.t_end = None

    def start(self):
        self.t_start = time.time()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        period = 1.0 / VIDEO_FPS
        while not self._stop.is_set():
            t0 = time.time()
            jpg = self.grabber.grab(timeout=1.0)
            if jpg:
                self.frames.append(jpg)
            time.sleep(max(0.0, period - (time.time() - t0)))

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=3.0)
        self.t_end = time.time()

    def save_mp4(self, path, log):
        n = len(self.frames)
        if n < 2:
            log(f'영상 프레임 부족({n}장) - 영상 생략')
            return None
        elapsed = max(self.t_end - self.t_start, 0.1)
        fps = max(1.0, min(30.0, n / elapsed))
        first = cv2.imdecode(np.frombuffer(self.frames[0], np.uint8), cv2.IMREAD_COLOR)
        h, w = first.shape[:2]
        scale = min(1.0, _env_float('PANO_VIDEO_WIDTH', 1920.0) / w)
        for _ in range(4):
            out_w, out_h = int(w * scale) // 2 * 2, int(h * scale) // 2 * 2
            writer = cv2.VideoWriter(
                path, cv2.VideoWriter_fourcc(*'avc1'), fps, (out_w, out_h))
            if not writer.isOpened():  # H.264 인코더 없으면 mp4v로 폴백
                writer = cv2.VideoWriter(
                    path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (out_w, out_h))
            for jpg in self.frames:
                img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
                if img is None:
                    continue
                if (out_w, out_h) != (w, h):
                    img = cv2.resize(img, (out_w, out_h))
                writer.write(img)
            writer.release()
            size_mb = os.path.getsize(path) / 1e6
            log(f'영상 저장: {path} ({n}프레임, {fps:.1f}fps, {out_w}x{out_h}, '
                f'{size_mb:.1f}MB)')
            if size_mb <= VIDEO_MAX_MB:
                return path
            scale *= 0.7
            log(f'영상이 {VIDEO_MAX_MB:.0f}MB 초과 - 해상도 낮춰 재인코딩')
        log('영상이 너무 커서 첨부 생략')
        return None


def run_sweep(stair_node, grabber, recorder, turn_dir, out_dir, ts):
    """제자리 회전 5방향 촬영. 반환: (성공여부, [(라벨, 사진경로)])"""
    log = stair_node.get_logger().info
    photos = []
    turn_left = turn_dir           # stair_node.turn_direction (왼쪽 회전 방향 부호)
    prev = 0

    def rotate(deg_signed):
        # deg_signed < 0: 왼쪽, > 0: 오른쪽
        mag = max(abs(deg_signed) - OVERSHOOT_COMP_DEG, 1.0)
        direction = turn_left if deg_signed < 0 else -turn_left
        return stair_node.run_turn_segment(mag, direction)

    for angle, label in SWEEP:
        step = angle - prev
        if step != 0 and not rotate(step):
            return False, photos
        prev = angle
        stair_node.publish_move(0.0, 0.0, 0.0)
        time.sleep(SETTLE_SEC)
        jpg = None
        for _ in range(3):
            jpg = grabber.grab(timeout=2.0)
            if jpg:
                break
        if jpg:
            path = os.path.join(out_dir, f'pano_{ts}_{label}.jpg')
            with open(path, 'wb') as f:
                f.write(jpg)
            photos.append((label, path))
            log(f'사진 저장 [{label}]: {path} ({len(jpg)} bytes)')
        else:
            stair_node.get_logger().warn(f'사진 실패 [{label}]')

    # 정면 복귀
    if not rotate(0 - prev):
        return False, photos
    stair_node.publish_move(0.0, 0.0, 0.0)
    return True, photos


def capture_panorama(record_video=True):
    """rclpy가 이미 init된 상태에서 5방향 촬영(+영상). 반환: (ok, photos, video_path)"""
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    ts = time.strftime('%Y%m%d_%H%M%S')

    stair_node = StairTraverseNode()
    stair_node.turn_speed_rad_s = TURN_SPEED_RAD_S
    stair_node.mission_start_time = time.time()
    stair_node.max_duration_sec = _env_float('MISSION_MAX_DURATION_SEC', 240.0)
    grabber = FrameGrabber()
    executor = SingleThreadedExecutor()
    executor.add_node(grabber)
    threading.Thread(target=executor.spin, daemon=True).start()

    recorder = VideoRecorder(grabber) if record_video else None
    ok = False
    photos = []
    video_path = None
    try:
        # 첫 프레임 확인 (카메라 응답 없으면 회전 자체를 안 함)
        # 방금 만든 노드는 DDS 연결이 늦어 첫 요청이 유실될 수 있어 재시도
        first = None
        for _ in range(5):
            first = grabber.grab(timeout=2.0)
            if first is not None:
                break
        if first is None:
            stair_node.get_logger().warn(
                '카메라 첫 프레임 수신 실패 - 회전은 계속하고 각도마다 재시도')
        if recorder:
            recorder.start()
        ok, photos = run_sweep(
            stair_node, grabber, recorder, stair_node.turn_direction,
            OUTPUT_DIR, ts)
        if recorder:
            recorder.stop()
            if photos:
                video_path = recorder.save_mp4(
                    os.path.join(OUTPUT_DIR, f'pano_{ts}.mp4'),
                    stair_node.get_logger().info)
    finally:
        if recorder and not recorder._stop.is_set():
            recorder.stop()
        stair_node.publish_move(0.0, 0.0, 0.0)
        executor.shutdown()
        grabber.destroy_node()
        stair_node.destroy_node()
    return ok, photos, video_path


def send_panorama_email(photos, video_path, ok=True):
    body = ['옥상 5방향 촬영 결과 (정면 기준 각도)']
    body += [f'  - {label}: {os.path.basename(p)}' for label, p in photos]
    body.append(f'영상: {os.path.basename(video_path) if video_path else "없음"}')
    if not ok:
        body.append('※ 회전 시퀀스가 중간에 중단됨 (일부만 촬영)')
    attachments = [p for _, p in photos] + ([video_path] if video_path else [])
    return email_sender.send_with_retry(
        subject='[Go2] 옥상 5방향 촬영 결과', body='\n'.join(body),
        attachment_paths=attachments,
        max_wait_sec=_env_float('MISSION_EMAIL_RETRY_SEC', 1800.0))


def run_panorama(send_email=True, record_video=True):
    rclpy.init()
    try:
        ok, photos, video_path = capture_panorama(record_video)
    finally:
        rclpy.shutdown()

    print(f'[촬영] {len(photos)}/5장' + (f', 영상 {video_path}' if video_path else ''))
    if not ok:
        print('[중단] 회전 시퀀스 실패(E-stop/타임아웃 등)')
    if not send_email or not photos:
        print('[완료] 이메일 생략')
        return ok
    sent, err = send_panorama_email(photos, video_path, ok)
    print('[완료] 이메일 전송 성공' if sent else f'[실패] 이메일 전송 실패: {err}')
    return ok and sent


def main():
    argv = sys.argv[1:]
    run_panorama(send_email='--no-email' not in argv,
                 record_video='--no-video' not in argv)


if __name__ == '__main__':
    main()
