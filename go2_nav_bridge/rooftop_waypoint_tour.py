#!/usr/bin/env python3
"""rooftop_waypoint_tour — 옥상 평지에서 촬영 지점 5곳을 차례로 이동하며 사진 촬영.

시나리오(5F 계단 앞 -> 옥상 도착) 중 "옥상 도착 이후" 구간. 2026-10-07 조종기 시연
rosbag(run2~5)에서 5초 이상 정지한 지점을 평균한 ~/rooftop_waypoints.yaml을 읽어
각 지점마다 [제자리 회전 -> 직진 -> 지점 yaw로 회전 -> hold_sec 정지 + 사진] 을 한다.
촬영 지점은 4곳(P1~P4)이고 마지막 HOME은 출발점 복귀(정지만, `photo: false`).

원칙(mission_5f_to_rooftop.py와 동일): stair_traverse_node.py는 수정하지 않고 그
클래스의 기존 공개 메서드(run_turn_segment/run_translate_segment)만 재사용한다.
새 안전 로직은 최소한만 추가 — 리모컨 개입(E-stop)/타임아웃/roll 초과는 기존 메서드가
처리하고, 여기서는 1구간 최대 거리 상한만 둔다(옥상 가장자리 추락 방지, SIDE_ROOFTOP.md).

좌표계: 로봇을 놓은 출발 자세 기준 로컬 좌표(x 앞, y 왼쪽). 시연 때와 같은 위치/방향에
로봇을 놓고 시작해야 함 (코드가 검증 안 함). 좌표는 /sportmodestate(제어 노드와 같은
좌표계) 기준으로 추출했다.

사용법:
  ros2 run go2_nav_bridge rooftop_waypoint_tour --dry-run     # 계획만 출력, 로봇 안 움직임
  ros2 run go2_nav_bridge rooftop_waypoint_tour --only P1     # 한 지점만 (출발 자세 -> P1)
  ros2 run go2_nav_bridge rooftop_waypoint_tour               # 전체 5곳
  ros2 run go2_nav_bridge rooftop_waypoint_tour --email       # 끝나고 사진 이메일 전송
  --wp-file <경로>   (기본 ~/rooftop_waypoints.yaml)   --no-photo   사진 생략
  --goto X Y        출발 자세 기준 로컬 좌표(x 앞, y 왼쪽, m)로 한 번만 이동 (장애물 회피 시험용:
                    앞 1.5m에 의자 등을 놓고 `--goto 3 0`). 사진/yaml 없이 동작.

실행 전 확인: cmd_vel_bridge/stair_traverse_node가 떠 있으면 먼저 내려둘 것.
"""
import math
import os
import sys
import threading
import time

import numpy as np
import rclpy
import yaml
from rclpy.executors import SingleThreadedExecutor

from go2_nav_bridge import email_sender
from go2_nav_bridge.rooftop_panorama import FrameGrabber
from go2_nav_bridge.stair_traverse_node import API_STOPMOVE, StairTraverseNode


def _env_float(name, default):
    try:
        return float(os.environ.get(name, default))
    except ValueError:
        return default


DEFAULT_WP_FILE = os.path.expanduser('~/rooftop_waypoints.yaml')
OUTPUT_DIR = os.path.expanduser(os.environ.get('TOUR_OUTPUT_DIR', '~/photos'))
MAX_LEG_M = _env_float('TOUR_MAX_LEG_M', 5.5)
# 한 구간 최대 이동 거리. 시연 최장 구간이 5.1m(P1->P2)라 약간 여유만 둠. 옥상 가장자리가
# 가까우면 줄일 것 (TOUR_MAX_LEG_M=3 등). 초과하면 이동 전에 중단.
ARRIVE_TOL_M = _env_float('TOUR_ARRIVE_TOL_M', 0.25)
MAX_CORRECTIONS = 2          # 도착 후 위치 오차가 허용치보다 크면 재접근 횟수
MIN_TURN_DEG = 3.0           # 이보다 작은 회전은 생략 (오버슈트 보정으로 더 못 줄임)
SETTLE_SEC = _env_float('TOUR_SETTLE_SEC', 1.0)
MAX_DURATION_SEC = _env_float('MISSION_MAX_DURATION_SEC', 240.0)
FORWARD_SPEED = _env_float('TOUR_FORWARD_SPEED', 0.6)
# 직진 속도 [m/s]. stair_traverse_node 기본 0.28은 계단용이라 평지 옥상에선 느림(2026-10-07
# 첫 실행에서 3.3m에 18s). 사람 조종기 시연 실측: 전진 중앙값 0.35~0.39, p90 0.62~0.92 m/s.
# 자율 실행 0.28(1차) -> 0.35 -> 0.45 -> 0.6(4차, 방향 보정 포함 완주, 잔여 오차 <0.25m).
TURN_SPEED_RAD_S = _env_float('TOUR_TURN_SPEED', 1.0)
OVERSHOOT_COMP_DEG = math.degrees(TURN_SPEED_RAD_S * 0.08)
# 회전 속도 [rad/s]와 오버슈트 보정(정지 지연 ~0.08s 가정, 파노라마와 같은 식). 4차 실행에서
# 1.0rad/s로 오버슈트 최대 약 5도 이내로 완주. 속도를 바꾸면 보정도 같이 바뀜.


def _wrap_deg(d):
    return (d + 180.0) % 360.0 - 180.0


def load_waypoints(path):
    with open(path, encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    return float(cfg.get('hold_sec', 5.0)), cfg['waypoints']


def local_to_world(start, x, y):
    """start=(x, y, yaw_rad) 기준 로컬 좌표 -> 월드(odom) 좌표."""
    c, s = math.cos(start[2]), math.sin(start[2])
    return start[0] + c * x - s * y, start[1] + s * x + c * y


def plan_leg(cur, start, wp):
    """현재 자세 cur=(x,y,yaw_rad)에서 wp로 가는 계획. 반환: (dist, turn1_deg, turn2_deg)
    turn은 왼쪽 +. turn1: 목표 방향으로, turn2: 지점 yaw로."""
    tx, ty = local_to_world(start, wp['x'], wp['y'])
    dx, dy = tx - cur[0], ty - cur[1]
    dist = math.hypot(dx, dy)
    heading = math.atan2(dy, dx)
    turn1 = _wrap_deg(math.degrees(heading - cur[2])) if dist > ARRIVE_TOL_M else 0.0
    final_yaw = start[2] + math.radians(wp['yaw_deg'])
    after = heading if dist > ARRIVE_TOL_M else cur[2]
    turn2 = _wrap_deg(math.degrees(final_yaw - after))
    return dist, turn1, turn2


def print_plan(waypoints, hold_sec):
    """로봇/ROS 없이 출발 자세=(0,0,0)에서 이상적으로 도착한다고 가정한 계획 출력."""
    start = (0.0, 0.0, 0.0)
    cur = (0.0, 0.0, 0.0)
    total = 0.0
    print(f'계획 (출발 자세 기준, 각 지점 {hold_sec:.0f}초 정지 + 사진, 구간 상한 {MAX_LEG_M:.1f}m)')
    for wp in waypoints:
        dist, t1, t2 = plan_leg(cur, start, wp)
        flag = '  <-- 상한 초과, 실행 시 중단' if dist > MAX_LEG_M else ''
        print(f"  {wp['name']}: 회전 {t1:+6.1f}° -> 전진 {dist:.2f}m -> 회전 {t2:+6.1f}° "
              f"(목표 yaw {wp['yaw_deg']:+d}°){flag}")
        total += dist
        tx, ty = local_to_world(start, wp['x'], wp['y'])
        cur = (tx, ty, start[2] + math.radians(wp['yaw_deg']))
    print(f'  총 이동 약 {total:.1f}m')


def _pose(node):
    st = node.latest_state
    return st.position[0], st.position[1], st.imu_state.rpy[2]


def _spin_until_state(node, timeout=5.0):
    end = time.time() + timeout
    while node.latest_state is None and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    return node.latest_state is not None


def _turn(node, deg_signed):
    """deg_signed: 왼쪽 +. 오버슈트 보정은 파노라마와 동일(검증된 방식)."""
    if abs(deg_signed) < MIN_TURN_DEG:
        return True
    mag = max(abs(deg_signed) - OVERSHOOT_COMP_DEG, 1.0)
    direction = node.turn_direction if deg_signed > 0 else -node.turn_direction
    return node.run_turn_segment(mag, direction)


GUARD_ENABLED = _env_float('TOUR_OBSTACLE_GUARD', 1.0) != 0.0
GUARD_STOP_M = _env_float('TOUR_GUARD_STOP_M', 0.8)
GUARD_HALF_DEG = _env_float('TOUR_GUARD_HALF_DEG', 25.0)
GUARD_SLOW_START_M = _env_float('TOUR_GUARD_SLOW_START_M', 1.3)
GUARD_MIN_SPEED = 0.25       # 감속 하한 [m/s]
GUARD_CONSECUTIVE = 3        # 연속 3회(/scan 10Hz -> 0.3초) 미만이어야 정지 (일시적 잡음 무시)
GUARD_IGNORE_SEC = 1.0       # 직진 시작 후 1초는 무시 (시작 직후 일시적 0.51m 관측 있었음)
GUARD_SCAN_MAX_AGE_SEC = 1.0
# 2026-10-07 전체 미션 첫 실행에서 HOME 복귀 중 책상에 부딪힐 뻔함. 당시 /scan 전방 +-25도 최소
# 거리는 1.55->1.26->0.94->0.64->0.43m로 줄었고, 정상 직진 구간은 항상 1.08m 이상이었음
# (자율 3개 bag 분석). 0.8m 미만이 0.3초 지속되면 정지: 사람 개입보다 약 0.8초 빠름.
# 라이다 높이 필터(scan_maker_l1 min_height 0.05~max_height 0.60m) 안에 들어오는 물체만 보임.
# 이 가드는 "정지"만 한다(회피 아님). 정지하면 투어를 중단하고 찍은 사진은 이메일로 보냄.

AVOID_ENABLED = _env_float('TOUR_AVOID', 1.0) != 0.0
AVOID_HALF_WIDTH_M = _env_float('TOUR_AVOID_HALF_WIDTH_M', 0.32)  # 로봇 반폭(약 0.15) + 여유
AVOID_BODY_BACK_M = 0.2    # 몸통 길이: 로봇 중심보다 이만큼 뒤의 점까지 통로에 포함(옆 모서리 스침 방지)
AVOID_TRIGGER_M = _env_float('TOUR_AVOID_TRIGGER_M', 0.8)  # 목표 방향 통로가 이 미만이면 회피 모드 진입
AVOID_RELEASE_M = _env_float('TOUR_AVOID_RELEASE_M', 1.3)  # 목표 방향이 이 이상 비면 회피 종료
AVOID_SELF_RANGE_M = 0.45   # 이 안쪽 점은 로봇 자기 몸(센서 앞 0.1~0.17m, 옆 0.3m에 항상 찍힘, 2026-10-07 bag)이라 무시
AVOID_LOOK_M = 1.6          # 이 거리까지만 장애물을 고려
AVOID_OK_M = _env_float('TOUR_AVOID_OK_M', 1.2)   # 이만큼 비어 있어야 그 방향을 "통과 가능"으로 봄
AVOID_MAX_DEV_M = _env_float('TOUR_AVOID_MAX_DEV_M', 1.0)  # 출발->목표 직선에서 옆으로 벗어날 최대치
AVOID_MAX_SEC = _env_float('TOUR_AVOID_MAX_SEC', 20.0)     # 한 구간에서 회피 기동을 할 최대 시간
AVOID_SPEED = _env_float('TOUR_AVOID_SPEED', 0.35)         # 회피 중 속도 상한
AVOID_PHI_RANGE_DEG = 100   # 후보 방향: 목표 방위가 아닌 로봇 정면 기준 +-100도
AVOID_SWITCH_COST = 0.3     # 직전 선택 방향과 달라질 때 벌점(떨림 방지)
# 2026-10-07: 옥상은 사람 없는 고정 환경이라 정지 대신 회피로 미션을 완주한다. /scan에서 로봇
# 폭만큼의 통로가 AVOID_OK_M 이상 비어 있는 방향 중 목표 방위에 가장 가까운 것을 골라 간다.
# **라이다는 옥상 가장자리/낮은 단차를 못 본다** -> 옆으로 벗어나는 폭(AVOID_MAX_DEV_M)과 회피
# 시간을 제한하고, 통과 가능한 방향이 없거나 한계를 넘으면 정지(투어 중단)한다.

STEER_KP = _env_float('TOUR_STEER_KP', 1.0)
STEER_MAX_Z = _env_float('TOUR_STEER_MAX_Z', 0.4)
STEER_FREEZE_M = 0.4   # 목표까지 이보다 가까우면 방위 갱신을 멈춤(근접 시 방위가 흔들림)


def scan_points(node):
    """/scan -> 로봇 좌표계 점 (xs, ys). 데이터 없음/오래됨 -> None. (scan 각도 0 = 로봇 정면)"""
    scan = node.latest_scan
    if scan is None or node.latest_scan_recv_time is None:
        return None
    age = (node.get_clock().now() - node.latest_scan_recv_time).nanoseconds / 1e9
    if age > GUARD_SCAN_MAX_AGE_SEC:
        return None
    r = np.asarray(scan.ranges, dtype=float)
    a = scan.angle_min + np.arange(len(r)) * scan.angle_increment
    ok = np.isfinite(r) & (r > max(scan.range_min, AVOID_SELF_RANGE_M))
    return r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])


def free_distance(xs, ys, phi, back=0.0):
    """로봇에서 phi(rad, 정면 기준 왼쪽 +) 방향으로 폭 2*AVOID_HALF_WIDTH_M 통로를 따라 갈 때
    처음 막히는 거리 [m]. AVOID_LOOK_M까지 비어 있으면 AVOID_LOOK_M."""
    c, sn = math.cos(phi), math.sin(phi)
    fx = xs * c + ys * sn
    fy = -xs * sn + ys * c
    m = (fx > -back) & (np.abs(fy) < AVOID_HALF_WIDTH_M) & (fx < AVOID_LOOK_M)
    return float(max(fx[m].min(), 0.0)) if m.any() else AVOID_LOOK_M


def choose_heading(xs, ys, bearing_rel, remaining, prev_phi=None):
    """목표 방위(bearing_rel, rad, 정면 기준)에 가장 가까우면서 통로가 비어 있는 방향 선택.
    반환: (phi, free_dist) 또는 통과 가능한 방향이 없으면 None."""
    need = min(AVOID_OK_M, remaining + 0.3)   # 목표가 더 가까우면 그만큼만 비면 됨
    best = None
    for deg in range(-AVOID_PHI_RANGE_DEG, AVOID_PHI_RANGE_DEG + 1, 5):
        phi = math.radians(deg)
        d = free_distance(xs, ys, phi, AVOID_BODY_BACK_M)
        if d < need:
            continue
        cost = abs(_wrap_deg(math.degrees(phi - bearing_rel)))
        if prev_phi is not None:
            cost += AVOID_SWITCH_COST * abs(math.degrees(phi - prev_phi))
        if best is None or cost < best[0]:
            best = (cost, phi, d)
    return None if best is None else (best[1], best[2])


def front_clearance(node):
    """전방 +-GUARD_HALF_DEG 내 /scan 최소 유효거리 [m]. 데이터 없음/오래됨 -> None."""
    scan = node.latest_scan
    if scan is None or node.latest_scan_recv_time is None:
        return None
    age = (node.get_clock().now() - node.latest_scan_recv_time).nanoseconds / 1e9
    if age > GUARD_SCAN_MAX_AGE_SEC:
        return None
    best = math.inf
    for i, r in enumerate(scan.ranges):
        if not math.isfinite(r) or r <= scan.range_min:
            continue
        ang = math.degrees(scan.angle_min + i * scan.angle_increment)
        if abs(_wrap_deg(ang)) <= GUARD_HALF_DEG and r < best:
            best = r
    return best


def wait_for_scan(node, timeout=3.0):
    """/scan이 들어오는지 확인(가드 사전 점검). 가드가 꺼져 있으면 항상 True."""
    if not GUARD_ENABLED:
        return True
    end = time.time() + timeout
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        if front_clearance(node) is not None:
            return True
    node.get_logger().error(
        '/scan 수신 없음 - 장애물 가드를 쓸 수 없어 시작 거부. scan_maker_l1을 먼저 켜세요 '
        '(가드 없이 진행하려면 TOUR_OBSTACLE_GUARD=0)')
    return False


class AvoidState:
    """한 번의 직진(구간) 동안 유지되는 회피 상태."""

    def __init__(self):
        self.prev_phi = None
        self.close_count = 0     # 목표 방향이 막힌 연속 횟수(회피 진입 판정) / 통과 방향 없음 연속 횟수
        self.avoiding = False
        self.avoid_sec = 0.0


def steer_step(st, bearing_rel, remaining, pts, base_speed, dt):
    """한 제어 주기의 (전진속도, yaw rate, 이벤트). 이벤트: None | 'avoiding' | 'stop'.
    평소에는 검증된 방위 보정(목표 방향으로만 조향)만 하고, 목표 방향 통로가 막힐 때만 회피
    모드로 들어간다 -> 좁은 통로/옆 벽 때문에 멀쩡한 구간에서 옆으로 틀지 않게 함.
    pts=(xs, ys) 로봇 좌표계 /scan 점. 로봇/ROS 없이 시뮬레이션/재생에서도 호출 가능."""
    xs, ys = pts
    z_goal = max(-STEER_MAX_Z, min(STEER_MAX_Z, STEER_KP * bearing_rel))
    d_goal = free_distance(xs, ys, bearing_rel)          # 목표 방향 통로(몸통 뒤쪽 제외)
    blocked = d_goal < remaining + 0.3                   # 목표가 장애물보다 앞이면 막힌 게 아님
    if not st.avoiding:
        st.close_count = st.close_count + 1 if (d_goal < AVOID_TRIGGER_M and blocked) else 0
        if st.close_count < GUARD_CONSECUTIVE:
            return base_speed, z_goal, None
        st.avoiding, st.close_count, st.prev_phi = True, 0, None
    elif d_goal >= AVOID_RELEASE_M or not blocked:       # 목표 방향이 충분히 열림 -> 복귀
        st.avoiding, st.close_count, st.prev_phi = False, 0, None
        return base_speed, z_goal, None
    plan = choose_heading(xs, ys, bearing_rel, remaining, st.prev_phi)
    if plan is None:                       # 통과 가능한 방향이 없음 -> 멈추고, 지속되면 중단
        st.close_count += 1
        return 0.0, 0.0, ('stop' if st.close_count >= GUARD_CONSECUTIVE else 'avoiding')
    st.close_count = 0
    phi, _ = plan
    st.prev_phi = phi
    st.avoid_sec += dt
    return (min(base_speed, AVOID_SPEED),
            max(-STEER_MAX_Z, min(STEER_MAX_Z, STEER_KP * phi)), 'avoiding')


def _steer_translate(node, tx, ty):
    """목표 (tx, ty)(월드)까지 이동하며 방위각 오차를 계속 보정하고, 길이 막히면 /scan에서
    비어 있는 방향으로 돌아서 간다.
    stair_traverse_node.run_translate_segment는 회전 직후 yaw를 그대로 유지만 해서
    회전 오버슈트(+2~3도)가 3~5m 직진 동안 0.2~0.5m 옆 오차로 커졌음(2026-10-07 자율
    1·2차 실행). 여기서는 현재 위치에서 목표를 향한 방위를 매 주기 다시 계산해 보정한다.
    회피: AVOID_* 상수 참고. 돌아갈 방향이 없거나 한계(옆 이탈/시간)를 넘으면 정지하고 중단.
    E-stop/타임아웃/roll 초과 처리는 run_translate_segment와 동일."""
    log = node.get_logger()
    t0 = time.time()
    start = None
    bearing = None
    close_count = 0
    speed = node.forward_speed
    st = AvoidState()
    was_avoiding = False

    def stop(reason, msg):
        log.error(msg)
        node.abort_reason = reason
        node.emergency_halt()
        return False

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.0)
        if node.check_estop() or node.check_timeout():
            node.emergency_halt()
            return False
        if node.latest_state is None:
            time.sleep(node.control_dt)
            continue
        roll = math.degrees(node.latest_state.imu_state.rpy[0])
        if node.check_roll(roll):
            log.error(f'roll 초과(steer translate 중): {roll:.1f}deg')
            node.emergency_halt()
            return False
        x, y, yaw = _pose(node)
        if start is None:
            start = (x, y)
        remaining = math.hypot(tx - x, ty - y)
        planned = math.hypot(tx - start[0], ty - start[1])
        if remaining <= ARRIVE_TOL_M:
            log.info(f'steer translate 완료: 잔여 {remaining:.2f}m ({time.time() - t0:.1f}s)'
                     + (f', 회피 {st.avoid_sec:.1f}s' if st.avoid_sec else ''))
            node.publish_move(0.0, 0.0, 0.0)
            return True
        # 회피로 돌아가면 이동 거리가 늘어나므로 상한은 회피 허용 폭만큼 더 줌
        if math.hypot(x - start[0], y - start[1]) > planned * 1.3 + 0.5 + (
                4 * AVOID_MAX_DEV_M if AVOID_ENABLED else 0.0):
            return stop('position', f'계획 {planned:.2f}m보다 과도하게 이동 - 중단 (좌표/위치 이상)')
        if bearing is None or remaining > STEER_FREEZE_M:
            bearing = math.atan2(ty - y, tx - x)
        bearing_rel = node._wrap_angle(bearing - yaw)

        if GUARD_ENABLED and AVOID_ENABLED:
            pts = scan_points(node)
            if pts is None:
                return stop('scan_lost', '/scan이 끊김(또는 오래됨) - 안전을 위해 정지')
            line = math.hypot(tx - start[0], ty - start[1]) or 1.0
            dev = abs((tx - start[0]) * (y - start[1]) - (ty - start[1]) * (x - start[0])) / line
            if dev > AVOID_MAX_DEV_M:
                return stop('avoid_limit', f'회피로 직선에서 {dev:.2f}m 벗어남 (한계 '
                            f'{AVOID_MAX_DEV_M:.1f}m) - 정지')
            speed, z_cmd, event = steer_step(
                st, bearing_rel, remaining, pts, node.forward_speed, node.control_dt)
            if st.avoid_sec > AVOID_MAX_SEC:
                return stop('avoid_limit', f'회피 {st.avoid_sec:.0f}s 초과 - 정지')
            if event == 'stop':
                return stop('obstacle', f'통과 가능한 방향 없음(전방 정지 거리 부족, 잔여 '
                            f'{remaining:.2f}m) - 정지')
            if event == 'avoiding' and not was_avoiding:
                log.info(f'장애물 회피 시작: 목표 방향 통로 막힘, 잔여 {remaining:.2f}m')
            if event != 'avoiding' and was_avoiding:
                log.info('장애물 회피 종료 - 목표 방향 복귀')
            was_avoiding = event == 'avoiding'
            node.publish_move(speed, 0.0, z_cmd)
            time.sleep(node.control_dt)
            continue

        if GUARD_ENABLED and time.time() - t0 > GUARD_IGNORE_SEC:   # 회피 끈 경우: 정지 가드만
            clear = front_clearance(node)
            if clear is None:
                return stop('scan_lost', '/scan이 끊김(또는 오래됨) - 안전을 위해 정지')
            blocked = clear < remaining + 0.3     # 목표가 장애물보다 앞이면 막힌 게 아님
            close_count = close_count + 1 if (clear < GUARD_STOP_M and blocked) else 0
            if blocked and clear < GUARD_SLOW_START_M:
                frac = (clear - GUARD_STOP_M) / (GUARD_SLOW_START_M - GUARD_STOP_M)
                speed = max(GUARD_MIN_SPEED, min(node.forward_speed, node.forward_speed * frac))
            else:
                speed = node.forward_speed
            if close_count >= GUARD_CONSECUTIVE:
                return stop('obstacle', f'전방 장애물 {clear:.2f}m < {GUARD_STOP_M:.2f}m (잔여 '
                            f'{remaining:.2f}m) - 정지')
        z_cmd = max(-STEER_MAX_Z, min(STEER_MAX_Z, STEER_KP * bearing_rel))
        node.publish_move(speed, 0.0, z_cmd)
        time.sleep(node.control_dt)
    return False


def _hold(node, seconds):
    """제자리 정지. 대기 중에도 리모컨 개입/타임아웃 확인."""
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.0)
        if node.check_estop() or node.check_timeout():
            node.emergency_halt()
            return False
        node.publish_move(0.0, 0.0, 0.0)
        time.sleep(0.05)
    return True


def _grab_photo(grabber, name, ts, log):
    jpg = None
    for _ in range(3):
        jpg = grabber.grab(timeout=2.0)
        if jpg:
            break
    if not jpg:
        log(f'사진 실패 [{name}]')
        return None
    path = os.path.join(OUTPUT_DIR, f'tour_{ts}_{name}.jpg')
    with open(path, 'wb') as f:
        f.write(jpg)
    log(f'사진 저장 [{name}]: {path} ({len(jpg)} bytes)')
    return path


def go_to_waypoint(node, start, wp):
    """현재 자세에서 wp 위치로 이동 + wp yaw로 정렬. 위치 오차가 크면 재접근."""
    log = node.get_logger().info
    for attempt in range(MAX_CORRECTIONS + 1):
        dist, t1, _ = plan_leg(_pose(node), start, wp)
        if dist <= ARRIVE_TOL_M:
            break
        if dist > MAX_LEG_M:
            node.get_logger().error(
                f"{wp['name']}: 구간 {dist:.2f}m > 상한 {MAX_LEG_M:.1f}m - 이동 안 함. "
                '출발 위치가 시연과 다른지 확인')
            return False
        log(f"[{wp['name']}] 접근 {attempt + 1}: 회전 {t1:+.1f}° -> 전진 {dist:.2f}m")
        if not _turn(node, t1):
            return False
        tx, ty = local_to_world(start, wp['x'], wp['y'])
        if not _steer_translate(node, tx, ty):
            return False
        node.publish_move(0.0, 0.0, 0.0)
        time.sleep(SETTLE_SEC)
    else:
        left, _, _ = plan_leg(_pose(node), start, wp)
        node.get_logger().warning(f"{wp['name']}: 재접근 후에도 오차 {left:.2f}m - 그대로 진행")

    _, _, t2 = plan_leg(_pose(node), start, wp)
    log(f"[{wp['name']}] 지점 yaw {wp['yaw_deg']:+d}°로 정렬: 회전 {t2:+.1f}°")
    if not _turn(node, t2):
        return False
    node.publish_move(0.0, 0.0, 0.0)
    return True


LAST_ABORT_REASON = None   # run_tour가 중단되면 'obstacle'/'scan_lost'/None(E-stop 등)


def run_tour(waypoints, hold_sec, take_photo=True):
    """rclpy init된 상태에서 실행. 반환: (ok, [(이름, 사진경로)])
    중단 원인은 모듈 변수 LAST_ABORT_REASON 참고."""
    global LAST_ABORT_REASON
    LAST_ABORT_REASON = None
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    ts = time.strftime('%Y%m%d_%H%M%S')
    node = StairTraverseNode()
    node.abort_reason = None
    node.turn_speed_rad_s = TURN_SPEED_RAD_S
    node.forward_speed = FORWARD_SPEED
    node.mission_start_time = time.time()
    node.max_duration_sec = MAX_DURATION_SEC
    log = node.get_logger().info

    grabber = executor = None
    photos = []
    ok = False
    try:
        if not _spin_until_state(node):
            node.get_logger().error('/sportmodestate 수신 없음 - 중단')
            return False, photos
        if not wait_for_scan(node):
            return False, photos
        start = _pose(node)
        log(f'출발 자세: x={start[0]:.2f} y={start[1]:.2f} yaw={math.degrees(start[2]):.0f}° '
            f'(웨이포인트 원점)')

        if take_photo:
            grabber = FrameGrabber()
            executor = SingleThreadedExecutor()
            executor.add_node(grabber)
            threading.Thread(target=executor.spin, daemon=True).start()
            for _ in range(5):   # DDS 연결 전 첫 요청 유실 대응 (I-86)
                if grabber.grab(timeout=2.0) is not None:
                    break
            else:
                node.get_logger().warning('카메라 첫 프레임 수신 실패 - 지점마다 재시도')

        for wp in waypoints:
            if not go_to_waypoint(node, start, wp):
                return False, photos
            shoot = take_photo and wp.get('photo', True)
            log(f"[{wp['name']}] 도착 - {hold_sec:.0f}초 정지" + (' + 사진' if shoot else ''))
            t0 = time.time()
            if not _hold(node, SETTLE_SEC):
                return False, photos
            if shoot:
                path = _grab_photo(grabber, wp['name'], ts, log)
                if path:
                    photos.append((wp['name'], path))
            if not _hold(node, max(0.0, hold_sec - (time.time() - t0))):
                return False, photos
        ok = True
        log('전체 촬영 완료')
    finally:
        LAST_ABORT_REASON = node.abort_reason
        node.publish_move(0.0, 0.0, 0.0)
        node.call_sport_api(API_STOPMOVE, {}, wait_response=False)
        if executor is not None:
            executor.shutdown()
        if grabber is not None:
            grabber.destroy_node()
        node.destroy_node()
    return ok, photos


def send_tour_email(photos, ok=True):
    """촬영 사진 이메일 전송 (대기열 + 재시도). 반환: (ok, error)"""
    return email_sender.send_with_retry(
        subject='[Go2] 옥상 4지점 촬영 결과',
        body='\n'.join(f'  - {n}: {os.path.basename(p)}' for n, p in photos)
        + ('' if ok else '\n※ 중간에 중단됨 (일부만 촬영)'),
        attachment_paths=[p for _, p in photos],
        max_wait_sec=_env_float('MISSION_EMAIL_RETRY_SEC', 1800.0))


def _arg(argv, flag, default=None):
    for i, tok in enumerate(argv):
        if tok == flag and i + 1 < len(argv):
            return argv[i + 1]
    return default


def main():
    argv = sys.argv[1:]
    if '--goto' in argv:
        k = argv.index('--goto')
        gx, gy = float(argv[k + 1]), float(argv[k + 2])
        hold_sec = 1.0
        waypoints = [{'name': 'GOTO', 'x': gx, 'y': gy, 'photo': False,
                      'yaw_deg': round(math.degrees(math.atan2(gy, gx)))}]
        argv = argv + ['--no-photo']
    else:
        hold_sec, waypoints = load_waypoints(
            os.path.expanduser(_arg(argv, '--wp-file', DEFAULT_WP_FILE)))
    only = _arg(argv, '--only')
    if only:
        waypoints = [w for w in waypoints if w['name'] == only]
        if not waypoints:
            raise SystemExit(f'--only {only}: 해당 이름의 웨이포인트 없음')

    print_plan(waypoints, hold_sec)
    if '--dry-run' in argv:
        print('[dry-run] 로봇은 움직이지 않음')
        return

    rclpy.init()
    try:
        ok, photos = run_tour(waypoints, hold_sec, take_photo='--no-photo' not in argv)
    finally:
        rclpy.shutdown()

    if not ok and LAST_ABORT_REASON:
        print(f'[중단 원인] {LAST_ABORT_REASON}')
    print(f"[촬영] {len(photos)}/{sum(1 for w in waypoints if w.get('photo', True))}장" + ('' if ok else ' - 중간에 중단됨'))
    if '--email' in argv and photos:
        sent, err = send_tour_email(photos, ok)
        print('[완료] 이메일 전송 성공' if sent else f'[실패] 이메일 전송 실패: {err}')
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
