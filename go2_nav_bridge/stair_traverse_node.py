#!/usr/bin/env python3
"""stair_traverse_node

구조 (D6 실측으로 확정, 도그레그 착지참):
  SpeedLevel(-1) -> FreeAvoid(false) -> ClassicWalk(true)
    -> climb(9) -> [turn(90,left) -> translate(~1m) -> turn(90,left)] -> climb(9)
    -> Move(0,0,0) -> StopMove(1003) -> ClassicWalk(false) -> BalanceStand(1002)

  (구버전: turn(180) 단일 회전으로 가정했었으나, D6 실물 시험에서 로봇이
   방금 올라온 계단 쪽을 다시 보는 것으로 확인 -> 실제 착지참은 직선 유턴이
   아니라 90도+이동+90도 도그레그 구조로 판명, 여기 맞춰 재설계함)

api_id 전부 .cpp 구현부에서 실측 확인됨 (D5, ros2_sport_client.cpp):
  Damp=1001({}) BalanceStand=1002({}) StopMove=1003({}) RecoveryStand=1006({})
  Move=1008({"x":vx,"y":0.0,"z":vyaw}) SpeedLevel=1015({"data":level})
  FreeAvoid=2048({"data":bool}) ClassicWalk=2049({"data":bool})
  AutoRecoverySet=2054({"data":bool})

파라미터 중 D5 rosbag(stair_up_151941)으로 실측된 것:
  seconds_per_step, pitch_climb_deg, pitch_level_deg, level_hold_sec,
  min_climb_margin, turn_direction, max_roll_deg, forward_speed
파라미터 중 실측 안 된 것 (기본값은 참고용 추정치):
  turn_speed_rad_s - 착지참 dyaw/dur 비율로 역산한 값, 의도적 제어값 아님
  stall_window_sec, stall_pitch_eps_deg, stall_dist_eps_m - 보수적 추정
  yaw_hold_kp, yaw_hold_max_z - D6 신규(I-56/K-34), 가설값
  landing_turn_deg(90.0), landing_move_dist_m(1.0) - D6 사용자 실측 보고 기반
    (계단 오른 직후 "왼쪽 90도 -> 약 100cm -> 왼쪽 90도" 육안 관찰)
  landing_move_speed - forward_speed 재사용, 착지참 평지에서의 검증은 안 됨
  lateral_right_angle_*_deg, lateral_hold_sign - D7 신규(우측 벽 단독 추종
    방식, 좌측=난간이라 라이다 신뢰도 낮을 것으로 판단해 dual-wall에서 변경),
    전부 가설값. lateral_right_target_m은 climb 시작 시점 우측 최소거리로
    자동 캘리브레이션(하드코딩 안 함) - "출발 시 로봇이 대략 중앙"이라는
    가정에 의존, 이 전제 자체도 미검증. lateral_hold_sign 부호도 미검증 -
    DAY_7 3단계 평지 테스트에서 로봇을 옆으로 밀어보면서 되돌리는 방향인지
    확인 후 필요시 -1.0으로 반전할 것.

실행 전 확인:
  cmd_vel_bridge가 0.4초 워치독으로 StopMove를 계속 끼워넣을 수 있으므로,
  이 노드 실행 전 cmd_vel_bridge를 반드시 내려둘 것.
"""
import time
import math
import json
from collections import deque

import rclpy
from rclpy.node import Node
from unitree_api.msg import Request, Response
from unitree_go.msg import SportModeState, WirelessController
from sensor_msgs.msg import LaserScan

# ---- api_id 상수 ----
API_DAMP = 1001
API_BALANCESTAND = 1002
API_STOPMOVE = 1003
API_RECOVERYSTAND = 1006
API_MOVE = 1008
API_SPEEDLEVEL = 1015
API_FREEAVOID = 2048
API_CLASSICWALK = 2049
API_AUTORECOVERYSET = 2054


class StairTraverseNode(Node):
    def __init__(self):
        super().__init__('stair_traverse_node')

        # ---- 파라미터 ----
        self.declare_parameter('flight_step_counts', [9, 9])
        self.declare_parameter('turn_direction', 1.0)
        self.declare_parameter('seconds_per_step', 1.14)
        self.declare_parameter('forward_speed', 0.28)
        self.declare_parameter('min_climb_margin', 7.0)
        self.declare_parameter('pitch_climb_deg', 30.0)
        self.declare_parameter('pitch_level_deg', 5.0)
        self.declare_parameter('level_hold_sec', 3.0)
        self.declare_parameter('max_roll_deg', 15.0)
        self.declare_parameter('max_duration_sec', 90.0)
        # 미검증
        self.declare_parameter('turn_speed_rad_s', 0.35)
        self.declare_parameter('stall_window_sec', 3.0)
        self.declare_parameter('stall_pitch_eps_deg', 2.0)
        self.declare_parameter('stall_dist_eps_m', 0.03)
        self.declare_parameter('control_rate_hz', 20.0)
        # 헤딩홀드 (I-56) - K-48로 climb1 실측 검증됨(yaw_err ±3.4~3.8°, 정상 완주,
        # D8(8/21) I-62 최종 확정, 코드 기본값을 검증값으로 정정)
        # D8(8/21) I-62 최종 확정, 코드 기본값을 검증값으로 정정)
        self.declare_parameter('yaw_hold_enabled', True)
        self.declare_parameter('yaw_hold_kp', 0.3)
        self.declare_parameter('yaw_hold_max_z', 0.2)
        # 횡방향(좌우) 위치 보정 - D6 신규 선언, D7 실제 구현, 전부 가설값
        self.declare_parameter('lateral_hold_enabled', True)
        self.declare_parameter('lateral_hold_kp', 0.3)
        self.declare_parameter('lateral_hold_max_vy', 0.10)
        # D7 신규: 우측 벽 단독 추종 방식 (좌측=난간, 라이다 신뢰도 낮을 것으로
        # D8(8/21) 오프라인 재분석(lidar_climb_142210, 23초, 스캔 230개) 기반으로
        # -84~-70deg(0-360 관례 276~290deg)로 축소. 유효비율/표준편차 둘 다 이 범위가
        # 최선(K-49 후속, I-65) - bag 1개 근거라 여전히 미검증, 추가 실측 필요.
        # 라이다 좌표계 기준 전방=0deg, 우측=270deg(-90deg) 근방 가정 - 미검증
        self.declare_parameter('lateral_right_angle_min_deg', 276.0)
        self.declare_parameter('lateral_right_angle_max_deg', 290.0)
        # 부호 미검증(가설) - 3단계 평지 테스트에서 확인 후 필요시 -1.0으로 반전
        self.declare_parameter('lateral_hold_sign', 1.0)
        # /scan 메시지가 이 시간(초)보다 오래되면 오래된 값으로 간주, 보정 끔(fail-safe)
        self.declare_parameter('lateral_scan_max_age_sec', 0.5)

        # 오도메트리 기반 횡방향 보정 (2026-09-18 신규, 라이다 불필요) -
        # 아래 compute_odom_lateral_correction() 참고. 기본값 False로 둬서
        # 기존에 검증된 동작(라이다 lateral_hold만 켜진 상태)을 안 건드림 -
        # 켜려면 명시적으로 True로 설정할 것.
        self.declare_parameter('odom_lateral_hold_enabled', False)
        self.declare_parameter('odom_lateral_hold_kp', 0.3)
        self.declare_parameter('odom_lateral_hold_max_vy', 0.10)
        self.declare_parameter('odom_lateral_hold_sign', 1.0)
        # 단계별 검증용 (climb 강제 조기정지)
        self.declare_parameter('climb_step_limit', 0)
        # 도그레그 착지참 (D6 신규)
        self.declare_parameter('landing_turn_deg', 88.0)
        self.declare_parameter('landing_move_dist_m', 1.03)
        self.declare_parameter('landing_move_dist_tolerance_m', 0.05)

        p = self.get_parameter
        self.flight_step_counts = p('flight_step_counts').value
        self.turn_direction = p('turn_direction').value
        self.seconds_per_step = p('seconds_per_step').value
        self.forward_speed = p('forward_speed').value
        self.min_climb_margin = p('min_climb_margin').value
        self.pitch_climb_deg = p('pitch_climb_deg').value
        self.pitch_level_deg = p('pitch_level_deg').value
        self.level_hold_sec = p('level_hold_sec').value
        self.max_roll_deg = p('max_roll_deg').value
        self.max_duration_sec = p('max_duration_sec').value
        self.turn_speed_rad_s = p('turn_speed_rad_s').value
        self.stall_window_sec = p('stall_window_sec').value
        self.stall_pitch_eps_deg = p('stall_pitch_eps_deg').value
        self.stall_dist_eps_m = p('stall_dist_eps_m').value
        self.control_dt = 1.0 / p('control_rate_hz').value
        self.yaw_hold_enabled = p('yaw_hold_enabled').value
        self.yaw_hold_kp = p('yaw_hold_kp').value
        self.yaw_hold_max_z = p('yaw_hold_max_z').value
        self.lateral_hold_enabled = p('lateral_hold_enabled').value
        self.lateral_hold_kp = p('lateral_hold_kp').value
        self.lateral_hold_max_vy = p('lateral_hold_max_vy').value
        self.lateral_right_angle_min_deg = p('lateral_right_angle_min_deg').value
        self.lateral_right_angle_max_deg = p('lateral_right_angle_max_deg').value
        self.lateral_hold_sign = p('lateral_hold_sign').value
        self.lateral_scan_max_age_sec = p('lateral_scan_max_age_sec').value
        self.odom_lateral_hold_enabled = p('odom_lateral_hold_enabled').value
        self.odom_lateral_hold_kp = p('odom_lateral_hold_kp').value
        self.odom_lateral_hold_max_vy = p('odom_lateral_hold_max_vy').value
        self.odom_lateral_hold_sign = p('odom_lateral_hold_sign').value
        self.climb_step_limit = p('climb_step_limit').value
        self.landing_turn_deg = p('landing_turn_deg').value
        self.landing_move_dist_m = p('landing_move_dist_m').value
        self.landing_move_dist_tolerance_m = p('landing_move_dist_tolerance_m').value

        self.get_logger().warn(
            'cmd_vel_bridge가 떠 있으면 0.4초 워치독이 이 노드의 명령을 끊습니다. '
            'cmd_vel_bridge를 먼저 내렸는지 확인하세요.')

        # ---- sport API 요청/응답 ----
        self.sport_req_pub = self.create_publisher(
            Request, '/api/sport/request', 10)
        self.sport_resp_sub = self.create_subscription(
            Response, '/api/sport/response', self._on_sport_response, 10)
        self._pending_id = None
        self._pending_result = None

        # ---- 상태 구독 ----
        self.latest_state = None
        self.state_sub = self.create_subscription(
            SportModeState, '/sportmodestate', self._on_state, 10)

        # ---- E-stop: 물리 리모컨 개입 감지 ----
        self.estop_triggered = False
        self.wireless_sub = self.create_subscription(
            WirelessController, '/wirelesscontroller', self._on_wireless, 10)

        # ---- 라이다 기반 횡방향(좌우) 보정 (D7 신규, 우측 벽 단독 추종) ----
        self.latest_scan = None
        self.latest_scan_recv_time = None
        self.lateral_right_target_m = None  # climb 구간마다 재캘리브레이션
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self._on_scan, 10)

        self.mission_start_time = None

    # ---------------- 콜백 ----------------

    def _on_sport_response(self, msg: Response):
        if self._pending_id is not None and msg.header.identity.id == self._pending_id:
            self._pending_result = msg

    def _on_state(self, msg: SportModeState):
        self.latest_state = msg

    def _on_wireless(self, msg: WirelessController):
        deadzone = 0.08
        stick_active = (abs(msg.lx) > deadzone or abs(msg.ly) > deadzone or
                         abs(msg.rx) > deadzone or abs(msg.ry) > deadzone)
        if stick_active or msg.keys != 0:
            if not self.estop_triggered:
                self.get_logger().error(
                    '리모컨 개입 감지 -> E-stop 트리거 (제자리 정지 후 미션 중단)')
            self.estop_triggered = True

    def _on_scan(self, msg: LaserScan):
        self.latest_scan = msg
        self.latest_scan_recv_time = self.get_clock().now()

    # ---------------- sport API 호출 ----------------

    def call_sport_api(self, api_id, parameter=None, wait_response=True, timeout=2.0):
        req = Request()
        self._pending_id = time.time_ns()
        req.header.identity.id = self._pending_id
        req.header.identity.api_id = api_id
        req.parameter = json.dumps(parameter) if parameter is not None else ''
        self._pending_result = None
        self.sport_req_pub.publish(req)

        if not wait_response:
            return 0, None

        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._pending_result is not None:
                break
        if self._pending_result is None:
            self.get_logger().warn(f'sport API {api_id} 응답 타임아웃')
            return -1, None
        code = self._pending_result.header.status.code
        data = self._pending_result.data
        return code, data

    def publish_move(self, vx, vy, vyaw):
        self.call_sport_api(API_MOVE, {'x': vx, 'y': vy, 'z': vyaw},
                             wait_response=False)

    # ---------------- 안전장치 ----------------

    def check_estop(self):
        return self.estop_triggered

    def check_roll(self, current_roll_deg):
        return abs(current_roll_deg) > self.max_roll_deg

    def check_timeout(self):
        return (time.time() - self.mission_start_time) > self.max_duration_sec

    def check_stall(self, history):
        if len(history) < 2:
            return False
        t_oldest = history[0][0]
        if (time.time() - t_oldest) < self.stall_window_sec:
            return False
        pitches = [h[1] for h in history]
        xs = [h[2] for h in history]
        ys = [h[3] for h in history]
        pitch_range = max(pitches) - min(pitches)
        dist = math.hypot(xs[-1] - xs[0], ys[-1] - ys[0])
        return pitch_range < self.stall_pitch_eps_deg and dist < self.stall_dist_eps_m

    def emergency_halt(self):
        self.get_logger().error('EMERGENCY HALT - 제자리 정지')
        self.publish_move(0.0, 0.0, 0.0)
        time.sleep(0.1)
        self.call_sport_api(API_STOPMOVE, {})

    @staticmethod
    def _wrap_angle(angle_rad):
        while angle_rad > math.pi:
            angle_rad -= 2 * math.pi
        while angle_rad < -math.pi:
            angle_rad += 2 * math.pi
        return angle_rad

    # ---------------- 라이다 기반 횡방향 보정 (D7 신규) ----------------

    @staticmethod
    def _normalize_deg(deg):
        """임의 표기(0~360 또는 -180~180)의 각도를 -180~180 범위로 정규화.

        D7 버그 수정: DAY_7.md 스펙의 우측 각도창(250~290, 0~360 관례)을
        이 로봇의 실제 /scan 각도 관례(-180~180, angle_min=-pi 실측 확인됨)와
        안 맞추고 그대로 라디안 변환해 뺄셈했더니 인덱스가 배열 밖으로 계산되어
        강제 clamp -> 항상 .inf 구간(로봇 후방)을 가리키는 버그가 있었음
        (D7 8/20 실측으로 발견, 원본 250~290deg 그대로는 우측이 아니라
        로봇 거의 정후방 근처를 가리키고 있었음).
        """
        return ((deg + 180.0) % 360.0) - 180.0

    def _lateral_right_min(self):
        """우측 각도창 내 라이다 최소 유효거리. 데이터 없음/오래됨/무효 -> None."""
        if self.latest_scan is None or self.latest_scan_recv_time is None:
            return None
        age_sec = (self.get_clock().now() - self.latest_scan_recv_time).nanoseconds / 1e9
        if age_sec > self.lateral_scan_max_age_sec:
            return None

        scan = self.latest_scan
        n = len(scan.ranges)
        if n == 0 or scan.angle_increment == 0.0:
            return None

        a_min = math.radians(self._normalize_deg(self.lateral_right_angle_min_deg))
        a_max = math.radians(self._normalize_deg(self.lateral_right_angle_max_deg))
        i_min = int((a_min - scan.angle_min) / scan.angle_increment)
        i_max = int((a_max - scan.angle_min) / scan.angle_increment)
        i_min = max(0, min(n - 1, i_min))
        i_max = max(0, min(n - 1, i_max))
        if i_min > i_max:
            i_min, i_max = i_max, i_min
        window = scan.ranges[i_min:i_max + 1]
        valid = [r for r in window
                 if math.isfinite(r) and scan.range_min < r < scan.range_max]
        return min(valid) if valid else None

    def compute_lateral_correction(self):
        """우측 벽 단독 추종 방식 횡방향 보정 (D7 재설계, 전부 가설/미검증).

        당초 좌우 벽 중앙유지(dual-wall)로 계획했으나, 실제 계단 구조가
        우측=벽 / 좌측=난간이라 좌측은 라이다 신뢰도가 낮을 것으로 판단해
        (얇은 봉 구조라 빔이 사이로 빠지거나 값이 불안정할 가능성),
        우측 벽까지 거리만으로 좌우 위치를 잡는 방식으로 변경.

        목표거리(self.lateral_right_target_m)는 하드코딩하지 않고, 이번
        climb 구간 시작 시점(run_climb_segment에서 리셋)의 첫 유효 우측
        최소거리로 자동 캘리브레이션한다 - "출발 시점엔 로봇이 대략
        중앙에 있다"는 가정에 의존 (미검증, 운용자가 매번 비슷한 위치에서
        출발시켜야 함).

        cross_track = lateral_right_target_m - 우측_최소거리
          (양수 = 목표보다 벽에 가까움 -> 벽에서 멀어지는 방향 보정 필요, 가정)
        vy_cmd = lateral_hold_sign * lateral_hold_kp * cross_track
          lateral_hold_sign 부호 자체가 미검증 - 3단계 평지 테스트에서
          로봇을 옆으로 밀어보고 vy_cmd가 되돌리는 방향인지 반드시 확인.
          반대로 나오면 lateral_hold_sign을 -1.0으로 바꿀 것.

        반환: (vy_cmd, cross_track, right_min)
          라이다 무효/캘리브레이션 전 -> (0.0, None, right_min_or_None) (fail-safe)
        """
        if not self.lateral_hold_enabled:
            return 0.0, None, None

        right_min = self._lateral_right_min()
        if right_min is None:
            return 0.0, None, None

        if self.lateral_right_target_m is None:
            self.lateral_right_target_m = right_min
            self.get_logger().info(
                f'우측 벽 목표거리 캘리브레이션: {right_min:.3f}m')
            return 0.0, 0.0, right_min

        cross_track = self.lateral_right_target_m - right_min
        vy_cmd = self.lateral_hold_sign * self.lateral_hold_kp * cross_track
        vy_cmd = max(-self.lateral_hold_max_vy, min(self.lateral_hold_max_vy, vy_cmd))
        return vy_cmd, cross_track, right_min

    # ---------------- 오도메트리 기반 횡방향 보정 (2026-09-18 신규) ----------------

    def compute_odom_lateral_correction(self, climb_start_pos, yaw_ref, pos):
        """라이다 없이, climb 시작 지점 대비 현재 위치의 횡방향(cross-track)
        벗어남만으로 vy 보정을 계산한다.

        동기: 라이다 우측벽 추종(compute_lateral_correction)은 계단 등반 중
        실측(K-49, go2_edu_plan/STATE_6.md)에서 유효 빔 비율 69.4%, 스텝 주기와
        상관된 거리값 요동(0.03~0.37m)이 확인돼 실전 투입이 보류된 상태다.
        여긴 그 문제를 아예 피한다 - sportmodestate.position은 매 climb 시작
        시점(yaw_ref/climb_start_pos, run_climb_segment에서 이미 기록하고
        있었으나 지금까지 실제로는 안 쓰이던 값)부터의 로봇 자체 추정 위치라
        라이다 시야/반사 문제가 없다. 대신 로봇 내부 상태추정기(다리
        기구학+IMU 융합 추정) 자체의 오차는 그대로 남는다 - 이 오차가
        라이다 문제보다 작은지는 아직 실기로 확인 안 됐음.

        cross_track = climb 시작 지점 기준, 진행 방향의 오른쪽으로 벗어난 거리
          (REP103: x=전방, y=좌측 기준이라 오른쪽 벗어남 = -y 성분)
        vy_cmd = odom_lateral_hold_sign * odom_lateral_hold_kp * cross_track
          부호 미검증 - compute_lateral_correction과 동일한 방식으로 평지에서
          로봇을 옆으로 밀어보고 vy_cmd가 되돌리는 방향인지 확인 후 필요시
          -1.0으로 반전할 것 (DAY_7 3단계 테스트와 같은 절차, K-43 참고).

        반환: (vy_cmd, cross_track)
        """
        if not self.odom_lateral_hold_enabled:
            return 0.0, None

        dx = pos[0] - climb_start_pos[0]
        dy = pos[1] - climb_start_pos[1]
        # yaw_ref 방향을 기준으로 진행방향 성분/오른쪽 성분으로 분해.
        # right_unit = (sin(yaw_ref), -cos(yaw_ref)) - REP103에서 전방벡터를
        # -90도(시계) 회전시킨 것 = 로봇 기준 오른쪽.
        cross_track = dx * math.sin(yaw_ref) - dy * math.cos(yaw_ref)
        vy_cmd = self.odom_lateral_hold_sign * self.odom_lateral_hold_kp * cross_track
        vy_cmd = max(-self.odom_lateral_hold_max_vy,
                     min(self.odom_lateral_hold_max_vy, vy_cmd))
        return vy_cmd, cross_track

    # ---------------- 구간 실행: climb ----------------

    def run_climb_segment(self, step_count):
        if step_count <= 0:
            self.get_logger().info('step_count<=0, climb 스킵')
            return True

        expected_duration = step_count * self.seconds_per_step
        self.get_logger().info(
            f'climb 시작: {step_count}단, 예상 {expected_duration:.1f}s')

        segment_start = time.time()
        history = deque()
        landing_hold_start = None
        landing_entry_pos = None
        yaw_ref = None
        self.lateral_right_target_m = None  # 이번 climb 구간에서 새로 캘리브레이션

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)

            if self.check_estop():
                self.emergency_halt()
                return False
            if self.check_timeout():
                self.get_logger().error('전역 타임아웃')
                self.emergency_halt()
                return False
            if self.latest_state is None:
                time.sleep(self.control_dt)
                continue

            roll = math.degrees(self.latest_state.imu_state.rpy[0])
            pitch = math.degrees(self.latest_state.imu_state.rpy[1])
            yaw = self.latest_state.imu_state.rpy[2]
            pos = self.latest_state.position

            if yaw_ref is None:
                yaw_ref = yaw
                climb_start_pos = (pos[0], pos[1])
                self.get_logger().info(
                    f'yaw 기준값 설정: {math.degrees(yaw_ref):.1f}deg')

            if self.check_roll(roll):
                self.get_logger().error(f'roll 초과: {roll:.1f}deg')
                self.emergency_halt()
                return False

            history.append((time.time(), pitch, pos[0], pos[1]))
            while history and (time.time() - history[0][0]) > self.stall_window_sec:
                history.popleft()

            elapsed = time.time() - segment_start

            if self.climb_step_limit > 0:
                hard_stop_sec = self.climb_step_limit * self.seconds_per_step
                if elapsed >= hard_stop_sec:
                    self.get_logger().info(
                        f'단계별 검증용 강제 정지: {self.climb_step_limit}단 목표, '
                        f'{elapsed:.1f}s 경과 (min_climb_margin 무시하고 정지)')
                    self.publish_move(0.0, 0.0, 0.0)
                    return True

            if elapsed >= self.min_climb_margin:
                if self.check_stall(history):
                    self.get_logger().error('stall 감지')
                    self.emergency_halt()
                    return False

                if abs(pitch) < self.pitch_level_deg:
                    if landing_hold_start is None:
                        landing_hold_start = time.time()
                        landing_entry_pos = (pos[0], pos[1])
                        self.get_logger().info(
                            f'착지참 진입 감지, 위치=({pos[0]:.2f}, {pos[1]:.2f})')
                    elif (time.time() - landing_hold_start) >= self.level_hold_sec:
                        dist = math.hypot(pos[0] - landing_entry_pos[0],
                                           pos[1] - landing_entry_pos[1])
                        self.get_logger().info(
                            f'climb 완료 ({elapsed:.1f}s, {step_count}단), '
                            f'착지참 진입 후 이동거리={dist:.2f}m')
                        self.publish_move(0.0, 0.0, 0.0)
                        return True
                else:
                    landing_hold_start = None

            if self.yaw_hold_enabled:
                yaw_error = self._wrap_angle(yaw_ref - yaw)
                z_cmd = self.yaw_hold_kp * yaw_error
                z_cmd = max(-self.yaw_hold_max_z, min(self.yaw_hold_max_z, z_cmd))
            else:
                yaw_error = 0.0
                z_cmd = 0.0

            lidar_vy, cross_track, right_min = self.compute_lateral_correction()
            odom_vy, odom_cross_track = self.compute_odom_lateral_correction(
                climb_start_pos, yaw_ref, pos)
            # 2026-09-28: mission_5f_to_rooftop.py가 climb 종료 후 도그레그 전진
            # 거리를 보정하는 데 이 값을 읽을 수 있도록 인스턴스에 남겨둠(기존엔
            # 지역변수라 함수 끝나면 사라짐 - 제어 로직/동작은 전혀 안 바뀜,
            # 값을 노출만 함).
            self.odom_cross_track = odom_cross_track
            vy_cmd = lidar_vy + odom_vy

            if cross_track is not None:
                lateral_log = (f'lidar_cross_track={cross_track:+.3f}m '
                                f'lidar_vy={lidar_vy:+.3f}m/s '
                                f'(R={right_min:.2f}m target='
                                f'{self.lateral_right_target_m:.2f}m)')
            else:
                lateral_log = 'lidar_cross_track=N/A vy=+0.000m/s'
            if odom_cross_track is not None:
                odom_log = (f'odom_cross_track={odom_cross_track:+.3f}m '
                            f'odom_vy={odom_vy:+.3f}m/s')
            else:
                odom_log = 'odom_lateral_hold=off'

            self.get_logger().info(
                f'yaw_err={math.degrees(yaw_error):+.1f}deg z_cmd={z_cmd:+.3f}rad/s '
                f'{lateral_log} {odom_log} vy_total={vy_cmd:+.3f}m/s',
                throttle_duration_sec=1.0)

            self.publish_move(self.forward_speed, vy_cmd, z_cmd)
            time.sleep(self.control_dt)

        return False

    # ---------------- 구간 실행: turn ----------------

    def run_turn_segment(self, angle_deg, direction):
        self.get_logger().info(f'turn 시작: {angle_deg}deg, direction={direction}')
        segment_start = time.time()
        cumulative_deg = 0.0
        last_yaw = None
        deadband_deg = 0.3

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)

            if self.check_estop():
                self.emergency_halt()
                return False
            if self.check_timeout():
                self.emergency_halt()
                return False
            if self.latest_state is None:
                time.sleep(self.control_dt)
                continue

            roll = math.degrees(self.latest_state.imu_state.rpy[0])
            yaw = math.degrees(self.latest_state.imu_state.rpy[2])

            if self.check_roll(roll):
                self.get_logger().error(f'roll 초과(turn 중): {roll:.1f}deg')
                self.emergency_halt()
                return False

            if last_yaw is not None:
                delta = yaw - last_yaw
                while delta > 180:
                    delta -= 360
                while delta < -180:
                    delta += 360
                if abs(delta) > deadband_deg:
                    cumulative_deg += delta
            last_yaw = yaw

            if abs(cumulative_deg) >= angle_deg:
                self.get_logger().info(
                    f'turn 완료: 누적 {cumulative_deg:.1f}deg '
                    f'({time.time()-segment_start:.1f}s)')
                self.publish_move(0.0, 0.0, 0.0)
                return True

            self.publish_move(0.0, 0.0, direction * self.turn_speed_rad_s)
            time.sleep(self.control_dt)

        return False

    # ---------------- 구간 실행: translate (D6 신규, 착지참 평지 직진이동) ----------------

    def run_translate_segment(self, distance_m):
        self.get_logger().info(f'translate 시작: 목표 {distance_m:.2f}m')
        segment_start = time.time()
        start_pos = None
        yaw_ref = None

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)

            if self.check_estop():
                self.emergency_halt()
                return False
            if self.check_timeout():
                self.emergency_halt()
                return False
            if self.latest_state is None:
                time.sleep(self.control_dt)
                continue

            roll = math.degrees(self.latest_state.imu_state.rpy[0])
            yaw = self.latest_state.imu_state.rpy[2]
            pos = self.latest_state.position

            if self.check_roll(roll):
                self.get_logger().error(f'roll 초과(translate 중): {roll:.1f}deg')
                self.emergency_halt()
                return False

            if start_pos is None:
                start_pos = (pos[0], pos[1])
                yaw_ref = yaw

            traveled = math.hypot(pos[0] - start_pos[0], pos[1] - start_pos[1])

            if traveled >= (distance_m - self.landing_move_dist_tolerance_m):
                self.get_logger().info(
                    f'translate 완료: {traveled:.2f}m '
                    f'({time.time()-segment_start:.1f}s)')
                self.publish_move(0.0, 0.0, 0.0)
                return True

            if self.yaw_hold_enabled:
                yaw_error = self._wrap_angle(yaw_ref - yaw)
                z_cmd = self.yaw_hold_kp * yaw_error
                z_cmd = max(-self.yaw_hold_max_z, min(self.yaw_hold_max_z, z_cmd))
            else:
                z_cmd = 0.0

            self.publish_move(self.forward_speed, 0.0, z_cmd)
            time.sleep(self.control_dt)

        return False

    # ---------------- 구간 실행: 도그레그 착지참 (turn->translate->turn) ----------------

    def run_landing_traverse(self, turn_deg, distance_m, direction):
        self.get_logger().info(
            f'착지참 도그레그 시작: turn {turn_deg}deg -> {distance_m:.2f}m -> turn {turn_deg}deg')
        if not self.run_turn_segment(turn_deg, direction):
            return False
        if not self.run_translate_segment(distance_m):
            return False
        if not self.run_turn_segment(turn_deg, direction):
            return False

        # D6 신규: 두 번째 회전 직후 roll이 잔류할 수 있어(회전 자체가 롤 방향
        # 불안정을 유발하는 것으로 D6 실물에서 반복 관찰됨, I-59), climb 진입
        # 전 정지 상태로 안정화 시간을 준다. 안정화 후에도 roll이 크면 여기서
        # 미리 걸러서 climb으로 안 넘어가게 함.
        self.get_logger().info('회전 후 안정화 대기 (1.5s)')
        self.publish_move(0.0, 0.0, 0.0)
        settle_start = time.time()
        while time.time() - settle_start < 1.5:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.check_estop():
                self.emergency_halt()
                return False
        if self.latest_state is not None:
            settled_roll = math.degrees(self.latest_state.imu_state.rpy[0])
            self.get_logger().info(f'안정화 후 roll={settled_roll:.1f}deg')
            if abs(settled_roll) > self.max_roll_deg * 0.6:
                self.get_logger().error(
                    f'안정화 후에도 roll 과다({settled_roll:.1f}deg) - climb 진입 보류')
                self.emergency_halt()
                return False

        self.get_logger().info('착지참 도그레그 완료')
        return True

    # ---------------- 전체 시퀀스 ----------------

    def run_sequence(self):
        self.mission_start_time = time.time()

        self.get_logger().info('AutoRecoverySet(false)')
        self.call_sport_api(API_AUTORECOVERYSET, {'data': False})

        self.call_sport_api(API_SPEEDLEVEL, {'data': -1})
        self.call_sport_api(API_FREEAVOID, {'data': False})
        code, _ = self.call_sport_api(API_CLASSICWALK, {'data': True})
        if code != 0:
            self.get_logger().error(f'ClassicWalk 진입 실패 code={code}')
            return False

        for i, step_count in enumerate(self.flight_step_counts):
            if not self.run_climb_segment(step_count):
                return False
            if i < len(self.flight_step_counts) - 1:
                if not self.run_landing_traverse(
                        self.landing_turn_deg, self.landing_move_dist_m,
                        self.turn_direction):
                    return False

        self.get_logger().info('전체 등반 완료 - 정상 종료 절차')
        self.publish_move(0.0, 0.0, 0.0)
        time.sleep(0.2)
        self.call_sport_api(API_STOPMOVE, {})
        self.call_sport_api(API_CLASSICWALK, {'data': False})
        self.call_sport_api(API_BALANCESTAND, {})
        self.call_sport_api(API_AUTORECOVERYSET, {'data': True})
        return True


def main():
    rclpy.init()
    node = StairTraverseNode()
    try:
        success = node.run_sequence()
        node.get_logger().info(f'미션 결과: {"성공" if success else "실패"}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
