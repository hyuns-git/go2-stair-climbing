#!/usr/bin/env python3
import json
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from unitree_go.msg import WirelessController
from std_srvs.srv import Trigger
from action_msgs.srv import CancelGoal
from unitree_api.msg import Request

# K-2: Go2 sport mode 최소/최대 속도 (STATE.md 기준)
MIN_VX = 0.10
MIN_VYAW = 0.18
MAX_VX = 0.36
MAX_VYAW = 0.60
WATCHDOG_SEC = 0.4

API_ID_MOVE = 1008
API_ID_STOPMOVE = 1003
API_ID_AUTORECOVERY_SET = 2054  # 실측 확인: parameter 키는 'flag'가 아니라 'data'


def clamp_with_deadzone(v, min_v, max_v):
    """0이면 0 그대로, 아니면 [min_v, max_v]로 부스트/클램프."""
    if v == 0.0:
        return 0.0
    sign = 1.0 if v > 0 else -1.0
    mag = min(max(abs(v), min_v), max_v)
    return sign * mag


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')
        self.req_id = 0
        self.pub = self.create_publisher(Request, '/api/sport/request', 10)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        # DAY_4 안전계층 #5: 조종기 개입 감지 -- 실측 결과, 펌웨어가 자동으로
        # 우선순위를 넘겨주지 않고 Nav2 명령과 계속 경합한다. 그래서 스틱/버튼
        # 입력이 감지되면 우리가 직접 E-stop과 동일한 로직을 걸어야 한다.
        self.wireless_sub = self.create_subscription(
            WirelessController, '/wirelesscontroller', self.wireless_cb, 10)
        STICK_DEADZONE = 0.08
        self.last_cmd_time = self.get_clock().now()
        self.moving = False

        # DAY_4 안전계층 #1: 소프트 E-stop 상태
        self.estop_engaged = False
        self.estop_engage_srv = self.create_service(
            Trigger, 'estop/engage', self.estop_engage_cb)
        self.estop_release_srv = self.create_service(
            Trigger, 'estop/release', self.estop_release_cb)

        # E-stop 시 진행 중이던 Nav2 목표를 명시적으로 취소하기 위한 서비스 클라이언트.
        # (cmd_vel 차단만으로는 navigate_to_pose 액션이 "대기중"으로 남아,
        #  해제 후 예전 목표를 그대로 재개할 위험이 있음)
        # action_msgs/srv/CancelGoal 규격: goal_id, stamp 둘 다 0(기본값)이면
        # 해당 액션 서버의 모든 활성 목표를 취소한다 (실측 확인됨).
        self.cancel_client = self.create_client(
            CancelGoal, '/navigate_to_pose/_action/cancel_goal')

        self.timer = self.create_timer(0.1, self.watchdog_cb)

        # DAY_4 안전계층 #6: 페이로드 실은 채 AutoRecovery=true면 넘어졌을 때
        # 격하게 일어나며 장비(Orin2+배터리+Ouster) 파손 위험 (Unitree 공식 경고)
        self.set_auto_recovery(False)

        self.get_logger().info('cmd_vel_bridge started (E-stop + AutoRecovery=false 적용됨)')

    def make_request(self, api_id, param_dict):
        req = Request()
        self.req_id += 1
        req.header.identity.id = self.req_id
        req.header.identity.api_id = api_id
        req.parameter = json.dumps(param_dict)
        return req

    def set_auto_recovery(self, flag: bool):
        req = self.make_request(API_ID_AUTORECOVERY_SET, {'data': flag})
        self.pub.publish(req)
        self.get_logger().info(f'AutoRecoverySet({flag}) 전송')

    def _engage(self, reason: str):
        if self.estop_engaged:
            return  # 이미 걸려있으면 중복 처리 안 함
        self.send_stop()
        self.estop_engaged = True
        self.moving = False

        # 진행 중이던 Nav2 목표 전부 취소 -- 이게 없으면 액션 서버가
        # "대기중"으로 남아 해제 후 예전 목표를 그대로 재개할 수 있음
        if self.cancel_client.service_is_ready():
            cancel_future = self.cancel_client.call_async(CancelGoal.Request())
            cancel_future.add_done_callback(self._cancel_done_cb)
        else:
            self.get_logger().warn('cancel_goal 서비스 응답 없음 (navigate_to_pose 미기동 가능성)')

        self.get_logger().warn(f'!!! E-STOP 발동 ({reason}) !!! 이후 /cmd_vel 전부 차단, Nav2 목표 취소 요청')

    def estop_engage_cb(self, request, response):
        self._engage('서비스 호출')
        response.success = True
        response.message = 'estop engaged'
        return response

    def wireless_cb(self, msg: WirelessController):
        stick_active = (abs(msg.lx) > 0.08 or abs(msg.ly) > 0.08
                         or abs(msg.rx) > 0.08 or abs(msg.ry) > 0.08)
        if (stick_active or msg.keys != 0) and not self.estop_engaged:
            self._engage('조종기 개입 감지')

    def _cancel_done_cb(self, future):
        try:
            result = future.result()
            self.get_logger().info(
                f'Nav2 목표 취소 완료 (취소된 목표 수: {len(result.goals_canceling)})')
        except Exception as e:
            self.get_logger().error(f'Nav2 목표 취소 실패: {e}')

    def estop_release_cb(self, request, response):
        self.estop_engaged = False
        self.get_logger().warn('E-stop 해제됨 (명시적 호출)')
        response.success = True
        response.message = 'estop released'
        return response

    def cmd_vel_cb(self, msg: Twist):
        if self.estop_engaged:
            return

        vx = clamp_with_deadzone(msg.linear.x, MIN_VX, MAX_VX)
        vyaw = clamp_with_deadzone(msg.angular.z, MIN_VYAW, MAX_VYAW)

        req = self.make_request(API_ID_MOVE, {'x': vx, 'y': 0.0, 'z': vyaw})
        self.pub.publish(req)

        self.last_cmd_time = self.get_clock().now()
        self.moving = (vx != 0.0 or vyaw != 0.0)

    def watchdog_cb(self):
        if self.estop_engaged:
            return
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if self.moving and elapsed > WATCHDOG_SEC:
            self.send_stop()
            self.moving = False
            self.get_logger().warn(f'cmd_vel 워치독 발동 ({elapsed:.2f}s) -> StopMove')

    def send_stop(self):
        req = self.make_request(API_ID_STOPMOVE, {})
        self.pub.publish(req)

    def destroy_node(self):
        self.get_logger().info('종료 -- StopMove 발행')
        try:
            self.send_stop()
            time.sleep(0.05)
        except Exception as e:
            # rclpy 컨텍스트가 이미 무효화된 상태(예: Ctrl+C 타이밍)일 수 있음.
            # 이 경우에도 프로세스가 에러로 죽지 않고 정리는 계속 진행돼야 함.
            self.get_logger().warn(f'종료 시 StopMove 발행 실패 (무시하고 계속): {e}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
