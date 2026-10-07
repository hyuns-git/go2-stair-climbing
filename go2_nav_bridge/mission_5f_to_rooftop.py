#!/usr/bin/env python3
"""mission_5f_to_rooftop — 5F 출발 -> 계단 등반 -> 옥상 도착 -> 사진 -> 이메일

HANDOFF.md 기반 상태 머신. 원칙(HANDOFF.md 그대로): 계단 등반은 "완벽"이 아니라
"일단 오른다"가 목표 — stair_traverse_node.py의 기존 K-xx 튜닝값(9+9단+도그레그,
yaw_hold_kp=0.3, 라이다 유효각 -84~-70도 등)을 그대로 재사용하고, 여기서 새 안전
검증/재시도/복구 로직을 추가하지 않는다. 실패하면 그 자리에서 멈추고 사람이
확인할 수 있게 로그만 남긴다.

트리거 방식 확인 (2026-09-11): stair_traverse_node.py는 서비스/액션 서버가
아니라 setup.py console_script(`ros2 run go2_nav_bridge stair_traverse_node`)로
직접 실행되는 단독형 노드이고, 외부에서 걸 수 있는 트리거 인터페이스(서비스/
토픽)가 없다. 여기서는 새 트리거 계층을 만들지 않고 StairTraverseNode 클래스를
그대로 import해서 그 클래스의 기존 공개 메서드(run_climb_segment/
run_landing_traverse/run_turn_segment)를 아래 5F 실측 구조 순서대로 직접
호출한다 — stair_traverse_node.py 파일 자체는 한 글자도 수정하지 않았다.

5F->옥상 실측 구조 — **2026-09-23 최종 확정 (STATE.md K-78).** 이전 두 번의
추정(2026-09-11 즉석 분석, 2026-09-18 pitch 임계값 알고리즘 재분석)은 둘 다
틀린 것으로 확인돼 폐기됨 — pitch 임계값 기반 자동 분석은 "90도 회전 ->
도그레그 전진 -> 90도 회전"처럼 두 번의 개별 회전이 있는 구간을 하나의 큰
회전으로 오판하는 방법론적 한계가 있었음. **이번엔 사용자가 Go2 공식 앱으로
실시간 카메라를 보면서 직접 눈으로 단수/회전을 셈** — 지금까지 중 가장
직접적인 관측이라 이걸 최종으로 채택한다. 실제 구조:

  1) 계단 오르기 (flight 1, 10단)
  2) 착지참 도그레그: 90도 왼쪽 회전 -> 전진 -> 90도 왼쪽 회전(다시)
     (stair_traverse_node.py의 run_landing_traverse가 정확히 이 패턴
     turn->translate->turn 이라 그대로 재사용 가능. 전진 거리는 **미실측**
     — 3F->4F 실측치(landing_move_dist_m=1.03m)를 임시로 재사용, I-79 참고)
  3) 계단 오르기 (flight 2, 9단)
  4) 옥상 문턱 앞 90도 오른쪽 단독 회전 (I-72에서 평지 단독 테스트로 이미
     실기 검증된 그대로: `run_turn_segment(90.0, -node.turn_direction)`)
  5) 계단 오르기 (flight 3, 2~3단 — 턱 넘어 옥상 도착까지 바로 이어짐,
     중간에 별도 평지 전진 구간 없음)

  회전 방향 부호: turn_direction=1.0(양수)가 "왼쪽"이라는 건 기존
  3F->4F 도그레그 검증 + K-64에서 그대로 확인됐고, "오른쪽" 90도는
  I-72에서 -turn_direction으로 평지 단독 실기 검증까지 끝남 — 이 미션은
  둘 다 기존에 검증된 부호를 그대로 재사용하므로 신규 불확실성 없음.

  **안전 관련 핵심 (K-68 그대로 재적용):** `run_climb_segment`는
  `min_climb_margin`(기본 7.0s) 경과 전까지는 착지 감지/stall 감지를 아예 안
  하고 무조건 전진만 한다 — flight 1(10단, ~10s+)은 문제 없지만 flight 3
  (2~3단, 짧음)처럼 min_climb_margin보다 짧을 수 있는 구간은 **평지 도달
  후에도 최소 7초를 채울 때까지 맹목 전진**할 위험이 있고, flight 3은 바로
  옥상 가장자리로 이어지는 구간이라 특히 위험함. 아래 구현에서는 flight 3
  호출 전후로만 `min_climb_margin`을 임시로 낮춘다(stair_traverse_node.py
  파일 자체는 수정하지 않음).

실행 전 확인 (stair_traverse_node.py와 동일):
  cmd_vel_bridge가 떠 있으면 0.4초 워치독이 명령을 끊으므로 먼저 내려둘 것.

옥상 맵/좌표: 2026-09-11 확인 결과 ~/maps/에 옥상(또는 5F 계단 진입 지점) 전용
맵이 없음(floor_3, l1_test*, office_ext01뿐). 이 미션은 Nav2/AMCL/맵을 쓰지
않고 stair_traverse_node.py와 동일하게 로봇을 계단 진입 지점에 물리적으로
배치한 뒤 여기서부터 sport API로만 진행하는 방식이라 이 미션 자체에는 맵이
필수는 아니다 — 다만 "5F 계단 진입 지점까지 자율 이동"까지 포함하려면 별도
맵/waypoint가 필요하며 현재는 없음 (STATE.md I-66 참고, 실물 매핑은 사람 작업).

사용법:
  ros2 run go2_nav_bridge mission_5f_to_rooftop
  ros2 run go2_nav_bridge mission_5f_to_rooftop --stage 2   # flight1~dogleg까지만
      (아래 STAGE_NAMES 참고. 전체를 한 번에 처음 태우지 말고 이 옵션으로
      한 구간씩 늘려가며 확인 권장.)

  MISSION_DOGLEG_ADVANCE_M=1.2 ros2 run go2_nav_bridge mission_5f_to_rooftop --stage 2
      (I-79 대응 — 도그레그 전진 거리가 미실측이라 3F->4F 값을 임시로 재사용
      중. 아래 상수들은 환경변수로 코드 수정 없이 덮어쓸 수 있음. 실측 줄자값이
      나오면 이걸로 먼저 검증한 뒤 코드 기본값을 바꿀 것.)

  ros2 run go2_nav_bridge mission_5f_to_rooftop --start-stage 3   # flight2부터
      (2026-09-28 신규 — flight1+도그레그가 재현성 있게 검증된 뒤, 매번 반복
      하지 않고 로봇을 flight2 계단 앞에 직접 놓고 그 뒤 구간만 테스트하기
      위함. --stage와 함께 쓰면 특정 구간만 실행 가능, 예:
      `--start-stage 3 --stage 4`는 flight2+접근전진+문턱회전만 실행. 로봇이
      실제로 그 지점에 있는지는 사람이 직접 확인할 것 - 코드가 검증 안 함.)
"""
import os
import sys
import time

import rclpy

from go2_nav_bridge.stair_traverse_node import (
    StairTraverseNode,
    API_AUTORECOVERYSET,
    API_SPEEDLEVEL,
    API_FREEAVOID,
    API_CLASSICWALK,
    API_STOPMOVE,
    API_BALANCESTAND,
)
from go2_nav_bridge.photo_shooter import PhotoShooterNode
from go2_nav_bridge import email_sender
from go2_nav_bridge import door_open
from go2_nav_bridge import rooftop_panorama
from go2_nav_bridge import rooftop_waypoint_tour
from go2_nav_bridge import speaker

def _env_float(name, default):
    """코드 수정 없이 실측값으로 덮어쓰기 위한 환경변수 오버라이드 (I-74/I-75 대응)."""
    v = os.environ.get(name)
    if v is None or v == '':
        return default
    return float(v)


def _env_bool(name, default):
    v = os.environ.get(name)
    if v is None or v == '':
        return default
    return v == '1'


# 2026-09-23 확정 (STATE.md K-78, 사용자 육안 실시간 카운트) 기준 — 이 미션 전용
# 상수. flight_step_counts([9,9] 기본값)는 3F->4F용이라 여기서는 재사용하지 않고
# 5F 실측치로 별도 정의한다.
FLIGHT1_STEPS = 10
FLIGHT2_STEPS = 9
FLIGHT3_STEPS = 3   # 육안 관측 "2~3단" — run_climb_segment는 step_count를 하드컷오프가
# 아니라 로그용으로만 쓰고 실제 종료는 pitch 평탄화로 적응 판단하므로 2든 3이든 동작엔 무관.

FLIGHT3_MIN_CLIMB_MARGIN_SEC = _env_float('MISSION_FLIGHT3_MIN_CLIMB_MARGIN_SEC', 2.0)
# 기본 min_climb_margin=7.0s는 이 짧은 구간(2~3단)보다 길어서 착지 감지가 아예 발동
# 못 할 수 있음 - 이 구간 호출 전후로만 임시로 낮춤 (stair_traverse_node.py 파일은
# 안 건드림). flight 3은 옥상 가장자리로 바로 이어지는 구간이라 특히 중요(K-68).

DOGLEG_TURN_DEG = _env_float('MISSION_DOGLEG_TURN_DEG', 82.0)
# K-78: 착지참에서 왼쪽 회전 -> 전진 -> 왼쪽 회전(다시). 처음엔 육안 관측대로
# 90도로 시작했으나, 실기 검증에서 세 차례에 걸쳐 조정. 매번 사용자가 조종기로
# 로봇을 flight2 정면까지 직접 돌린 각도를 yaw 값 전/후 측정으로 역산:
#   90->88도: (3F->4F landing_turn_deg=88도와 우연히 일치, run_turn_segment의
#     구조적 오버슈트 - "누적각 >= 목표"에서 멈춤 - 를 고려해 예방적으로 낮춤)
#   88->79도: 실측 누적 181.8도, 사용자가 -18도 보정(-80.6->-98.5도) -> 목표
#     누적 163.8도로 역산
#   79->82도: 실측 누적 160.8도, 사용자가 +8도 보정(-104.0->-96.0도) -> 목표
#     누적 168.8도로 역산 (79도 쪽은 살짝 부족했음)
#   두 역산값(163.8/168.8도) 평균 166.3도 기준, 오버슈트 감안해 82도로 수렴.
# **아직 실기로 82도 자체를 검증한 적은 없음 - 다음 --stage 2에서 실측 누적이
# 166도 근처로 나오는지, flight2 진입 방향이 똑바른지 확인 필수.**
DOGLEG_ADVANCE_M = _env_float('MISSION_DOGLEG_ADVANCE_M', 1.2)
# 도그레그 구간 전진 거리 - **5F 실측 안 됨(I-79)**. 2026-09-23: 평지 테스트에서
# 1.03m 실행 후 육안으로 "길다"고 판단해 0.7m로 낮췄었으나, 컨트롤러 실주행 bag
# 재분석(1.4~2.3m 추정)도 그렇고, 실제 계단에서 1.03m로 돌려보니 사용자 육안으로
# "한참 못 미침"(계단참을 다 못 건넘)으로 확인돼 1.5m로 올림. run_translate_segment는
# 폐루프 제어라 오버슈트가 구조적으로 없고(목표 도달 즉시 정지), 이 구간은 짧게
# 가면 착지참을 채 못 건너고 회전하는 게 더 위험할 수 있어 오버슈트보다 undershoot
# 쪽을 더 경계해야 함(옥상 문턱 구간과 반대 방향 논리). 2026-09-28: 1.5m(적응형
# 보정 적용 시 실제 1.32~1.40m)로도 여전히 "길다"는 사용자 육안 확인이 반복돼
# 1.15m로 낮춰서 성공적으로 검증됨(roll 0.5도, 비상정지 없음) - 이후 사용자
# 판단으로 1.2m로 살짝 다시 올림. 이것도 여전히 실측 아닌
# 실기 시행착오 값 — 실기 투입 전 줄자 실측 권장 — 실측 후엔 코드를 고치지 말고
# `MISSION_DOGLEG_ADVANCE_M=<값>` 환경변수로 먼저 검증할 것.

DOGLEG_ADAPTIVE_CORRECTION = _env_bool('MISSION_DOGLEG_ADAPTIVE_CORRECTION', True)
# 2026-09-28 (K-83/I-83 대응, 기본 꺼짐 — 켜기 전엔 기존 동작과 완전히 동일):
# 같은 DOGLEG_ADVANCE_M(1.5m)인데도 flight1 종료 시점 odom_cross_track 편차에
# 따라 한 번은 성공(seg5), 한 번은 벽에 너무 붙음(seg7, I-83)이 실기로 확인됨.
# 켜면 flight1 직후 node.odom_cross_track(2026-09-28에 stair_traverse_node.py
# 인스턴스 속성으로 노출시킴 - 그 전엔 함수 지역변수라 여기서 못 읽었음)을 읽어서
# corrected_advance = DOGLEG_ADVANCE_M + odom_cross_track 로 보정. 실기 데이터로
# 부호 교차검증됨. node.odom_cross_track이 없으면(속성이 안 남는 경우) 경고만
# 찍고 기존 고정값 DOGLEG_ADVANCE_M을 그대로 씀 — 안전한 폴백.
# **실기로 아직 한 번도 검증 안 된 신규 기능 - 켜서 쓸 때는 --stage 2로 도그레그
# 까지만 먼저 확인할 것.**

ROOFTOP_THRESHOLD_TURN_DEG = _env_float('MISSION_THRESHOLD_TURN_DEG', 87.0)
# K-78: 옥상 문턱 앞은 90도 오른쪽 단독 회전(도그레그 아님, 회전 1회뿐). I-72에서
# 평지 단독 테스트로 이미 실기 검증된 부호(-turn_direction)와 크기 그대로 재사용.

USE_TOUR = _env_bool('MISSION_TOUR', True)
# 2026-10-07: 옥상 도착 후 4지점 이동 촬영(rooftop_waypoint_tour, ~/rooftop_waypoints.yaml).
# 켜져 있으면 5방향 파노라마(MISSION_PANORAMA)는 쓰지 않음. 웨이포인트 원점은 옥상 도착
# 직후(ROOFTOP_FINAL_ADVANCE 후) 로봇 자세 - 조종기 시연 시작 자세와 같아야 하며 코드는
# 검증 안 함. 전체 미션에서 이 연결은 **실기 검증 0회.**
USE_PANORAMA = _env_bool('MISSION_PANORAMA', False)
USE_DOOR_OPEN = _env_bool('MISSION_DOOR_OPEN', True)
USE_SPEAKER = _env_bool('MISSION_SPEAKER', True)
DOOR_WAIT_SEC = _env_float('MISSION_DOOR_WAIT_SEC', 15.0)
ROOFTOP_APPROACH_ADVANCE_M = _env_float('MISSION_ROOFTOP_APPROACH_ADVANCE_M', 0.2)
# 2026-09-28 신규(사용자 요청) - flight2 완료 직후 문턱 회전 전에 몇 걸음 더
# 전진. 첫 실기(0.5m, 실측 0.46m)는 성공했으나 사용자 판단으로 "너무 많다"고
# 봐서 0.2m로 낮춤. **거리 미실측, 여전히 추정치 - 실기로 계속 조정 가능.**
ROOFTOP_FINAL_ADVANCE_M = _env_float('MISSION_ROOFTOP_FINAL_ADVANCE_M', 1.5)
# 2026-10-07: 1.0 -> 1.5. 전체 미션 첫 실행에서 4지점 투어의 HOME 복귀 중 책상에 부딪힐 뻔해
# 사람이 개입함. 투어 웨이포인트 원점이 이 전진 직후 자세라 전진을 늘리면 모든 지점이 앞으로 밀림.
# 2026-09-28 신규(사용자 요청) - flight3 완료 직후 사진 찍기 전 옥상 위로 더
# 전진(사용자 지정 약 1m). SIDE_ROOFTOP.md의 추락 경고 구간이라 값 조정 시
# 반드시 짧은 쪽(undershoot)으로 보수적으로 잡을 것. **이 구간도 실기 검증 0회.**

# 2026-09-28: FreeAvoid(API_FREEAVOID)를 평지 구간에 켜서 벽 충돌 방지에 쓸 수
# 있을지 시도했으나, 실기 테스트(test_freeavoid_translate.py, 5m 목표로 벽 향해
# 전진) 결과 **전혀 작동 안 함이 확인됨** - 켠 상태로도 그냥 벽으로 직진해서
# 사람이 리모컨으로 개입해야 했음. run_translate_segment/run_turn_segment가
# 쓰는 API_MOVE(직접 속도 명령) 경로에는 FreeAvoid가 개입하지 않는 것으로
# 결론. **이 방식은 폐기 - 다시 시도하지 말 것(K-92).** 벽 충돌 방지는 계속
# 파라미터 튜닝(거리/각도 값 조정)으로 접근할 것.

# 단계별 검증용 (전체를 한 번에 실기 투입하지 말고 --stage로 한 구간씩 늘려가며
# 확인할 것을 강력히 권장). 숫자는 climb_5f_to_rooftop() 안의 _reached() 호출과 1:1 대응.
STAGE_NAMES = [
    'flight1', 'dogleg', 'flight2', 'threshold_turn', 'flight3',
]


def _finish_sequence(node, reason):
    node.get_logger().info(f'{reason} - 종료 절차 (ClassicWalk off, AutoRecovery 복원)')
    node.publish_move(0.0, 0.0, 0.0)
    node.call_sport_api(API_STOPMOVE, {})
    node.call_sport_api(API_CLASSICWALK, {'data': False})
    node.call_sport_api(API_BALANCESTAND, {})
    node.call_sport_api(API_AUTORECOVERYSET, {'data': True})


def _hold_still(node, seconds):
    """제자리 정지 대기. 대기 중에도 리모컨 개입/타임아웃을 계속 확인."""
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.0)
        if node.check_estop() or node.check_timeout():
            node.emergency_halt()
            return False
        node.publish_move(0.0, 0.0, 0.0)
        time.sleep(0.05)
    return True


def climb_5f_to_rooftop(node: StairTraverseNode, max_stage: int = None,
                         start_stage: int = 1) -> bool:
    """5F -> 옥상 실측 구조대로 계단 등반 (stair_traverse_node.py는 수정하지 않고
    그 클래스의 기존 공개 메서드만 순서대로 호출). 각 단계 실패 시 즉시 False.

    max_stage: STAGE_NAMES 인덱스(1부터) 만큼만 실행하고 안전하게 종료.
    None이면 전 구간 실행 (기존 동작과 동일).
    start_stage: 이 인덱스보다 앞선 단계는 아예 건너뜀 (2026-09-28 신규 -
    flight1+도그레그가 안정적으로 검증됐으니 매번 반복하지 않고 flight2 계단
    앞부터 테스트하기 위함). 로봇이 이미 해당 단계 시작 지점에 물리적으로
    배치돼 있다고 가정 - 위치 확인은 사람이 직접 할 것."""
    # run_climb_segment/run_turn_segment 내부의 check_timeout()이
    # mission_start_time을 쓰는데, 원래 run_sequence()에서만 설정되던 값이라
    # 여기서 직접 초기화해줘야 함 (안 하면 None - float 뺄셈으로 바로 죽음).
    node.mission_start_time = time.time()
    # 전체 미션이 기본 90s를 넘기므로(flight1+2 ~50s + 도그레그/전진) 상향
    node.max_duration_sec = _env_float('MISSION_MAX_DURATION_SEC', 240.0)

    def reached(stage_idx):
        if max_stage is not None and stage_idx >= max_stage:
            _finish_sequence(
                node, f"[stage limit] '{STAGE_NAMES[stage_idx - 1]}'까지 완료, "
                f"max_stage={max_stage}라 여기서 정지")
            return True
        return False

    # 2026-09-23 (I-65/K-49 실기로 확인됨): 라이다 우측 벽 단독 추종
    # (lateral_hold_enabled)이 실제 5F 계단에서 왼쪽 난간 쪽으로 지속적으로
    # 미는 방향으로 오동작 - 왼쪽엔 안전 하한선이 없는 구조라 그대로 두면
    # 난간에 부딪힐 위험이 있음을 실기로 확인(사람이 리모컨으로 개입해 정지시킴).
    # stair_traverse_node.py는 수정하지 않고 인스턴스 속성만 꺼서 대응 -
    # yaw_hold만으로 진행(테스트에서 yaw_err는 계속 작았음, ±0.4~2.6도).
    node.lateral_hold_enabled = False

    # 2026-09-23 (같은 flight1 재검증 중 발견): lateral_hold를 끄고 나니 직진
    # 자체는 안정적이었으나, 착지참 진입 감지 후 기본 level_hold_sec(3.0s) 동안
    # 계속 전진하다 실제 5F 계단참이 좁아서(3F->4F보다 짧을 수 있음) 벽에 부딪힘
    # (사람이 리모컨으로 개입). run_climb_segment 코드 확인 결과 착지 감지 후에도
    # level_hold_sec 동안 계속 forward_speed로 전진하는 구조라 최대 0.28m/s*3.0s=
    # 0.84m까지 더 갈 수 있음 - 짧은 착지참엔 과함. K-68/K-69와 같은 패턴으로
    # stair_traverse_node.py는 안 건드리고 전 구간에 대해 1.0s로 줄임(여전히 가짜
    # 착지 오탐 방지용 디바운스는 유지 - 계단 중간의 순간적 pitch 평탄화는 보통
    # 1초보다 훨씬 짧음). 실기로 적정값 더 좁혀질 수 있음(I-81).
    node.level_hold_sec = 1.0

    # 2026-09-23 (I-81 이후 재검증 중 발견): lateral_hold(라이다)를 껐는데도
    # 왼쪽으로 계속 기우는 현상이 재현됨 - 라이다 보정 자체가 원인이 아니라 로봇
    # 걸음걸이/실제 계단 자체의 횡방향 편향으로 추정. yaw_hold는 헤딩만 잡을 뿐
    # 옆으로 밀리는 건 못 잡음. odom_lateral_hold(compute_odom_lateral_correction,
    # 2026-09-18 신규, 라이다 아니고 climb 시작 지점 대비 오도메트리 위치로만
    # 계산 - K-49의 라이다 신뢰도 문제를 원천적으로 피함)를 대신 켬. 부호
    # (odom_lateral_hold_sign=1.0 기본값)는 K-76에서 평지 실기로 이미 검증됨
    # ("로봇이 오른쪽으로 되돌아와 원래 직진선 복귀하는 것 육안 확인"). vy_cmd는
    # lidar_vy+odom_vy로 더해지는 구조라 lateral_hold_enabled=False와 공존 가능
    # (라이다 쪽은 0으로 기여). stair_traverse_node.py는 안 건드림. **실제 계단
    # 위에서 드리프트를 줄이는 효과 자체는 이번이 첫 실기 검증(I-76 (3) 항목).**
    node.odom_lateral_hold_enabled = True

    node.get_logger().info('AutoRecoverySet(false)')
    node.call_sport_api(API_AUTORECOVERYSET, {'data': False})
    node.call_sport_api(API_SPEEDLEVEL, {'data': -1})
    node.call_sport_api(API_FREEAVOID, {'data': False})
    code, _ = node.call_sport_api(API_CLASSICWALK, {'data': True})
    if code != 0:
        node.get_logger().error(f'ClassicWalk 진입 실패 code={code}')
        return False

    if start_stage <= 1:
        node.get_logger().info('[flight 1]')
        if not node.run_climb_segment(FLIGHT1_STEPS):
            return False
        if reached(1):
            return True
    else:
        node.get_logger().info(f'[start_stage={start_stage}] flight1 스킵 - '
                                '로봇이 이미 그 지점에 있다고 가정')

    if start_stage <= 2:
        dogleg_advance_m = DOGLEG_ADVANCE_M
        if DOGLEG_ADAPTIVE_CORRECTION:
            cross_track = getattr(node, 'odom_cross_track', None)
            if cross_track is None:
                node.get_logger().warning(
                    'DOGLEG_ADAPTIVE_CORRECTION 켜져있으나 node.odom_cross_track '
                    '없음 - 고정값 그대로 사용')
            else:
                dogleg_advance_m = DOGLEG_ADVANCE_M + cross_track
                node.get_logger().info(
                    f'도그레그 적응형 보정: odom_cross_track={cross_track:+.3f}m -> '
                    f'{DOGLEG_ADVANCE_M:.2f}m + ({cross_track:+.3f}m) = '
                    f'{dogleg_advance_m:.2f}m')

        node.get_logger().info(
            f'[착지참 도그레그 - {DOGLEG_TURN_DEG:.0f}도 왼쪽 회전 -> '
            f'{dogleg_advance_m:.2f}m 전진 -> {DOGLEG_TURN_DEG:.0f}도 왼쪽 회전]')
        if not node.run_landing_traverse(DOGLEG_TURN_DEG, dogleg_advance_m, node.turn_direction):
            return False
        if reached(2):
            return True
    else:
        node.get_logger().info(f'[start_stage={start_stage}] 도그레그 스킵 - '
                                '로봇이 이미 그 지점에 있다고 가정')

    if start_stage <= 3:
        node.get_logger().info('[flight 2]')
        if not node.run_climb_segment(FLIGHT2_STEPS):
            return False
        if reached(3):
            return True
    else:
        node.get_logger().info(f'[start_stage={start_stage}] flight2 스킵 - '
                                '로봇이 이미 그 지점에 있다고 가정')

    if start_stage <= 4:
        node.get_logger().info(
            f'[flight2 이후 접근 전진 {ROOFTOP_APPROACH_ADVANCE_M:.2f}m - '
            f'0.5m는 실기 성공(K-90), 0.2m는 미검증(K-91)]')
        if not node.run_translate_segment(ROOFTOP_APPROACH_ADVANCE_M):
            return False

        node.get_logger().info(f'[옥상 문턱 - 오른쪽 {ROOFTOP_THRESHOLD_TURN_DEG:.0f}도 단독 회전]')
        if not node.run_turn_segment(ROOFTOP_THRESHOLD_TURN_DEG, -node.turn_direction):
            return False

        if USE_DOOR_OPEN:
            node.get_logger().info(
                f'[문 열기 - {DOOR_WAIT_SEC:.0f}초 정지 후 문 열기 URL(.env) 접속]')
            if not _hold_still(node, DOOR_WAIT_SEC):
                return False
            door_ok, door_msg = door_open.open_door()
            node.get_logger().info(f'문 열기 {"성공" if door_ok else "실패(미션은 계속)"}: {door_msg}')
        if reached(4):
            return True
    else:
        node.get_logger().info(f'[start_stage={start_stage}] 접근 전진+문턱 회전 스킵 - '
                                '로봇이 이미 그 지점에 있다고 가정')

    node.get_logger().info('[flight 3 - 옥상 도착까지, min_climb_margin 임시 축소]')
    original_margin = node.min_climb_margin
    node.min_climb_margin = FLIGHT3_MIN_CLIMB_MARGIN_SEC
    try:
        flight3_ok = node.run_climb_segment(FLIGHT3_STEPS)
    finally:
        node.min_climb_margin = original_margin
    if not flight3_ok:
        return False

    node.get_logger().info(
        f'[flight3 이후 옥상 위 전진 {ROOFTOP_FINAL_ADVANCE_M:.2f}m - '
        f'신규, 실기 미검증(사용자 요청, SIDE_ROOFTOP.md 추락 경고 구간이라 '
        f'보수적으로 접근할 것)]')
    if not node.run_translate_segment(ROOFTOP_FINAL_ADVANCE_M):
        return False

    _finish_sequence(node, '전체 등반 완료')
    return True


def _build_report(climb_ok, photo_paths):
    lines = [
        f'계단 등반: {"성공" if climb_ok else "실패"}',
        f'촬영된 사진: {len(photo_paths)}장',
    ]
    for path in photo_paths:
        lines.append(f'  - {path}')
    if not photo_paths:
        lines.append('  (사진 없음 - photo_shooter 실패 또는 videohub API 미검증 문제일 수 있음)')
    return '\n'.join(lines)


def run_mission(max_stage: int = None, start_stage: int = 1):
    rclpy.init()
    photo_paths = []
    try:
        start_note = (f", start_stage={start_stage}('{STAGE_NAMES[start_stage - 1]}'부터)"
                      if start_stage > 1 else '')
        if max_stage is not None:
            print(f"[1/3] 계단 등반 시작 - 단계 제한 모드: '{STAGE_NAMES[max_stage - 1]}'까지만"
                  f'(--stage {max_stage}{start_note})')
        else:
            print('[1/3] 계단 등반 시작 (stair_traverse_node 기존 메서드 재사용, '
                  f'5F 실측 구조로 호출{start_note})')
        climb_node = StairTraverseNode()
        if USE_TOUR and not rooftop_waypoint_tour.wait_for_scan(climb_node):
            print('[중단] /scan 없음 - 옥상 이동 촬영의 장애물 가드를 쓸 수 없어 계단 등반 전에 '
                  '시작을 거부함. scan_maker_l1을 켜거나 TOUR_OBSTACLE_GUARD=0으로 실행.')
            climb_node.destroy_node()
            return False
        try:
            climb_ok = climb_5f_to_rooftop(climb_node, max_stage=max_stage, start_stage=start_stage)
        finally:
            climb_node.destroy_node()

        if not climb_ok:
            print('[중단] 계단 등반 실패 - 미션 중단, 사진/이메일 단계 진행 안 함. '
                  '사람이 현장 확인 필요.')
            return False

        if max_stage is not None:
            print(f"[완료] 단계 제한 모드 정상 종료 ('{STAGE_NAMES[max_stage - 1]}'까지). "
                  '사진/이메일은 전체 미션에서만 실행 - 여기서는 스킵.')
            return True

        print('[2/3] 옥상 도착 - 사진 촬영')
        if USE_TOUR:
            hold_sec, waypoints = rooftop_waypoint_tour.load_waypoints(
                rooftop_waypoint_tour.DEFAULT_WP_FILE)
            tour_ok, tour_photos = rooftop_waypoint_tour.run_tour(waypoints, hold_sec)
            photo_paths = [p for _, p in tour_photos]
            if not tour_ok:
                reason = rooftop_waypoint_tour.LAST_ABORT_REASON or 'E-stop/타임아웃/roll 등'
                print(f'[경고] 4지점 촬영이 중간에 중단됨({reason}) - 찍힌 것만 전송')
        elif USE_PANORAMA:
            pano_ok, pano_photos, pano_video = rooftop_panorama.capture_panorama()
            photo_paths = [p for _, p in pano_photos]
            if not pano_ok:
                print('[경고] 5방향 촬영이 중간에 중단됨 - 찍힌 것만 전송')
        else:
            photo_node = PhotoShooterNode()
            try:
                photo_paths = photo_node.capture_burst()
            finally:
                photo_node.destroy_node()

        tour_aborted = USE_TOUR and not tour_ok
        if USE_SPEAKER and tour_aborted:
            print('[음성] 4지점 촬영이 중단돼 mission complete 안내는 생략')
        elif USE_SPEAKER:
            spk_ok, spk_msg = speaker.say_mission_complete()
            print(f'[음성] mission complete 재생 {"성공" if spk_ok else "실패(미션은 계속)"}: {spk_msg}')
    finally:
        rclpy.shutdown()

    if not photo_paths:
        print('[경고] 사진 0장 - 이메일은 실패 기록만 담아 계속 전송 시도')

    print('[3/3] 이메일 전송')
    if USE_TOUR:
        ok, err = rooftop_waypoint_tour.send_tour_email(tour_photos, tour_ok)
    elif USE_PANORAMA:
        ok, err = rooftop_panorama.send_panorama_email(pano_photos, pano_video, pano_ok)
    else:
        ok, err = email_sender.send_mission_email(
            subject='[Go2] 5F -> 옥상 미션 결과',
            body=_build_report(climb_ok=True, photo_paths=photo_paths),
            attachment_paths=photo_paths,
        )
    if not ok:
        print(f'[실패] 이메일 전송 실패: {err}')
        return False

    if USE_TOUR and not tour_ok:
        print('[부분 완료] 계단 등반은 성공했으나 옥상 이동 촬영이 중단됨 (이메일은 전송됨)')
        return False

    print('[완료] 미션 정상 종료')
    return True


def _parse_stage_arg(argv, flag, default):
    """`--stage N` / `--start-stage N` 간단 파싱 (ros2 run 뒤 --ros-args 앞/뒤
    어디에 와도 되게 argv 전체를 훑음)."""
    for i, tok in enumerate(argv):
        if tok == flag and i + 1 < len(argv):
            n = int(argv[i + 1])
            if not (1 <= n <= len(STAGE_NAMES)):
                raise SystemExit(
                    f'{flag} 는 1~{len(STAGE_NAMES)} 범위 (STAGE_NAMES={STAGE_NAMES})')
            return n
    return default


def main():
    max_stage = _parse_stage_arg(sys.argv[1:], '--stage', None)
    start_stage = _parse_stage_arg(sys.argv[1:], '--start-stage', 1)
    if max_stage is not None and start_stage > max_stage:
        raise SystemExit(f'--start-stage({start_stage})가 --stage({max_stage})보다 큼')
    ok = run_mission(max_stage=max_stage, start_stage=start_stage)
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
