# STATE.md — 5F→옥상→사진→이메일 미션 (HANDOFF.md 실행 기록)

## 🏁 2026-10-02 최종 (교수님 앞 시연 완벽 성공 — 여기부터 읽을 것)

**전체 시나리오가 환경변수 없이 `ros2 run go2_nav_bridge mission_5f_to_rooftop`
한 번으로 자동 완주.** 순서: flight1(10단) → 도그레그(82도/1.2m 적응형) →
flight2(9단) → 접근 전진 0.2m → 옥상 문턱 87도 회전 → **15초 정지 후 문 열기
HTTP 접속** → flight3(3단) → 옥상 최종 전진 1.0m → **제자리 5방향 촬영(+영상)**
→ **스피커 "mission complete"** → 이메일(사진 5장 + 영상). 비상정지 0회.
마지막 실측(10/2): flight1 26.1s, 도그레그 84.1/1.09m/82.7, flight2 25.4s, 접근
0.16m, 문턱 -87.0도, flight3 8.1s, 최종 전진 0.95m, 사진 5/5.

### 코드 기본값 (전부 실기 검증됨, 아래 K-93~K-99 근거)
```
MISSION_DOGLEG_ADAPTIVE_CORRECTION = True   (기본값으로 승격)
MISSION_THRESHOLD_TURN_DEG = 87.0           (90 -> 87, 오버슈트 +2.4~4.2도 보정)
MISSION_MAX_DURATION_SEC   = 240            (stair_traverse_node 기본 90s가 전체 미션엔 부족했음)
MISSION_DOOR_OPEN = 1, MISSION_DOOR_WAIT_SEC = 15, MISSION_DOOR_URL = .env에서 읽음(인증 없는 문 열기 주소라 저장소에 안 올림)
MISSION_PANORAMA = 1  (5방향: 0.6rad/s, 오버슈트 보정 2.75도, 15fps, 1920 폭, 18MB 초과 시 자동 축소)
MISSION_SPEAKER = 1   (assets/mission_complete_16k.wav)
MISSION_EMAIL_RETRY_SEC = 1800  (이메일 대기열 ~/email_outbox, 네트워크 연결 즉시 전송)
```

### 이번 세션 신규 모듈 (go2_nav_bridge/)
- `rooftop_panorama.py` — 제자리 회전 5방향(-90,-45,0,+45,+90) 촬영 + 영상(H.264) +
  이메일. 단독 실행 `ros2 run go2_nav_bridge rooftop_panorama [--no-email|--no-video]`.
- `door_open.py` — 문 열기 HTTP GET (2xx면 성공, 실패해도 미션 계속).
- `speaker.py` — 로봇 스피커 재생(audiohub). 단독 `ros2 run go2_nav_bridge speaker`.
- `email_sender.py` 확장 — 첨부 mimetype 자동, SMTP timeout 120s, 디스크 대기열 +
  `--flush [--watch]`로 나중에 재전송.

### 이번 세션 K/I 항목 (K-93 ~ K-99, I-85 ~ I-87)
- **K-93 (타임아웃):** 90초 지점에서 `EMERGENCY HALT`가 이유 로그 없이 나와 리모컨
  개입처럼 보였지만 실제론 `check_timeout()`(max_duration_sec 기본 90). 전체 미션이
  90초를 넘김 -> mission 쪽에서 240s로 상향(stair_traverse_node는 안 건드림).
- **K-94 (문턱 회전):** 90도 명령 시 실측 -93.5/-94.2/-92.4(오버슈트 +2.4~4.2) ->
  87도로 낮추니 -87.5/-87.0. 도그레그 82도와 같은 "측정 후 역산" 방식.
- **K-95 (파노라마 회전 속도):** 0.35rad/s(기존)로 돌리면 제자리 걸음이 길어져 넘어질
  뻔함 -> 파노라마 전용 0.6rad/s(`PANO_TURN_SPEED`). 오버슈트 보정은 속도 비례
  (`rad/s*0.08` 도). 평지에서 오차 3도 이내, 옥상에선 R90이 약 5도 부족(복귀 후 7도
  틀어짐) -> 보정을 1.5도로 줄일 여지 있음(미적용). 미션의 문턱/도그레그 회전(0.35)은
  그대로(검증값).
- **K-96 (영상):** videohub는 JPEG만 줌(초당 ~180회 응답) -> 10~15fps로 throttle해
  모아 cv2로 mp4. mp4v 대신 **avc1(H.264)**가 훨씬 작고 폰/브라우저 호환. 1080p 회전
  영상은 40MB -> 18MB 한도 넘으면 자동 해상도 축소(1344x756, 17.4MB).
- **K-97 (이메일):** 영상(6.8MB+) 첨부 시 기존 SMTP timeout 15s에 걸려 `Server not
  connected`(업로드 ~0.2MB/s). timeout 120s로 상향 + 디스크 대기열/재시도 추가.
  **정정:** 이전 기록(I-68, K-63, K-88)의 "SMTP 키 미입력/.env 미생성"은 틀림 —
  `.env`는 9/11부터 Gmail SMTP로 채워져 있었고 `--check-port`/실제 전송 모두 정상.
- **K-98 (문 열기):** 서버가 **HTTP 202**(`{"result":"started"...}`)로 응답 = 성공.
  처음엔 200만 성공으로 봐서 "실패"로 오기록 -> 2xx 전체 성공으로 수정.
- **K-99 (스피커):** audiohub ROS 요청 ID — 1001 목록, 1002 재생, 1009 삭제,
  **2001 업로드**(4096B 블록 base64), **4001=ENTER_MEGAPHONE(실수로 호출했다가 4002로
  복구, 함부로 호출 금지)**. 응답 JSON 키가 **대문자**(UNIQUE_ID, CUSTOM_NAME).
  **44.1kHz wav는 code 0인데 무음, 16kHz mono wav만 재생됨.** 볼륨은 vui 1004(GET_VOLUME,
  현재 7). 로봇 내부에 업로드된 시험 파일 잔존: mission_complete(44.1k, 무음),
  mc16, mission_complete_16k — 필요시 1009로 정리.
- **I-85 (미해결):** `리모컨 개입 감지` E-stop 오탐/원인 불명. `/wirelesscontroller`에
  손 안 댔는데 `keys: 2048`이 찍힌 적 있음. 10/2에 flight1 직후·도그레그 2차 회전
  직후 각 1회 걸렸고(사용자 접촉 여부 불명), 이후 재실행은 무사. 판정(`keys != 0` 또는
  스틱>0.08)이 거칠어서 의심 -> 수정 시 stair_traverse_node `_on_wireless`만 좁힐 것.
- **I-86:** 파노라마 첫 프레임 요청이 DDS 연결 전에 유실될 수 있음(등반 직후 새 노드
  생성 시 1회 발생) -> 재시도 5회 + 사진당 3회 재시도 + 첫 프레임 실패해도 회전은 계속.
- **I-87:** 회전 중 영상 녹화(15fps/1080p) 부하로 회전 시간이 평지 시험보다 늘어남
  (90도 약 4s). 각도엔 영향 없었음.

### 다음 세션 계획 (사용자 예고, 2026-10-02)
**라이다 맵을 이용해 5층 사무실 안에서 출발 -> 계단 진입 지점까지 자율 이동 -> 이어서
등반.** 지금까지는 로봇을 계단 진입 지점에 사람이 직접 놓아야 했음. 준비물/주의:
- 맵: `config/slam_precise.yaml`(SLAM), `config/nav2_go2.yaml` + `behavior_trees/`
  (nav2)가 이미 있음 -> 5F 사무실 맵 생성부터.
- 핵심 리스크: flight1은 **진입 위치/방향 오차에 민감**(yaw_hold가 진입 시점 yaw를
  기준으로 삼고, 도그레그는 odom_cross_track으로 보정). nav2 도착 정밀도(위치/yaw)가
  계단 정면 정렬에 충분한지 먼저 측정할 것.
- 충돌 주의: `cmd_vel_bridge`(nav2 cmd_vel -> sport API)는 0.4초 워치독이 있어
  `stair_traverse_node`(직접 API_MOVE)와 같이 켜면 명령이 끊김(코드 경고문 있음) —
  이동 구간과 등반 구간의 핸드오프(브리지 내리기/ClassicWalk 전환/AutoRecovery) 설계 필요.
- 안전: 사무실 -> 계단 구간 장애물/사람, 계단 진입 직전 정렬 확인.

## ⚡ TL;DR (2026-09-28 최종, 여기부터 읽으면 됨)

**목표:** 5F 출발 → 계단 등반 → 옥상 도착 → 사진 촬영 → 이메일 전송, 전부 자동.

**진행 상황: 계단 등반 핵심 시퀀스(flight1→도그레그→flight2→문턱회전→flight3)가
실제 5F 계단에서 2회 연속 완주 성공(K-88/K-89) — 재현성 있음.** 사진 촬영도
성공(5/5장). **남은 핵심 미검증 구간은 이번 세션 후반에 사용자 요청으로 새로
추가한 두 곳** (아래 참고). *(※ 정정 10/2: 이메일 SMTP는 `.env`에 9/11부터 입력돼 있었고 정상 동작 — 위 K-97 참고.)*

핵심 설정(`mission_5f_to_rooftop.py`에 이미 반영됨, 실행 시
`MISSION_DOGLEG_ADAPTIVE_CORRECTION=1` 환경변수 필요):
```
lateral_hold_enabled = False        (K-80)
odom_lateral_hold_enabled = True    (K-82)
level_hold_sec = 1.0                (K-81)
DOGLEG_TURN_DEG = 82.0              (K-87, 실기 검증됨)
DOGLEG_ADVANCE_M = 1.2              (K-87, 실기 검증됨)
DOGLEG_ADAPTIVE_CORRECTION = True   (K-84/K-86, env로 켜야 함, 실기 검증됨)
ROOFTOP_APPROACH_ADVANCE_M = 0.2    (K-91, **미검증** - 0.5m는 검증됐으나 "길다"고 낮춤)
ROOFTOP_FINAL_ADVANCE_M = 1.0       (K-90, **미검증, 옥상 가장자리 구간**)
```
`stair_traverse_node.py`도 이번에 처음 한 줄 수정됨(K-84, odom_cross_track을
인스턴스 속성으로 노출, 동작 변화 없음). FreeAvoid로 벽 충돌 막아보려던 시도는
**실기로 확인 결과 전혀 효과 없어서 완전히 폐기함(K-92) — 다시 시도하지 말 것.**

### 다음 세션 최우선 과제
1. **`ROOFTOP_FINAL_ADVANCE_M`(1.0m, flight3 완료 후 사진 찍기 전 전진) 첫
   실기 검증** — 옥상 가장자리 바로 앞 구간이라 SIDE_ROOFTOP.md 추락 경고
   대상. 반드시 사람 감독 + 아주 짧게 나눠서(예: 0.3m부터) 확인할 것.
   `--start-stage 5`로 flight3부터 바로 테스트 가능(아래 참고).
2. `ROOFTOP_APPROACH_ADVANCE_M`(0.2m) 재검증 — `--start-stage 3 --stage 4`.
3. 위 둘 다 확정되면, 신규 구간 포함 **전체 통합(flight1부터 끝까지) 재현성**
   다시 확인.
4. 이메일 `.env` 설정 (사람이 직접).

### 지금 바로 할 일 (순서대로)
1. `ros2 topic hz /lf/sportmodestate`, `ros2 topic hz /scan`으로 로봇/라이다
   연결 확인 (로봇 재부팅되면 tmux 세션도 같이 날아감 — `scan_maker_l1`부터
   다시 띄워야 할 수 있음)
2. `cmd_vel_bridge`가 떠 있으면 반드시 먼저 내릴 것 (0.4초 워치독이 명령을 끊음)
3. **로봇을 원하는 시작 지점에 배치**
4. **`--stage`(최대 어디까지)와 `--start-stage`(어디부터, 2026-09-28 신규 K-90)로
   구간별 테스트:**
   ```bash
   cd ~/ros2_ws && source /opt/ros/humble/setup.bash && \
     source ~/unitree_ros2/setup.sh && source install/setup.bash
   MISSION_DOGLEG_ADAPTIVE_CORRECTION=1 \
     ros2 run go2_nav_bridge mission_5f_to_rooftop --stage 1   # flight1만
   # flight1+도그레그는 검증 끝났으니 flight2부터 바로 테스트하려면:
   MISSION_DOGLEG_ADAPTIVE_CORRECTION=1 \
     ros2 run go2_nav_bridge mission_5f_to_rooftop --start-stage 3
   ```
   `STAGE_NAMES = ['flight1','dogleg','flight2','threshold_turn','flight3']`
   (1~5). **주의: `--start-stage`로 건너뛴 구간은 로봇이 이미 그 지점에
   물리적으로 있다고 가정하고 실행함 - 코드가 위치를 확인해주지 않음, 사람이
   직접 확인할 것.**
5. **안전 필수**: 2인 이상, 조종기는 다른 사람이 소지, `damp` 터미널 대기,
   계단 아래/옥상 가장자리 사람 통제, AutoRecovery는 의도적으로 꺼짐(안
   되돌림) — 넘어지면 사람이 즉시 개입. 좌우(난간 쪽) 드리프트도 계속 주시.

### 이 세션 종료 시점 물리적 상태 (2026-09-28)
로봇은 마지막 테스트(`test_freeavoid_translate.py`, 평지에서 벽 향해 전진,
리모컨으로 개입시킴) 위치에 정지해 있음 — 계단이 아니라 **평지의 벽 근처**.
다음 세션 시작 전 사람이 직접 위치 확인 필요. tmux `go2` 세션(scan/rosbag/
mission/damp 4개 창)은 켜둔 채로 세션 종료 — 다음에 이어받으면 그대로 쓰거나,
로봇/Jetson 재부팅됐으면 새로 띄울 것.

### 구조 확정 (K-78, 참고용)
사용자 육안 실시간 카운트로 확정된 최종 구조 — flight1(10단) → 계단참 → 90°
왼쪽 회전 → 전진(도그레그) → 90° 왼쪽 회전(다시) → flight2(9단) → 계단참 →
90° 오른쪽 회전 → flight3(2~3단) → 옥상. 이전 버전(9+7+5단 3플라이트,
145°/178° 회전 등)은 전부 부정확했던 것으로 확인돼 폐기됨 — K-78/이 문서
맨 위 TL;DR만 참고할 것. (세부 파라미터/검증 상태는 위 TL;DR 참고 — 여기
아래는 과거 기록이라 최신 값과 다를 수 있음, K-xx 번호로 최신 값 추적할 것.)

### 파일 위치
- 미션 코드: `~/ros2_ws/src/go2_nav_bridge/go2_nav_bridge/mission_5f_to_rooftop.py`
- 계단 등반 공용 로직: 같은 폴더 `stair_traverse_node.py` (2026-09-28에 처음
  한 줄 수정됨, K-84 참고 — 그 전까진 절대 안 건드렸었음)
- 이 문서 전체 히스토리(K-57~K-92, I-66~I-84): 아래 계속
- 최초 지시서: `HANDOFF.md` (같은 폴더)
- 재분석 스크립트: `~/analyze_stair_recon_5f.py`
- 근거 rosbag(9/11): `~/stair_recon_5f_141622`, `~/stair_recon_5f_142457`
- **2026-09-23 실기 테스트 rosbag**: `~/mission_5f_manual_run5_*`(K-80 라이다
  오동작) ~ `run6_*`(K-81/K-82/K-83, level_hold_sec/odom_lateral_hold 디버깅)
- **2026-09-28 실기 테스트 rosbag**: `run7_*`(31분, K-85~K-88 도그레그 각도
  튜닝+첫 전체 성공) / `run8_*`(9분, K-89 재현성 확인) / `run9_*`(36분, K-90
  접근전진 성공 + K-91/92 FreeAvoid 실패 테스트). **정리/검증 결과**:
  `~/go2_tools/today_rosbag_summary_20260928.md` — bag별 climb 구간을 실제
  pitch 데이터로 검출해서 K-번호와 대응시켜둠(눈대중 아니고 실측 교차검증).
- **테스트/유틸 스크립트**: `~/go2_tools/` — `record_run.sh`(rosbag 자동
  번호 기록), `test_freeavoid_translate.py`(2026-09-28 신규, FreeAvoid 단독
  검증용), `emergency_damp.sh`, 그 외 이전 세션들의 bag 분석/시뮬레이션 관련
  스크립트들

---

> 이 문서는 `go2_edu_plan/STATE_6.md`(D7, 2026-08-20 종료 시점, K-56/I-65까지)의
> K-xx/I-xx 번호를 이어서 사용한다. `go2_edu_plan/` 쪽 문서들은 8/21(D8,
> day8_turn_check) 이후 갱신되지 않은 채 3주 넘게 방치돼 있었고, 그 사이
> 홈 디렉터리에 `rooftop_manual_*`, `analyze_rooftop_bag.py`,
> `gate_sequence_test.py`, `select_mode_ai.py` 등 미문서화 작업 흔적이 남아있음
> (2026-08-12~08-21 사이, STATE_6.md에는 반영 안 됨) — 참고만 하고 이번 세션
> 범위 밖이라 건드리지 않았음.

## 세션 개요 (2026-09-11)

`HANDOFF.md`의 "자동으로 진행할 것" 1~7항을 실행. 결과 요약: 코드 스켈레톤
4개 작성 + colcon build 통과 + import 테스트 통과. **로봇 실기 검증은 전혀
하지 않았음** (사람이 로봇을 계단 앞에 세우고 감독하며 실행해야 함).

## K (확정된 사실)

- **K-57 (신규):** `unitree_sdk2py`가 이 시스템에 설치돼 있지 않음
  (`import unitree_sdk2py` → `ModuleNotFoundError`, 전역 python3에서 확인,
  `find /` 로도 어디에도 없음). VideoClient/GetImageSample() 직접 호출 경로
  자체가 없음 — `photo_shooter.py`는 `stair_traverse_node.py`가 이미 쓰는
  unitree_api Request/Response ROS2 브릿지 방식(`/api/videohub/request`,
  api_id=1001)으로 작성함. **이 토픽명/api_id는 실기 미검증** — Unitree
  공식 프로토콜 문서 기준값 + `unitree_ros2/example`의 다른 클라이언트 명명
  규칙(`/api/<service>/request`, sport/motion_switcher/arm/voice 확인됨)
  유추일 뿐, 이 저장소엔 videohub 클라이언트 예제가 없음. → I-67.
- **K-58 (신규):** `stair_traverse_node.py`는 서비스/액션 서버가 아니라
  `setup.py` console_script로 직접 실행되는 단독형 노드(`main()`이 곧바로
  `run_sequence()` 호출) — 외부에서 걸 수 있는 트리거 인터페이스(서비스/토픽)
  자체가 없음. `mission_5f_to_rooftop.py`는 이 사실에 맞춰 새 트리거 계층을
  만들지 않고 `StairTraverseNode` 클래스를 그대로 import해서
  `run_sequence()`를 직접 호출하는 방식으로 통합함 (리턴값 True/False =
  완료/실패 신호). HANDOFF.md 지시대로 새 안전/재시도 로직 추가 안 함.
- **K-59 (신규):** `~/maps/`에 옥상(또는 5F 계단 진입 지점) 전용 맵 없음 —
  `floor_3`(빈 디렉터리), `l1_test01~03`, `office_ext01*`뿐. `waypoints.yaml`에
  있는 waypoint(`office_start`, `stair_entry`, `4f_entry_point` 등)도 이번
  미션 대상 층/구간과 이름이 안 맞음 → I-66.
- **K-60 (신규):** colcon build (`--packages-select go2_nav_bridge
  --symlink-install`) 통과, 4개 신규 모듈(`photo_shooter`, `email_sender`,
  `mission_5f_to_rooftop`, 및 `.env.example`) import 테스트 통과. 신규
  의존성 추가 없음(stdlib `smtplib`/`email`/`socket`만 사용, `python-dotenv`도
  미설치라 `.env` 파서는 최소 구현으로 직접 작성).
- **K-61 (신규):** `email_sender.py`는 `.env` 필수 키(SMTP_HOST/PORT/USER/
  PASSWORD, MAIL_FROM/TO)가 비어있으면 실제 네트워크 연결을 시도하지 않고
  바로 에러를 반환함을 확인 (`--check-port` 실행 결과로 검증). 현재
  `.env` 자체가 없음(`.env.example`만 있음, 값은 전부 빈칸) — 사람이
  IT팀 문의 후 직접 입력해야 함(HANDOFF.md 2항, Claude Code가 대신 입력 안 함).

## I (열린 이슈)

- **I-66 (신규, 사람 작업):** 옥상(또는 5F 계단 진입 지점) 맵/waypoint 없음 —
  실제 로봇 매핑 필요. 자동화 불가로 판단해 이번 세션에서는 시도 안 함.
- **I-67 (신규, 최우선 실기 검증 대상):** `photo_shooter.py`의 videohub
  토픽명/api_id=1001이 실제 로봇에서 존재/동작하는지 미검증. 로봇 연결 후
  `ros2 topic list | grep videohub` 먼저 확인, 안 나오면 `/frontvideostream`
  (Go2FrontVideoData, H.264 디코딩 필요, 대안 경로)으로 전환 검토.
- **I-68 (신규, 사람 작업 대기):** SMTP host/port/계정 정보 전부 없음(`.env`
  값 비어있음) — IT팀 문의 필요(HANDOFF.md 1항). 포트 오픈 여부 테스트
  (`nc -zv`/`email_sender.py --check-port`)도 host를 몰라 아직 실행 못 함.
  Orin1 iptables(DOCKER-USER 체인) SMTP 포트 확인도 host 확정 후 진행.
- **I-69 (신규):** `mission_5f_to_rooftop.py`가 재사용하는
  `stair_traverse_node.py`의 기본 파라미터(9+9단+도그레그 등)는 원래 **3F→4F**
  실측 기반(K-33~K-56 참고)이다. HANDOFF.md는 "기존 계단 그대로 재사용"을
  명시했지만, 5F 계단이 실제로 이 파라미터와 같은 구조(단수/단높이/착지참
  형태)인지는 이번 세션에서 확인하지 않았음 — 실기 투입 전 반드시 확인 필요.
- **I-70 (신규):** `mission_5f_to_rooftop.py`는 촬영 실패(0장)여도 이메일은
  "사진 없음" 기록과 함께 계속 전송하도록 만듦(HANDOFF.md에 명시된 요구사항은
  아니고, "실패 시 사람에게 알리는 수준"을 이메일 채널로 확장한 판단). 계단
  등반 자체가 실패한 경우는 이메일을 보내지 않고 콘솔 로그만 남기도록
  최소화함 — 이 비대칭이 맞는 설계인지 사람 확인 필요.

## 세션 개요 2 (2026-09-11, 같은 날 이어서)

Orin1/Orin2 네트워크 구성 정정 + 5F 계단 실측(사용자 수동 주행 2회 + rosbag) +
`mission_5f_to_rooftop.py` 계단 구조 코드 반영.

- **K-62 (신규):** 이 기기(`nvidia-desktop` = Orin2)에 `wlP1p1s0`(PCIe/M.2
  무선랜카드로 추정, USB 아님) 인터페이스로 자체 와이파이가 연결돼 있고
  8.8.8.8 핑 성공 — Orin2가 이제 Orin1을 거치지 않고 직접 인터넷에 붙음을
  확인함. `go2_edu_plan/STATE_6.md`의 K-39(Orin2 캐리어보드 USB 허브 결함,
  무선동글은 Orin1 유지 필수)는 USB 동글 기준 결론이었고, M.2 카드는 그
  결함 경로를 우회하므로 상충하지 않음. HANDOFF.md의 "Orin1 iptables"는
  이 정정 이전 가정 — 이메일 아웃바운드는 이제 Orin2 자체 iptables만
  확인하면 될 가능성이 높음(sudo 권한 없어 이번 세션에서 직접 확인은
  못 함). *(go2_edu_plan/STATE_6.md 자체는 이번 세션 범위 밖이라 수정 안 함.)*
- **K-63 (신규):** 이메일을 회사 SMTP 대신 Gmail SMTP로 보내는 방향 논의됨
  (IT팀 문의 불필요, 아웃바운드 587 포트 이슈 회피). `email_sender.py`는
  이미 host/port 무관 범용 구현이라 **코드 변경 없이 `.env`에 Gmail 값만
  넣으면 동작**(Gmail 앱 비밀번호 필요, 2단계 인증 활성화 전제). 사용자가
  이메일 설정은 나중으로 미룸 — `.env`는 아직 미생성 상태 유지.
- **K-64 (신규, I-69 해결):** 사용자가 컨트롤러로 5F->옥상 계단을 2회
  수동 주행하며 rosbag(`/scan`, `/sportmodestate`, `/wirelesscontroller`)
  기록 (`~/stair_recon_5f_141622`, `~/stair_recon_5f_142457`). 두 bag 모두
  `metadata.yaml` 누락 상태로 발견됐으나(`ros2 bag reindex`로 복구 성공,
  데이터 유실 없음 확인) `/scan`은 두 번 다 0개 — MobaXterm/와이파이 세션이
  녹화 중 끊기면서 `ros2 bag record` 프로세스가 비정상 종료된 것으로 추정
  (foreground 실행, SIGHUP 추정). `/scan` 부재는 이와 무관 — 로봇 내장
  유선망(enP8p1s0) 경로라 Orin2 와이파이와 별개, LiDAR 드라이버 미기동
  추정. → I-71.
  `/sportmodestate`(pitch/yaw/position) 분석 + 사용자 육안 확인으로 구조
  추정 시도 (이때 나온 단수/각도는 이후 틀린 것으로 확인돼 삭제됨 —
  최종 구조는 K-78 참고).
- **K-65 (신규):** 위 구조를 `mission_5f_to_rooftop.py`에 반영함
  (`climb_5f_to_rooftop()` 신규 함수). **`stair_traverse_node.py`는 한
  글자도 수정하지 않았고**, 그 클래스의 기존 공개 메서드(`run_climb_segment`,
  `run_landing_traverse`, `run_turn_segment`, `call_sport_api`,
  `publish_move`)만 5F 실측 순서대로 호출함 — `run_sequence()`의
  전제(2 flight + 도그레그 1회 고정)가 5F 구조와 안 맞아 `run_sequence()`
  대신 직접 오케스트레이션. `run_climb_segment`/`run_turn_segment`의 종료
  판정은 원래부터 step_count/angle_deg를 하드컷오프가 아니라 pitch
  평탄화·yaw 누적·stall 감지로 적응적으로 판단하므로(`climb_step_limit=0`
  기본값 확인함), 문턱 구간 step_count=4는 로그 표시용일 뿐 실제 안전
  로직에는 영향 없음 — HANDOFF.md의 "새 안전/재시도 로직 추가 금지"에
  위배되지 않는다고 판단함. colcon build + import 테스트 통과 확인.
  → I-72 (실기 미검증).

## I (열린 이슈, 추가)

- **I-71 (신규):** `/scan` 두 테스트 모두 미기록. 다음 실측 전 로봇 연결
  후 `ros2 topic hz /scan`으로 LiDAR 드라이버 기동 여부 먼저 확인 필요.
  `ros2 bag record`는 `tmux`/`screen` 안에서 실행해 SSH 끊김에 안전하게
  할 것(K-64 참고).
- **I-72 (부분 해결, 2026-09-11):** `climb_5f_to_rooftop()`의 옥상 문턱
  회전(`-node.turn_direction`) 방향을 평지에서 단독 실기 테스트함
  (`run_turn_segment(90.0, -node.turn_direction)`만 분리 실행,
  AutoRecoverySet/ClassicWalk 등 계단 진입 절차는 안 건드림). 결과:
  `누적 -91.4deg (12.7s)`로 정상 종료, 사용자가 실제로 오른쪽으로 돈 것
  육안 확인함 — **회전 부호는 확정.** (참고: 첫 시도는 Claude Code가
  외부에서 `timeout 20`으로 감쌌다가 마무리 시점에 프로세스가 잘려서
  "실패/중단"으로 보였음 — 코드/로봇 문제가 아니라 테스트 하니스 문제였음,
  timeout 없이 재실행하니 정상 완료.)
  **남은 부분(실기 미검증):** flight1 -> U턴 -> flight2 -> 이번에 확인한
  90도 회전 -> 문턱 climb까지 이어지는 **전체 시퀀스 통합 실행**은 아직
  한 번도 안 해봄. U턴 도그레그(`run_landing_traverse`)와 climb 자체는
  기존 3F->4F에서 이미 실기 검증된 동작이라 우선순위는 낮지만, AutoRecovery
  꺼진 상태로 실제 5F 계단에서 처음 통합 실행할 때는 HANDOFF.md대로 사람이
  현장에서 감독할 것.

## 세션 개요 3 (2026-09-18, 재분석)

사용자 요청: "그때 rosbag로 컨트롤러 이용해서 내가 가려는 코스 녹화한거 있는데
그거 다시 한번 제대로 분석해서 해보자" — 9/11의 즉석 분석(스크립트로 안 남음,
K-64)을 재현 가능한 스크립트로 다시 검증.

- **K-66 (신규):** `~/analyze_stair_recon_5f.py` 작성, 두 회차
  (`stair_recon_5f_141622`, `stair_recon_5f_142457`) 모두 `stair_traverse_node.py`와
  동일한 pitch 기준(pitch_climb_deg=30/pitch_level_deg=5)으로 재세그먼트해서
  K-64의 즉석 추정을 검증하려 시도함 (이때 나온 구체적 단수/각도는 이후
  또 틀린 것으로 확인돼 삭제됨 — 최종 구조는 K-78 참고). **방법론적으로는
  유효한 교훈**: pitch 임계값 기반 자동 세그먼트만으로는 "90도 회전 ->
  도그레그 전진 -> 90도 회전"처럼 두 번의 개별 회전이 있는 구간을
  하나의 순net 회전으로만 측정해서 실제 구조(K-78)를 오판하게 만들 수
  있음. 전 구간 pitch는 모든 CLIMB 세그먼트에서 시종일관 음수만 관찰됨
  — 하강 구간은 전혀 없음(SIDE_ROOFTOP.md 원안의 "문턱 오르고 내림" 가정은
  이 5F 코스엔 해당 안 됨, 순수 상행).
- **K-67 (신규):** `gait_type` 필드가 두 회차 전체(77556개/67504개 샘플)에서
  단 한 번도 0에서 안 바뀜 — SIDE_ROOFTOP.md가 제안한 "1 trot -> 3 climb stair
  -> 4 forwardDownStair" 게이트 전환 신호는 이 로봇/펌웨어에서 실제로 관찰되지
  않음. 다행히 `stair_traverse_node.py`는 원래부터 gait_type이 아니라 pitch만
  쓰고 있어 영향 없음 — 참고용 기록만.
- **K-68 (신규, 안전 관련 핵심):** `run_climb_segment`의 `min_climb_margin`
  (기본 7.0s)은 그 값 도달 전까지 착지 감지·stall 감지를 아예 수행하지 않고
  무조건 전진만 함을 코드 확인(`stair_traverse_node.py:412` `if elapsed >=
  self.min_climb_margin:` 이하에서만 착지/stall 체크). flight3(~5s)·
  flight4/threshold(~2~3s)는 이 7.0s보다 짧아 **실제 도달 후에도 최소 7초를
  채울 때까지 맹목 전진**하게 됨 — flight4는 옥상 문턱 바로 다음이라
  SIDE_ROOFTOP.md의 추락 경고와 직결되는 문제.
- **K-69 (신규, K-68 대응):** `mission_5f_to_rooftop.py`의
  `climb_5f_to_rooftop()`을 재작성함 — `stair_traverse_node.py`는 여전히 한
  글자도 안 건드림. (1) flight2->flight3 사이 신규 평지 이동은
  `run_translate_segment(5.0)`(거리기반, 오차 0.05m)로 추가 — 이동거리는
  두 회차 path-length 실측(5.18m/7.31m) 중 더 짧은 값보다도 낮게 잡음(초과주행
  위험 회피 우선, **실기 전 줄자 실측 권장, I-74**). (2) flight3은
  `run_climb_segment(5)` 호출 전후로만 `node.min_climb_margin`을 2.0s로 임시
  변경(파일 수정 없이 인스턴스 속성만 일시 override, 호출 후 원복). (3)
  flight4/threshold는 `run_climb_segment` 대신 `run_translate_segment(0.75)`로
  교체 — 옥상 가장자리 방향 맹목 전진 위험을 거리 기반 정확 정지로 대체.
  (4) 문턱 회전 크기 90도 -> 145.0도로 정정(부호는 기존 `-turn_direction` 그대로
  맞았음, 크기만 틀렸음). colcon build + import 테스트 통과 확인.

## I (열린 이슈, 추가 2)

- **I-73/I-74/I-75: K-78로 구조 자체가 바뀌면서 폐기됨.** (구 flight1~4 +
  145도 회전 + `CORRIDOR_ADVANCE_M` 5~7m 복도 가정 전부 무효. K-78의
  "90도 회전→도그레그 전진→90도 회전" 구조에 맞게 `mission_5f_to_rooftop.py`
  다시 작성 필요 — 도그레그 전진 거리는 미실측, 새 I 항목으로 다시 추적할 것.)

## 세션 개요 4 (2026-09-18, 같은 날 이어서 — 사용자 요청: "확실히 제대로 분석한 거 맞냐, 완벽하게 만들어")

사용자가 강화학습/시뮬레이션까지 써서 "완벽하게" 만들라고 요청함. 이 요청에 대한
판단을 먼저 기록: **이 문제엔 RL/시뮬레이션이 안 맞는다고 판단하고 사용자에게
그렇게 설명함** — 이 5층 건물의 CAD/물리 시뮬레이터가 없고(새로 만드는 게 실측보다
훨씬 오래 걸림), 막힌 부분(복도 길이 I-74, 문턱-가장자리 거리 I-75)은 학습으로
알아낼 값이 아니라 실측이 필요한 물리적 사실이며, 로봇 보행 자체는 이미 Unitree
펌웨어가 처리해서(K-67) 우리 쪽에 학습 대상이 없음. 대신 아래처럼 **실제로
검증 강도를 높일 수 있는 것들을 추가로 함**.

- **K-70 (신규):** 홈 디렉터리에 있던 추가 rosbag 2개(`rooftop_manual_144315`,
  `rooftop_manual_152420`, 둘 다 8/12 기록 — go2_edu_plan 쪽 STATE.md에 이미
  "미문서화, 범위 밖"으로 기록돼 있던 것들)를 같은 스크립트로 재확인함.
  `rooftop_manual_144315`는 17분/climb 6구간/회전 4번, `rooftop_manual_152420`는
  3분/climb 3구간/회전 1번 — 9/11 두 회차(climb 4구간/회전 2번, 서로 구조 일치)와
  구조가 전혀 다름. **사용자에게 직접 확인한 결과 "다른 경로였다(무시)"로 확정** —
  현재 미션(K-66~K-69)의 근거 데이터는 여전히 `stair_recon_5f_141622`/`142457`
  두 회차만이고, 이번 재확인으로 다른 후보 데이터가 없다는 것까지 확정됨.
- **K-71 (신규, K-66 강화):** pitch 임계값 민감도 재검증 — pitch_climb_deg를
  20/25/30/35도로, pitch_level_deg를 3/5/8도로 바꿔가며 재세그먼트한 결과
  **25~30도 구간에서는 pitch_level_deg 값과 무관하게 climb 4구간으로 안정적으로
  재현됨.** 20도에서는 5구간으로 늘어나는데, 실제로 뜯어보니 corridor 구간
  중 1.09초짜리 pitch 20.2도 블립(노이즈, 실제 계단 아님) 하나가 추가로
  잡힌 것 — 4구간 결론을 반증하지 않음. 35도에서는 1구간으로 붕괴하는데,
  이는 flight2/3/4의 최대 pitch(30.2~34.8도)가 35도에 못 미쳐서 대부분
  안 잡히는 것 — 임계값이 실측 pitch 범위 자체를 벗어난 경우라 당연한 현상.
  **부수 발견:** `pitch_climb_deg`는 `stair_traverse_node.py`에 선언만 되고
  `run_climb_segment` 안에서 실제로 안 쓰이는 죽은 파라미터임을 코드로 확인함
  (착지 판정은 `pitch_level_deg=5도`만 사용, `elapsed >= min_climb_margin`
  이후). 즉 flight3의 최대 pitch(30.2~30.7도, 30도 임계값에 근접)가 실제
  로봇 동작에 영향 주지 않음 — 이 우려는 기각.
- **K-72 (신규, 시도했으나 결론 없음, 투명성 위해 기록):** vx/foot_force
  진동 주기로 스텝수를 독립적으로 재확인해보려 했으나, raw vx의 zero-crossing이
  샘플 노이즈에 압도돼(10초 구간에 150~300회, 명백히 비물리적) 신뢰할 수 있는
  신호를 못 얻음. foot_force[0]는 전 구간에서 변화 없음(필드 미사용 추정).
  **이 접근은 폐기 — 스텝수 추정은 여전히 dur/seconds_per_step(1.14s) 근사가
  유일한 근거이고, 이는 3F->4F 계단에서 캘리브레이션된 값이라 5F 계단
  단높이/단너비가 다르면 절대 스텝수는 부정확할 수 있음.** 다만 실제 종료
  판정(K-65 참고)이 스텝수가 아니라 pitch 평탄화 기반 적응형이라 이 부정확성이
  로봇 동작 자체에 영향을 주지는 않음 — 로그상 표시값 정확도 문제로 한정됨.
- **K-73 (신규, 대응 코드):** `mission_5f_to_rooftop.py`에 단계 제한 실행
  기능 추가 — `--stage N`(1=flight1 ... 7=threshold_advance, `STAGE_NAMES` 참고)
  로 한 구간만 실행하고 안전 종료 절차(ClassicWalk off, AutoRecoverySet true)
  까지 밟은 뒤 멈춤. SIDE_ROOFTOP.md의 V-0~V-8 단계적 검증 방식과 같은
  원리 — 전체를 한 번에 실기 투입하지 말고 이걸로 한 구간씩 늘려가며 확인할
  것을 권장(I-73 대응). 또한 `CORRIDOR_ADVANCE_M`/`ROOFTOP_THRESHOLD_ADVANCE_M`/
  `ROOFTOP_THRESHOLD_TURN_DEG`를 환경변수(`MISSION_CORRIDOR_ADVANCE_M` 등)로
  코드 수정 없이 덮어쓸 수 있게 함 — 실측(I-74/I-75) 나오면 이 값으로 먼저
  검증 후 코드 기본값을 바꿀 것. colcon build + import 테스트 통과, `--stage`
  파싱/범위 검증/env override 단위 테스트 통과 확인.

## 세션 개요 5 (2026-09-18, 같은 날 이어서 — 라이다 대신 오도메트리 기반 횡방향 보정)

사용자 질문: 라이다 위치보정이 이미 있었는지(있음, 위 참고), rosbag에 라이다 값이
있는지(없음, I-71), 그리고 "라이다 아니어도 계단 오를 때 위치 똑바로 유지하는
매우 좋은 방법"을 요청함. **이번엔 HANDOFF.md 원칙(`stair_traverse_node.py`를
건드리지 않는다)에서 처음으로 벗어남** — 사용자가 명시적으로 이 파일의 핵심
로직(횡방향 보정)에 대한 개선을 요청했기 때문. 기본값을 전부 False로 둬서
기존 3F->4F/5F 검증된 동작(라이다 lateral_hold만 켜진 상태)은 전혀 안 바뀜.

- **K-74 (신규):** `run_climb_segment` 안에 `climb_start_pos`가 이미 캘리브레이션
  기준점으로 기록되고 있었는데(yaw_ref와 같은 시점에 세팅) **실제로는 어디에도
  안 쓰이던 죽은 변수**였음을 발견 - 이 지점이 오도메트리 기반 횡방향 보정에
  정확히 필요한 기준점이라 재사용함.
- **K-75 (신규, 구현):** `compute_odom_lateral_correction()` 신규 추가 -
  `sportmodestate.position`(다리 기구학+IMU 융합 추정, 라이다 불필요)만으로
  climb 시작 지점 기준 진행방향에 대한 횡방향 벗어남(cross_track)을 계산해서
  `vy_cmd`를 만든다. 신규 파라미터 4개(`odom_lateral_hold_enabled`(기본 False),
  `odom_lateral_hold_kp`(0.3), `odom_lateral_hold_max_vy`(0.10),
  `odom_lateral_hold_sign`(1.0, 미검증)) 전부 기존 라이다 lateral_hold와
  분리된 별도 값. `run_climb_segment`에서 라이다 vy + 오도메트리 vy를 더해서
  최종 vy_cmd로 씀(오도메트리 쪽이 기본 꺼져 있으니 지금은 합쳐도 라이다
  단독과 결과 동일). 로그도 `lidar_cross_track`/`odom_cross_track`/`vy_total`
  로 분리해서 찍음. **동기:** 라이다 우측벽 추종은 계단 등반 중 실측(K-49)에서
  유효빔비율 69.4%·스텝주기 상관 거리 요동이 확인돼 실전 투입이 보류된 상태였고
  (I-65), 5F 계단은 그 검증조차 안 된 상태(I-71)라 - 오도메트리는 라이다의
  시야/반사 문제를 원천적으로 피하지만, 대신 로봇 자체 상태추정기의 오차가
  그대로 남는다는 게 트레이드오프. **이게 실제로 더 나은지는 아직 실기로
  확인 안 됨 - 코드는 준비됐고 검증은 안 된 상태.**
- **부호 검증(코드 레벨만, 실기 아님):** yaw_ref를 0/90도로 바꿔가며 좌/우
  이동을 넣어 단위 테스트 - 오른쪽으로 벗어나면 cross_track 양수 -> vy_cmd
  양수(=왼쪽으로 보정, 라이다 쪽 K-43 부호 관례와 일치) 확인함. **다만 이건
  좌표 변환 공식이 맞다는 확인일 뿐, `odom_lateral_hold_sign=1.0`이 실제
  로봇에서도 맞는 방향인지는 여전히 미검증** — compute_lateral_correction의
  K-43과 똑같이 평지에서 로봇을 옆으로 밀어보는 테스트로 먼저 확인할 것.
  colcon build + import 테스트(`mission_5f_to_rooftop.py` 포함) 통과.

## 세션 개요 6 (2026-09-18, 같은 날 이어서 — I-76 1단계 실기 검증)

`~/odom_lateral_sign_test.py` 신규 작성(파일 관리 목적, `stair_traverse_node.py`는
안 건드림) — `lateral_hold_enabled=False`/`odom_lateral_hold_enabled=True`로
인스턴스 속성만 override하고 `run_climb_segment(1)`을 평지에서 직접 호출,
`climb_step_limit`로 강제 정지 시간 제한(K-69와 같은 override 방식).

- **K-76 (신규, I-76 1단계 해결):** 평지 실기로 `odom_lateral_hold_sign=1.0`
  부호 확인 완료. 사용자가 로봇 기준 **왼쪽**으로 밀었고, 로그상
  `odom_cross_track`이 음수로 내려감(코드 정의상 음수=왼쪽 벗어남, 정의와
  일치) → `vy_cmd`도 음수(로봇 기준 오른쪽 보정)로 산출됨 → **실제로
  로봇이 오른쪽으로 되돌아와 원래 직진선을 복귀하고 계속 전진하는 것을
  사용자가 육안 확인함.** 부호 반전 불필요, 현재 기본값(1.0) 그대로 유지.
  (참고: 첫 시도(`--step-limit 5`, 5.7s)는 `min_climb_margin`(7.0s) 도달 전에
  강제 정지돼 착지판정 로직이 아예 발동 안 했고, 밀기 타이밍도 로그 없이
  즉흥적이라 애매했음 - 재시도 시 4~9s 구간에 "지금 밀어주세요"/"손 떼세요"
  타이밍 신호를 별도 스레드로 실시간 출력하도록 스크립트 보강.)
- **K-77 (신규, 발견):** 평지에서는 `run_climb_segment`가 pitch<5deg 조건이
  항상 참이라, `elapsed>=min_climb_margin(7.0s)` 도달 즉시 착지판정 타이머가
  시작되고 `level_hold_sec(3.0s)` 후 **항상 정확히 10.0s에 종료**됨 -
  `climb_step_limit`을 그보다 크게(예: 14, 16s) 줘도 소용없음(10s에 이미
  끝나버림). 밀기->손떼기->복귀 관찰까지 여유 있게 보려면 평지 테스트 시
  `min_climb_margin`도 같이 임시로 크게(예: 30s) override해서 착지판정
  자체를 안 걸리게 해야 함 - 이번엔 그렇게 안 해서 놓은 직후(9s) 바로
  종료돼(10.0s) 복귀 구간 로그는 못 건졌음(사용자 육안 관찰로만 확인됨).
- **K-78 (신규, 2026-09-23, 계단 구조 최종 확정):** 사용자가 Go2 공식 앱으로
  실시간 카메라를 보면서 직접 눈으로 단수를 셈 — 알고리즘(pitch 임계값)
  추정이 아닌 직접 관측이라 지금까지 중 가장 신뢰도 높음. 확정 구조:
  **flight1(10단) → 계단참 → 90° 왼쪽 회전 → 전진(도그레그) → 90° 왼쪽
  회전(다시) → flight2(9단) → 계단참 → 90° 오른쪽 회전 → flight3(2~3단)
  → 옥상.** 이전 pitch-임계값 기반 자동 세그먼트 방식이 반복해서 틀렸던
  이유: climb 구간 사이 순net yaw 변화만 측정해서 "90도+도그레그+90도"를
  하나의 큰 회전으로 오판함 — 방법론 자체의 한계. **이 구조를 최종으로
  보고 이후 parametric 계단 지형(MuJoCo 등) 구축에 사용할 것.** 참고:
  이번 재확인 중 Go2 공식 앱의 실시간 SLAM이 ROS2 `/uslam/*` 토픽으로는
  전혀 안 잡힘을 별도 확인함(→ I-78).
- **K-79 (신규, 2026-09-23, I-79 대응):** `mission_5f_to_rooftop.py`를 K-78
  구조로 전면 재작성. `stair_traverse_node.py`는 이번에도 한 글자도 안
  건드림 — `run_landing_traverse(turn_deg, distance_m, direction)`가
  turn->translate->turn 패턴 그대로라 도그레그(90도+전진+90도)에 딱 맞아서
  재사용, 옥상 문턱 90도 회전은 `run_turn_segment(90.0, -node.turn_direction)`
  그대로(I-72에서 이미 실기 검증된 부호). 구 CORRIDOR_ADVANCE_M/
  ROOFTOP_THRESHOLD_ADVANCE_M 상수와 그 값을 쓰던 `run_translate_segment`
  호출 2곳은 전부 삭제(K-78 구조엔 해당 없음). `STAGE_NAMES`도
  `['flight1','dogleg','flight2','threshold_turn','flight3']`(1~5)로
  축소. flight3(2~3단)에 대한 K-68 안전 대응(min_climb_margin 임시 축소)은
  그대로 유지. colcon build + import 테스트 통과, **실기 투입은 아직 안 함**.
- **K-80 (신규, 2026-09-23, 중요 — I-65/K-49 실기로 확인됨):** K-78 구조로
  재작성한 `mission_5f_to_rooftop.py`로 5F flight1 실기 테스트 중,
  `lateral_hold_enabled`(라이다 우측 벽 단독 추종, D7 신규 기능)가
  **왼쪽 난간 쪽으로 지속적으로 미는 방향으로 오동작** — 사람이 리모컨으로
  긴급 개입해 정지시킴(왼쪽 난간 충돌 직전). bag(`mission_5f_manual_run5_
  20260923_101015`) 분석 결과 roll이 t=0~6s 동안 안정적(-3.9도)이다가
  t=6.5s부터 급격히 요동(최대 +3.53도, max_roll_deg=15.0도라 자동 안전
  cutoff는 발동 안 했을 상황)치기 시작했고, 같은 구간에서
  `lidar_cross_track`이 +0.24~0.31m로 벌어짐(R=0.33~0.40m가 target=0.65m보다
  계속 작아서 "오른쪽에 너무 붙었다"고 판단, 왼쪽=난간 쪽으로 계속 보정
  명령 발행). **원인 추정**: 이 기능은 "출발 시 로봇이 대략 중앙"이라는
  가정으로 초반 라이다 최솟값을 자동 캘리브레이션하는데(코드 주석
  `우측 벽 목표거리 캘리브레이션` 참고), 이 가정이 틀리면 왼쪽엔 안전
  하한선이 전혀 없는 구조(코드 주석에 "좌측=난간, 라이다 신뢰도 낮을
  것으로 [예상]"이라 아예 감시 안 함). **대응**: `stair_traverse_node.py`는
  수정하지 않고, `mission_5f_to_rooftop.py`에서 `node.lateral_hold_enabled
  = False`로 전체 미션에 대해 꺼버림(K-68/K-69와 같은 인스턴스 속성
  오버라이드 패턴) — yaw_hold만으로 진행(이번 테스트에서 yaw_err는 계속
  ±0.4~2.6도로 작았음). colcon build 통과, **이 변경 이후 실기 재검증
  아직 안 함**.
- **K-81 (신규, 2026-09-23, K-80 이후 같은 flight1 재검증 중 발견):**
  `lateral_hold_enabled=False`(K-80) 적용 후 재시도 - 직진(yaw_hold)은 안정적
  이었으나(yaw_err 계속 ±0.2~4.5도), **착지참 진입 감지 후에도 계속 전진하다
  벽에 부딪힘** (사람이 리모컨 긴급 개입, 벽 충돌 직전/직후). `run_climb_segment`
  코드 확인 결과 착지 감지(pitch<pitch_level_deg) 후 `level_hold_sec`(기본
  3.0s) 동안 forward_speed(0.28m/s)로 계속 전진하며 "진짜 착지참 맞는지"
  재확인하는 구조 - 최대 0.84m까지 더 갈 수 있음(중간에 pitch가 다시 커지면
  타이머 리셋). 실제 5F 계단참이 이보다 짧은 것으로 추정(3F->4F보다 좁을
  가능성). **대응**: `stair_traverse_node.py`는 안 건드리고
  `mission_5f_to_rooftop.py`에서 `node.level_hold_sec = 1.0`으로 전
  구간(flight1~3)에 대해 낮춤(K-68/K-69/K-80과 같은 오버라이드 패턴) -
  가짜 착지 오탐 방지 디바운스는 유지하되 최대 전진거리를 0.84m->약 0.28m
  수준으로 줄임. colcon build 통과, **이 변경 이후 실기 재검증 아직 안 함**.
- **K-82 (신규, 2026-09-23, I-76 (3) 대응, 중요):** `level_hold_sec=1.0`(K-81)
  적용 후 재검증 - 착지참 벽 충돌 재현 안 됐으나(K-81 문제는 해결된 듯),
  **왼쪽으로 기우는 현상이 다시 나타남**(라이다는 이미 꺼져있는 상태였음 -
  즉 라이다 보정 자체가 원인이 아니라 로봇 걸음걸이/실제 계단의 횡방향
  편향으로 추정). yaw_hold는 헤딩(방향)만 잡을 뿐 옆으로 밀리는 위치
  자체는 못 잡음. **대응**: `odom_lateral_hold`(2026-09-18 신규,
  climb 시작 지점 대비 오도메트리 위치로만 횡방향 보정 계산 - 라이다 안
  씀, K-49의 라이다 신뢰도 문제 원천 회피)를 켬. 부호(`odom_lateral_hold_sign`
  기본값 1.0)는 K-76에서 평지 실기로 이미 검증됨. `vy_cmd = lidar_vy +
  odom_vy`로 더해지는 구조라 `lateral_hold_enabled=False`와 공존 가능(라이다
  기여분 0). `stair_traverse_node.py`는 안 건드리고
  `node.odom_lateral_hold_enabled = True`만 오버라이드. colcon build 통과,
  **실제 계단 위에서 드리프트를 줄이는 효과 자체는 이번이 첫 실기 검증
  (I-76의 (3)항목) — 아직 결과 확인 전**.
- **K-83 (신규, 2026-09-23, I-76 (3) 결과 — 성공):** K-82 적용 후 재검증
  결과 `odom_lateral_hold`가 실제로 효과 있음을 확인 — flight1 완주 2회
  성공(odom_cross_track 계속 ±0.1m 이내), 도그레그(90도+1.5m+90도)도 roll
  0.4~0.6도로 안정적으로 2회 성공. **`DOGLEG_ADVANCE_M`도 1.03m가 실제
  착지참을 못 건널 만큼 부족함을 실기로 확인, 1.5m로 올려서 재검증 성공**
  (1.47~1.48m 실이동, 목표와 거의 일치). 다만 **`DOGLEG_ADVANCE_M`을 고정
  거리로 두는 방식 자체의 한계 발견**: 전체 시나리오 통합 실행에서는 flight1의
  odom_cross_track이 -0.11m까지 벌어진 채로 도그레그에 진입했고(단독
  stage2 테스트보다 드리프트 큼), 같은 1.5m를 전진했는데도 이번엔 벽에
  너무 가까워짐(사용자 육안 확인, 두 번째 회전 시작 직후 리모컨 개입)
  — flight1 종료 시점의 실제 위치/자세가 매번 조금씩 다르므로, 고정
  거리로는 재현성이 떨어질 수 있음. → I-83.
- **I-76 (부분 해결, 2026-09-18 → K-76):** `odom_lateral_hold_sign` 부호는
  평지 실기로 확인 완료(K-76). **남은 부분:** (2) 짧은 계단(예: flight3,
  min_climb_margin 이미 낮춰둔 구간)에서 **라이다 lateral_hold는 끄고
  이것만 켜서** 단독 효과 확인, (3) 되면 실제 드리프트가 줄어드는지 켜기
  전/후 비교. 지금은 기본 꺼짐 상태라 아무것도 실행에 영향 없음. 다음
  평지 재검증(복귀 구간 로그 확보용) 시 K-77 참고해서 `min_climb_margin`도
  같이 늘릴 것.
- **I-77 (신규, 근본 원인 미확정):** go2_edu_plan/STATE_6.md I-57 자체가
  "헤딩(yaw)은 정상인데 위치가 중앙이 아니다"로 기록돼 있었고, 3F->4F에서
  마지막으로 확인된 "드리프트 없이 완주"(K-48)는 오히려 라이다 보정을 끈
  상태였음. 즉 **횡방향 보정(라이다든 오도메트리든)이 드리프트의 실제
  해법인지 자체가 아직 확정 안 됐다** — 진입 헤딩 정렬 오차, 보행 자체의
  좌우 비대칭 등 다른 원인일 가능성도 열어둘 것. I-76 검증 시 "보정 끈
  상태에서도 이번엔 드리프트가 있었는지"부터 먼저 확인 권장.
- **I-78 (신규, 2026-09-23, 막다른 길 확정):** Go2 공식 앱의 실시간 SLAM은
  ROS2 `/uslam/*` 토픽으로 안 나옴 — 앱 켠 채 111초 실제 이동까지 하며
  녹화했는데도 전부 0개 메시지(`mission_5f_manual_run2_20260923_091718`
  bag으로 확인). WebRTC 등 ROS2/DDS 밖의 별도 채널로 추정. **이 경로로
  계단 3D 복원 시도하지 말 것** — K-78의 직접 관측 구조 + 표준/실측 치수로
  parametric 지형을 만드는 쪽이 유일한 실용적 경로.
- **I-79 (부분 해결, 2026-09-23 → K-79):** `mission_5f_to_rooftop.py`를
  K-78 구조로 재작성 완료(colcon build + import 테스트 통과, K-79 참고).
  **남은 부분:** 도그레그 전진 거리(`DOGLEG_ADVANCE_M`, 기본 1.03m)가
  3F->4F 값을 임시로 재사용 중이라 5F 실측 아님 — 실기 투입 전 줄자 실측
  권장. 전체 시퀀스 실기 투입 자체도 아직 한 번도 안 해봄(--stage로
  한 구간씩 확인할 것).
- **I-80 (부분 해결, 2026-09-23 → K-81):** `lateral_hold_enabled=False`(K-80)
  적용 후 재검증 결과 **직진 자체(yaw_hold)는 안정적으로 확인됨** — 왼쪽
  난간 드리프트 문제는 재현 안 됨. 근본 원인(캘리브레이션 로직 자체 결함인지
  이 계단 폭/형상 문제인지)은 여전히 미확정이지만, 이번 5F 코스에서는 계속
  꺼둘 것을 권장. **남은 부분**: 같은 재검증 중 다른 문제(착지참 진입 후
  벽 충돌, K-81)가 새로 발견돼 그쪽을 마저 고치는 중 — flight1 완주까지는
  아직 성공 못 함.
- **I-81 (부분 해결, 2026-09-23 → K-82):** `level_hold_sec=1.0`(K-81) 적용
  후 재검증한 시도에서 벽 충돌 재현은 안 됨(왼쪽 드리프트 문제(K-82)로
  다시 중단돼 완전히 끝까지 확인된 건 아님 — 계속 지켜볼 것). 1.0s가 실제
  5F 계단참들 크기에 맞는지도 여전히 불확실(추정치).
- **I-82 (해결, 2026-09-23 → K-83):** `odom_lateral_hold_enabled = True`
  실기 검증 완료 — 효과 있음 확인(K-83). I-76의 (3)항목도 이걸로 해결됨.
- **I-83 (해결, 2026-09-28 → K-86):** `DOGLEG_ADVANCE_M` 고정 거리 방식의
  재현성 문제, 적응형 보정(`MISSION_DOGLEG_ADAPTIVE_CORRECTION=1`, K-84)으로
  해결 확인(K-86, 지금까지 가장 큰 편차 -0.139m 상황에서 검증 성공). 앞으로
  이 플래그를 기본으로 켜고 진행할 것. **여전히 남은 것**: flight2/
  threshold_turn/flight3 포함한 전체 시나리오 통합 실행은 아직 성공한 적
  없음 — 다음 실기 최우선 과제.

## 세션 개요 7 (2026-09-28, 우분투 PC 세션이 짠 I-83 대응 패치를 Jetson에 반영)

우분투 PC 쪽 세션(인수인계받은 HANDOFF_SIM_V2.md 기반)이 I-83(도그레그 재현성
문제) 대응 패치를 작성했으나, 그 세션은 인수인계 zip 안의 **복사본**
`mission_5f_to_rooftop.py`만 갖고 있었고 원본 `stair_traverse_node.py`가
없어 `node.odom_cross_track`이 실제 인스턴스 속성인지 확인 못 한 채 안전한
폴백만 만들어뒀음. Jetson에서 확인해보니 실제로는 `run_climb_segment` 안의
지역변수였음(속성 아님) — 그 세션의 우려가 맞았음.

- **K-84 (신규, 2026-09-28, I-83 대응):** (1) `stair_traverse_node.py`에
  **처음으로** 한 줄 추가 — `odom_cross_track`을 지역변수에서
  `self.odom_cross_track`으로도 남기게 함(제어 로직/동작 변경 없음, 값을
  외부에 노출만 함). 지금까지 "이 파일은 절대 안 건드린다" 원칙에서 처음
  벗어난 지점이지만, 기존 동작을 하나도 안 바꾸는 순수 노출용 변경이라
  안전하다고 판단함. (2) `mission_5f_to_rooftop.py`에 우분투 PC 패치를 옮겨
  적용 + 방금 (1)로 실제로 작동하게 함 — 새 env 플래그
  `MISSION_DOGLEG_ADAPTIVE_CORRECTION=1`(기본 꺼짐, 켜기 전엔 기존 동작과
  완전 동일)로 `corrected_advance = DOGLEG_ADVANCE_M + odom_cross_track`
  보정. `node.odom_cross_track`이 없으면 경고만 찍고 고정값 폴백(이제는
  (1) 덕분에 정상 케이스에서 항상 값이 있음). colcon build + import 테스트
  통과, **실기 검증 전혀 안 함 — 다음 실기 세션에서 `--stage 2`로 먼저 확인
  필수** (I-83 이어서 진행).

같은 날, 로봇 계단 앞 배치 후 실기 진행:

- **K-85 (신규, 2026-09-28, 실기로 재확인):** `MISSION_DOGLEG_ADAPTIVE_CORRECTION`
  끈 채로(고정 1.5m) 전체 시나리오 재시도 - flight1 성공(26.1s) 후 도그레그
  translate 도중 다시 벽에 붙을 뻔해 사용자가 리모컨 개입(I-83이 재현된 것,
  이번엔 K-83때보다도 큰 odom_cross_track 편차였을 것으로 추정 - 고정거리
  방식의 재현성 한계가 우연이 아니라 반복적으로 나타남을 재확인).
- **K-86 (신규, 2026-09-28, I-83 해결):** K-84의 도그레그 적응형 보정을
  `MISSION_DOGLEG_ADAPTIVE_CORRECTION=1`로 켜고 `--stage 2` 실기 검증 —
  **첫 실기 시도에서 바로 성공**. flight1 종료 시 odom_cross_track=-0.139m
  (지금까지 관측된 것 중 가장 큰 편차)였는데, 보정 로직이 `1.50m +
  (-0.139m) = 1.36m`로 자동 축소해서 실제 1.32m만 전진, 벽 충돌 없이 두 번째
  회전까지 정상 완료(회전 후 roll=0.6도, 안정적). **I-83 해결로 판단** —
  가장 큰 편차 상황에서 검증됐고 로직도 단순해 추가 검증 부담이 낮음. 앞으로
  `MISSION_DOGLEG_ADAPTIVE_CORRECTION=1`을 기본으로 켜고 진행할 것을 권장
  (다만 아직 flight2 이후까지 포함한 전체 시나리오 통합 성공은 없음).
- **K-87 (신규, 2026-09-28, DOGLEG_TURN_DEG 정밀 보정):** K-86 성공 직후,
  사용자 육안으로 flight2 진입 방향이 "왼쪽으로 치우침" 확인 — 도그레그의
  두 회전이 `run_turn_segment`의 구조적 오버슈트("누적각 >= 목표"에서 멈춤)
  때문에 실측 누적 181.8도(90도x2 명령 기준)까지 나온 게 원인으로 추정.
  `stair_traverse_node.py`의 yaw_hold는 도그레그 종료 시점 방향을 그대로
  "직진 기준"으로 삼아 교정하지 않으므로, 이 오차가 flight2 내내 유지되며
  난간 쪽 위험을 키움. **측정 기반 튜닝 절차**: 사용자가 조종기로 로봇을
  flight2 정면까지 직접 돌리면, 그 전/후 yaw 값을 `ros2 topic echo`로 읽어
  정확한 보정각을 역산 → `DOGLEG_TURN_DEG`에 반영 → 재검증. 세 차례 반복:
  88도(-18도 보정 필요, 목표 163.8도 역산) → 79도(오히려 +8도 보정 필요,
  목표 168.8도 역산, 79도는 부족했음) → 두 역산값 평균(166.3도) 기준
  **82도로 수렴, 실측 누적 168.7도로 검증 완료**. `DOGLEG_ADVANCE_M`도 같은
  과정에서 1.5→1.15→1.2m로 사용자 육안 판단에 따라 조정됨. **이 절차
  (조종기로 직접 정렬 + yaw 값 전/후 측정)가 육안 어림짐작보다 훨씬 정밀한
  파라미터 튜닝 방법으로 확인됨 — 앞으로 비슷한 각도 보정 필요 시 재사용
  권장.**
- **K-88 (신규, 2026-09-28, 마일스톤):** K-87의 82도/1.2m 설정 +
  `MISSION_DOGLEG_ADAPTIVE_CORRECTION=1`로 **전체 시나리오(--stage 없이)를
  실제 5F 계단에서 처음부터 끝까지 완주** — flight1(26.4s) → 도그레그(누적
  168.7도, 1.17m) → **flight2(9단, 24.9s, 첫 실기 성공)** → **옥상 문턱
  90도 회전(누적 -93.5도, 첫 실기 성공)** → **flight3(3단, 8.2s, 첫 실기
  성공, 옥상 가장자리 직전 구간)** → 사진 촬영 5/5장 성공. 비상정지 0회.
  **flight2/threshold_turn/flight3 전 구간이 이번에 처음으로 실기 검증됨.**
  이메일만 `.env` SMTP 키 미입력으로 실패(HANDOFF.md 원래부터 사람이 직접
  채워야 하는 값 — 로봇/등반 로직과 무관). **5F->옥상 계단 등반 자동화
  미션의 핵심 목표(등반+촬영) 달성.**
- **K-89 (신규, 2026-09-28, 재현성 확인):** 같은 설정(82도/1.2m/적응형 보정
  켜짐)으로 `--stage 5`(등반 전체, 사진/이메일 생략) **2회차 연속 성공** —
  flight1(27.5s) → 도그레그(누적 83.9+X도, 1.13m) → flight2(9단, 26.2s) →
  옥상 문턱 회전(누적 -94.2도) → flight3(3단, 8.2s). 전부 비상정지 없이
  정상 완료. **동일 설정으로 2회 연속 전체 등반 성공 확인 — 재현성 있음으로
  판단.** flight3 착지 감지가 이번엔 두 번 일어났다가(731.7s, 736.9s) 최종
  확정됨 - level_hold_sec 디바운스가 의도대로 잠깐의 오탐을 걸러낸 것으로
  보임(문제 아님, 정상 동작).
- **K-90 (신규, 2026-09-28, 사용자 요청 기능 추가):** 두 가지 신규 구간 +
  테스트 편의 기능 추가, `stair_traverse_node.py`는 안 건드림:
  (1) flight2 완료 직후 문턱 회전 전에 `ROOFTOP_APPROACH_ADVANCE_M`(기본
  0.5m, "2~3걸음" 요청을 임시로 환산 - **실측 아님**)만큼 `run_translate_segment`
  로 전진 추가. (2) flight3 완료 직후 사진 촬영 전에
  `ROOFTOP_FINAL_ADVANCE_M`(기본 1.0m, 사용자 지정)만큼 옥상 위로 전진 추가
  - SIDE_ROOFTOP.md 추락 경고 구간이라 주석에 보수적 접근 명시. (3)
  `climb_5f_to_rooftop()`에 `start_stage` 파라미터 + `--start-stage N` CLI
  옵션 추가 - flight1+도그레그가 K-88/K-89로 재현성 검증됐으니 매번 반복하지
  않고 로봇을 flight2 계단 앞에 직접 놓고 그 뒤 구간만 테스트 가능(예:
  `--start-stage 3`). `--stage`(max)와 결합 가능, `start_stage > max_stage`면
  즉시 에러. colcon build + import 테스트 + 인자 파싱 단위 테스트 통과.
  **(1)(2) 두 신규 구간은 실기 검증 0회 — 다음 실기에서 `--start-stage 3`으로
  flight2 계단 앞부터 시작해서 확인할 것.**
- **I-84 (부분 해결, 2026-09-28):** `ROOFTOP_APPROACH_ADVANCE_M`은 첫 실기
  (0.5m, `--start-stage 3 --stage 4`)에서 성공(실측 0.46m, 비상정지 없음) —
  다만 사용자 육안 판단으로 "너무 많다"고 봐서 0.2m로 재조정함(K-91),
  이 값은 아직 미검증. `ROOFTOP_FINAL_ADVANCE_M`(1.0m)은 옥상 가장자리
  구간이라 여전히 실기 검증 0회 — 다음 최우선. 짧은 --stage로 나눠 확인할 것.
- **K-91 (신규, 2026-09-28, 사용자 요청):** `ROOFTOP_APPROACH_ADVANCE_M`
  0.5→0.2m로 낮춤(첫 실기 성공했으나 사용자가 "너무 많다"고 판단, I-84
  대응) — 이 값 자체는 아직 실기 미검증.
- **K-92 (신규, 2026-09-28, 시도했으나 폐기 — 재시도 금지):** 평지 구간(도그레그,
  접근/최종 전진)에서만 로봇 내장 장애물 회피(`API_FREEAVOID`)를 켰다가 계단
  진입 전 다시 끄는 방식으로 벽 충돌을 막아보려 시도(`FREEAVOID_ON_FLAT`
  플래그, 기본 꺼짐, 도입 후 바로 실기 단독 테스트함:
  `test_freeavoid_translate.py`로 평지에서 벽 향해 5m 목표 전진). **결과:
  전혀 작동 안 함** — 켠 상태로도 그냥 벽으로 직진해서 사람이 리모컨으로
  개입해야 했음. `run_translate_segment`/`run_turn_segment`가 쓰는
  `API_MOVE`(직접 속도 명령) 경로에는 FreeAvoid가 개입하지 않는 것으로
  결론. **이 기능은 코드에서 완전히 제거함**(`_flat_segment_freeavoid`
  클래스 + 3곳의 호출부) — 잘못된 안전감을 줄 수 있어서 기본 꺼진 채로도
  안 남겨둠. `stair_traverse_node.py`는 안 건드림. colcon build + import
  테스트 통과. **벽 충돌 방지는 앞으로도 거리/각도 파라미터 튜닝으로만
  접근할 것 — FreeAvoid 재시도하지 말 것.**

## 사람이 해야 할 것 (HANDOFF.md "사람이 직접 해야 하는 것"과 동일, 갱신 없음)

HANDOFF.md 문서 하단 "사람이 직접 해야 하는 것" 1~7항 그대로 유효. 이번
세션에서 자동화 가능한 부분(코드 스켈레톤, build, .env 키 구조, 5F 계단
구조 반영)은 끝냈고, IT팀 SMTP 정보 수집/입력(또는 Gmail 전환 확정),
옥상 맵 실측, 로봇 물리 배치, 현장 안전 감독은 전부 아직 사람 손을 거치지
않음.
