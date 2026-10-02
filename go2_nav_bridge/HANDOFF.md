# HANDOFF — 5F→옥상→사진→이메일 미션

## 목표 / 원칙
5층 출발 → (기존 계단 그대로) 등반 → 옥상 도착 → Go2 RGB 카메라로 사진 촬영 → 회사 메일 전송.

**계단 등반은 "완벽"이 목표가 아니라 "일단 오른다"가 목표.**
기존 stair_traverse_node.py의 K-xx 튜닝값(9+9단+도그레그, yaw_hold_kp=0.3,
LiDAR 유효각 −84°~−70° 등)을 그대로 재사용하고, 새로운 안전 검증/재시도/
복구 로직을 추가로 설계하지 말 것. 실패하면 즉시 멈추고 사람에게 알리는
수준이면 충분함 (AutoRecoverySet(false)는 의도적 설정이므로 건드리지 말 것).

## 참고 파일 (~/ros2_ws/src/go2_nav_bridge/go2_nav_bridge/ 로 복사해서 시작)
- mission_5f_to_rooftop.py — 상태 머신 뼈대
- photo_shooter.py — 사진 촬영 서비스 노드 뼈대 (VideoClient API 미검증)
- email_sender.py — 이메일 전송 모듈 (회사 SMTP, .env 필요)
- .env.example — .env 템플릿 (실제 값은 사람이 채움)

## 자동으로 진행할 것 (Claude Code가 스스로 판단·실행)
1. `unitree_sdk2py.go2.video.video_client.VideoClient` / `GetImageSample()`이
   설치된 SDK에 실제로 존재하는지 import 테스트로 확인. 없으면 설치된 SDK를
   뒤져서 실제 카메라 접근 클래스/메서드로 photo_shooter.py를 수정.
2. stair_traverse_node.py가 실제로 어떻게 트리거되는지(서비스/토픽/launch
   조건) 코드에서 확인하고, mission_5f_to_rooftop.py의 stair_trigger_client를
   그 인터페이스에 맞게 교체. **이 단계에서 새 안전 로직을 추가하지 말 것** —
   트리거 후 완료/실패 신호만 받으면 됨.
3. `~/maps/` 안에 옥상용 맵이 있는지 확인. 있으면 waypoint 좌표를 찾아 mission
   코드에 반영. 없으면 STATE.md/I-xx에 "옥상 맵 없음 — 실제 로봇 매핑 필요"라고
   기록만 하고 다음 단계로 진행 (맵 생성 자체는 물리적 작업이라 자동화 불가).
4. colcon build, import 테스트, 문법/의존성 오류 수정.
5. `.env` 파일의 키 구조만 준비 — 실제 비밀번호·서버 주소는 절대 채우지 말 것.
6. Orin1 iptables(DOCKER-USER 체인)에서 SMTP 포트가 열려 있는지
   `nc -zv <host> <port>` 등으로 테스트만 하고, 규칙 변경 자체는 사람 승인 후 진행.
7. 진행 상황을 STATE.md에 기존 K-xx/I-xx 컨벤션으로 기록.

## 사람이 직접 해야 하는 것 (자동화 불가 / 보안상 금지)
1. IT팀에 회사 SMTP 서버 정보(host, port, TLS 방식, 계정) 문의해서 받아오기.
2. 받은 정보 + 계정 비밀번호를 `.env`에 직접 입력 (Claude Code가 대신 입력 금지).
3. Orin1 iptables 규칙 변경 최종 승인.
4. 옥상 맵이 없다면 로봇을 옥상에서 실제로 주행시켜 맵/waypoint 직접 생성.
5. 로봇을 5층 계단 진입 지점에 물리적으로 배치.
6. 미션 실행 트리거 및 계단 등반 중 현장 안전 감독 — AutoRecovery가 의도적으로
   꺼져 있으므로 넘어짐 위험 시 즉시 개입할 사람이 현장에 있어야 함.
7. Orin2 물리 모니터 연결 여부 확인 (기존 미확인 항목).
