#!/usr/bin/env python3
"""email_sender — 회사 SMTP로 미션 결과(사진 포함) 이메일 전송

설정은 이 파일과 같은 디렉터리의 `.env`(git에 커밋하지 말 것, `.env.example`
참고)에서 읽는다. **실제 SMTP host/port/계정/비밀번호는 사람이 IT팀에 문의해
직접 `.env`에 입력해야 한다 — Claude Code가 대신 채우지 않음(HANDOFF.md 2/3항).**
필수 키가 비어있으면 실제 네트워크 연결을 시도하지 않고 바로 에러를 반환한다.

SMTP_USE_TLS=true(기본)면 STARTTLS, false면 암시적 TLS(SMTPS, 보통 465포트)로
접속한다 — 회사 서버가 어느 방식인지 IT팀에 확인 필요.
"""
import json
import mimetypes
import os
import smtplib
import socket
import ssl
import time
from email.message import EmailMessage
from pathlib import Path

_ENV_PATH = Path(__file__).resolve().parent / '.env'
OUTBOX_DIR = Path(os.path.expanduser(os.environ.get('EMAIL_OUTBOX_DIR', '~/email_outbox')))
_REQUIRED_KEYS = ('SMTP_HOST', 'SMTP_PORT', 'SMTP_USER', 'SMTP_PASSWORD',
                   'MAIL_FROM', 'MAIL_TO')


def _load_env(env_path=_ENV_PATH):
    """.env 파일을 dict로 로드 (python-dotenv 미설치 상태라 최소 파서 직접 구현).

    형식: KEY=VALUE, '#'로 시작하는 줄/빈 줄 무시.
    """
    env = {}
    if not env_path.exists():
        return env
    for line in env_path.read_text(encoding='utf-8').splitlines():
        line = line.strip()
        if not line or line.startswith('#') or '=' not in line:
            continue
        key, _, value = line.partition('=')
        env[key.strip()] = value.strip()
    return env


def _get_config():
    env = {**_load_env(), **os.environ}  # 환경변수가 .env보다 우선
    missing = [k for k in _REQUIRED_KEYS if not env.get(k)]
    if missing:
        return None, (
            f'.env 필수 키 누락: {", ".join(missing)} '
            '(IT팀에 SMTP 정보 문의 후 .env에 직접 입력 필요, HANDOFF.md 참고)')

    try:
        port = int(env['SMTP_PORT'])
    except ValueError:
        return None, f'SMTP_PORT가 정수가 아님: {env["SMTP_PORT"]!r}'

    use_tls = env.get('SMTP_USE_TLS', 'true').strip().lower() not in ('false', '0', 'no')

    return {
        'host': env['SMTP_HOST'],
        'port': port,
        'use_tls': use_tls,
        'user': env['SMTP_USER'],
        'password': env['SMTP_PASSWORD'],
        'mail_from': env['MAIL_FROM'],
        'mail_to': env['MAIL_TO'],
    }, None


def send_mission_email(subject, body, attachment_paths=None):
    """미션 결과 이메일 전송.

    Returns: (ok: bool, error: str | None)
    """
    config, err = _get_config()
    if config is None:
        return False, err

    msg = EmailMessage()
    msg['Subject'] = subject
    msg['From'] = config['mail_from']
    msg['To'] = config['mail_to']
    msg.set_content(body)

    for path in (attachment_paths or []):
        try:
            data = Path(path).read_bytes()
        except OSError as e:
            return False, f'첨부파일 읽기 실패 ({path}): {e}'
        mime, _ = mimetypes.guess_type(str(path))
        maintype, _, subtype = (mime or 'application/octet-stream').partition('/')
        msg.add_attachment(data, maintype=maintype, subtype=subtype,
                            filename=Path(path).name)

    try:
        if config['use_tls']:
            with smtplib.SMTP(config['host'], config['port'], timeout=120) as smtp:
                smtp.starttls(context=ssl.create_default_context())
                smtp.login(config['user'], config['password'])
                smtp.send_message(msg)
        else:
            with smtplib.SMTP_SSL(config['host'], config['port'],
                                   context=ssl.create_default_context(),
                                   timeout=120) as smtp:
                smtp.login(config['user'], config['password'])
                smtp.send_message(msg)
    except (smtplib.SMTPException, OSError) as e:
        return False, f'SMTP 전송 실패: {e}'

    return True, None


def queue_email(subject, body, attachment_paths=None):
    """전송 전에 디스크 대기열에 저장 (네트워크가 없어도 사진/영상 경로는 보존)."""
    OUTBOX_DIR.mkdir(parents=True, exist_ok=True)
    job = OUTBOX_DIR / f'{time.strftime("%Y%m%d_%H%M%S")}_{time.time_ns() % 10**6:06d}.json'
    job.write_text(json.dumps({
        'subject': subject, 'body': body,
        'attachments': [str(p) for p in (attachment_paths or [])]}, ensure_ascii=False))
    return job


def flush_outbox():
    """대기열의 메일을 전부 전송 시도. 반환: (보낸 수, 남은 수, 마지막 에러)."""
    sent, last_err = 0, None
    jobs = sorted(OUTBOX_DIR.glob('*.json')) if OUTBOX_DIR.exists() else []
    for job in jobs:
        data = json.loads(job.read_text())
        paths = [p for p in data['attachments'] if Path(p).exists()]
        ok, err = send_mission_email(data['subject'], data['body'], paths)
        if ok:
            job.unlink()
            sent += 1
        else:
            last_err = err
            break  # 네트워크/인증 문제면 나머지도 실패하므로 중단
    remaining = len(list(OUTBOX_DIR.glob('*.json'))) if OUTBOX_DIR.exists() else 0
    return sent, remaining, last_err


def send_with_retry(subject, body, attachment_paths=None,
                    max_wait_sec=1800.0, interval_sec=3.0, log=print):
    """대기열에 저장 후 네트워크가 연결될 때까지 재시도해 최대한 빨리 전송.

    반환: (ok, error). 시간 안에 못 보내면 대기열에 남기고 False
    (`ros2 run go2_nav_bridge email_sender --flush`로 나중에 재전송 가능).
    """
    queue_email(subject, body, attachment_paths)
    deadline = time.time() + max_wait_sec
    attempt = 0
    while True:
        attempt += 1
        port_ok, port_err = check_smtp_port_open(timeout=3.0)
        if port_ok:
            _, remaining, err = flush_outbox()
            if remaining == 0:
                return True, None
        else:
            err = port_err
        if attempt == 1 or attempt % 10 == 0:
            log(f'[이메일] 전송 대기 중 (시도 {attempt}): {err}')
        if time.time() >= deadline:
            return False, f'{max_wait_sec:.0f}초 안에 전송 못 함 - 대기열에 남김: {err}'
        time.sleep(interval_sec)


def check_smtp_port_open(timeout=3.0):
    """SMTP host:port 오픈 여부만 확인 (로그인/전송 없음, `nc -zv`와 동일 목적).

    HANDOFF.md 6항: iptables 규칙 변경 자체는 사람 승인 후 진행 — 여기서는
    포트가 열려있는지 확인만 한다.
    """
    config, err = _get_config()
    if config is None:
        return False, err
    try:
        with socket.create_connection((config['host'], config['port']), timeout=timeout):
            return True, None
    except OSError as e:
        return False, f'{config["host"]}:{config["port"]} 연결 실패: {e}'


def main():
    import argparse
    parser = argparse.ArgumentParser(description='email_sender 단독 테스트')
    parser.add_argument('--check-port', action='store_true',
                         help='.env의 SMTP host:port 오픈 여부만 확인 (전송 안 함)')
    parser.add_argument('--flush', action='store_true',
                         help='대기열(~/email_outbox)의 미전송 메일을 재전송')
    parser.add_argument('--watch', action='store_true',
                         help='--flush와 함께: 대기열이 빌 때까지 계속 재시도')
    parser.add_argument('--send-test', action='store_true',
                         help='.env 설정으로 테스트 메일 실제 전송')
    args = parser.parse_args()

    if args.check_port:
        ok, err = check_smtp_port_open()
        print('포트 열림' if ok else f'포트 확인 실패: {err}')
        return

    if args.flush:
        while True:
            port_ok, port_err = check_smtp_port_open(timeout=3.0)
            sent, remaining, err = flush_outbox() if port_ok else (0, -1, port_err)
            print(f'전송 {sent}건, 남음 {remaining}건' + (f' ({err})' if err else ''))
            if remaining == 0 or not args.watch:
                return
            time.sleep(5.0)

    if args.send_test:
        ok, err = send_mission_email(
            subject='[Go2] email_sender 테스트',
            body='이 메일은 email_sender.py --send-test 로 발송된 테스트 메일입니다.')
        print('전송 성공' if ok else f'전송 실패: {err}')
        return

    parser.print_help()


if __name__ == '__main__':
    main()
