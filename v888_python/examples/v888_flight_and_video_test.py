"""
안전 주의: 최초 실행 시 반드시 프로펠러(날개)를 제거한 상태로 테스트할 것.
모터 회전/방향만 확인 후, 정상 동작 확인되면 날개를 장착하고 재테스트.
"""
import os
import sys
import time

# Allow running this script directly from inside examples/ as well as from
# the project root, by making sure the parent folder (which contains the
# v888 package) is on sys.path.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2
from v888 import V888, V888Error

drone = V888()

# WiFi(드론 AP)에 실제로 연결되어 있지 않으면 여기서 즉시 중단합니다.
# require_network=True면 연결이 안 잡혀 있을 때 connect()가 소켓을 열지도,
# 핸드셰이크를 보내지도 않고 바로 V888Error를 던지므로, 아래의 어떤 조종
# 명령(streamon/takeoff/send_rc_control/land 등)도 실행되지 않습니다.
try:
    drone.connect(require_network=True, require_ssid_prefix="FLOW_")
except V888Error as e:
    print(f"[오류] {e}")
    print("드론 WiFi에 연결되어 있지 않아 조종 명령을 실행하지 않고 종료합니다.")
    sys.exit(1)

print("연결 완료.")


def move_and_check(label, roll, pitch, throttle, yaw, hold_seconds=2):
    """
    조종 명령을 보내고 hold_seconds만큼 유지한 뒤, 다음 동작으로 넘어가기
    전에 실제로 패킷이 계속 잘 나가고 있는지(is_link_healthy) 확인합니다.
    WiFi가 끊겨서 패킷이 안 나가고 있다면, 스크립트가 그걸 모른 채 계속
    다음 안무를 진행하지 않도록 여기서 멈춰서 알립니다.

    동작 "시작 전"에도 링크를 확인합니다 -- 링크가 불안정한 채로 이동을
    시작하면, 명령이 일부만 전달되어 "우측 이동은 됐는데 좌측 이동은 덜
    됨" 같은 식으로 왕복 이동이 비대칭적으로 어긋나는 현상(원위치로 못
    돌아옴)의 원인이 될 수 있습니다.
    """
    if not _wait_for_link(label, "시작 전"):
        print(f"  → '{label}' 동작을 생략하고 착륙으로 넘어갑니다.")
        return False

    print(f"{label}...")
    drone.send_rc_control(roll, pitch, throttle, yaw)
    time.sleep(hold_seconds)
    drone.hover()

    if not _wait_for_link(label, "종료 후"):
        return False
    return True


def _wait_for_link(label, when, max_wait=5.0):
    """링크가 불안정하면 최대 max_wait초 동안 복구를 기다린다."""
    if drone.is_link_healthy(max_silence=1.0):
        return True
    print(
        f"[경고] '{label}' 동작 {when} WiFi 링크 상태가 불안정합니다 "
        f"(마지막 정상 전송 후 {drone.seconds_since_last_successful_send():.1f}초 경과). "
        f"연결이 계속 끊겨 있으면 명령이 드론에 전달되지 않습니다."
    )
    for _ in range(int(max_wait * 10)):
        if drone.is_link_healthy(max_silence=1.0):
            print("  → 링크 복구됨.")
            return True
        time.sleep(0.1)
    print(f"  → {max_wait:.0f}초 내 복구 안 됨.")
    return False


# 영상 스트림 시작 (조종 패킷 전송보다 먼저 트리거를 보내야 함 - streamon()이 자동 처리)
drone.streamon()
frame_read = drone.get_frame_read()

print("5초간 영상 미리보기 (프레임 저장)...")
start = time.time()
saved = False
while time.time() - start < 5:
    jpeg_bytes = frame_read.frame
    if jpeg_bytes and not saved:
        with open("v888_preview.jpg", "wb") as f:
            f.write(jpeg_bytes)
        saved = True
        print("첫 프레임 저장: v888_preview.jpg")
    time.sleep(0.1)

print("이륙...")
drone.takeoff()
time.sleep(3)

link_ok = True
# 전진/후진 값을 전부 대칭(±15)으로 맞춤 (이전 버전은 첫 전진만 20이라
# 크기가 달라서, 곧바로 이어지는 후진이 그만큼 덜 이동하는 게 산수상
# 당연한 결과였음 -- 링크 끊김과 별개로 원위치 미복귀의 원인 중 하나였음)
link_ok = link_ok and move_and_check("전진", 0, 15, 0, 0)
link_ok = link_ok and move_and_check("후진", 0, -15, 0, 0)
link_ok = link_ok and move_and_check("좌회전", 0, 0, 0, -15)
link_ok = link_ok and move_and_check("우회전", 0, 0, 0, 15)
link_ok = link_ok and move_and_check("우측 이동", 15, 0, 0, 0)
link_ok = link_ok and move_and_check("좌측 이동", -15, 0, 0, 0)

if not link_ok:
    print("[안내] 중간에 링크가 끊겨 남은 안무를 생략하고 바로 착륙을 시도합니다.")

print("착륙...")
drone.land()
time.sleep(2)

# 착륙 토글이 안 먹힐 경우를 대비한 비상 정지 (V888 실측: 착륙 반복 실패 시 Stop 버튼으로 해결됨)
# drone.emergency_stop()

drone.streamoff()
drone.end()
print("종료.")
