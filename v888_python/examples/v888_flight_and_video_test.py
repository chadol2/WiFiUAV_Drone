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
from v888 import V888

drone = V888()
drone.connect()
print("연결 완료.")

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

print("전진...")
drone.send_rc_control(0, 30, 0, 0)
time.sleep(2)
drone.hover()

print("후진...")
drone.send_rc_control(0, -30, 0, 0)
time.sleep(2)
drone.hover()

print("착륙 시도...")
drone.land()
time.sleep(2)

# 착륙 토글이 안 먹힐 경우를 대비한 비상 정지 (V888 실측: 착륙 반복 실패 시 Stop 버튼으로 해결됨)
# drone.emergency_stop()

drone.streamoff()
drone.end()
print("종료.")
