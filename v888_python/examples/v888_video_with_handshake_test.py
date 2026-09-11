"""
v888_video_with_handshake_test.py

v888_video_no_handshake_test.py의 대조군(control group) 스크립트.
차이는 딱 하나 -- connect()에서 skip_handshake를 안 씀(핸드셰이크 정상 진행).
나머지 조건(대기시간, 측정 방식, RUN_SECONDS)은 완전히 동일하게 맞춰서,
"핸드셰이크 유무"만 깨끗하게 비교할 수 있게 했다.

사용법:
    examples/ 폴더 안에서: python v888_video_with_handshake_test.py
    프로젝트 루트에서:      python examples/v888_video_with_handshake_test.py

주의: 조종 명령은 보내지 않고 영상만 확인하므로 프로펠러 장착 상태로
진행해도 안전합니다.
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from v888 import V888

RUN_SECONDS = 10  # no_handshake_test와 동일하게 맞춤

print("=== 핸드셰이크 포함(대조군) 영상 스트림 테스트 ===")
drone = V888()
drone.connect()  # skip_handshake 안 씀 -- 핸드셰이크 정상 진행 (유일한 차이점)
print("연결됨 (핸드셰이크 포함, RC/ACK 송신 정상 진행).")

drone.streamon()
print(f"streamon() 완료. {RUN_SECONDS}초간 프레임 수신 여부 확인...")

frame_read = drone.get_frame_read()
seen_frames = 0
last_frame_bytes = None
start = time.time()
while time.time() - start < RUN_SECONDS:
    jpeg_bytes = frame_read.frame
    if jpeg_bytes and jpeg_bytes != last_frame_bytes:
        seen_frames += 1
        last_frame_bytes = jpeg_bytes
        print(f"  t={time.time()-start:5.1f}s  새 프레임 수신 (누적 {seen_frames}개, {len(jpeg_bytes)} bytes)")
    time.sleep(0.1)

print(f"\n=== 결과 (대조군: 핸드셰이크 포함) ===")
print(f"{RUN_SECONDS}초 동안 수신된 서로 다른 프레임 수: {seen_frames}개")
if seen_frames == 0:
    print("=> 핸드셰이크가 있어도 프레임이 전혀 안 들어옴 -- 다른 원인 의심 필요")
else:
    print(f"=> 정상적으로 {seen_frames}개 프레임 수신됨")
print("\n이 결과를 v888_video_no_handshake_test.py 결과와 비교하세요.")

if seen_frames > 0 and last_frame_bytes:
    with open("with_handshake_test_last_frame.jpg", "wb") as f:
        f.write(last_frame_bytes)
    print("마지막 프레임을 with_handshake_test_last_frame.jpg로 저장했습니다.")

drone.end(land_first=False)
print("종료.")
