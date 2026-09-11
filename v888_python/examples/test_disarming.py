"""
테스트 절차
프로펠러 제거 상태로 시작 (모터 회전만 육안/소리로 확인하면 되니까)
drone.arm_motors_test('up')로 모터를 돌림 (이전 확인상 이 상태는 계속 유지됨 — 뜨지 않고 계속 회전만 함)
짧은 시간으로 emergency_stop(flag_hold_time=X) 호출
모터가 멈췄는지 확인
멈췄으면 → 다시 아밍하고 더 짧은 시간으로 재시도
안 멈췄으면 → 다시 아밍하고 더 긴 시간으로 재시도
이런 식으로 이분탐색하면서 "멈추는 최소 시간"을 좁혀나감
"""
import time
import os
import sys

# Allow running this script directly from inside examples/ as well as from
# the project root, by making sure the parent folder (which contains the
# v888 package) is on sys.path.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from v888 import V888

drone = V888()
drone.connect()

print("연결 완료.")

test_durations = [1.5, 1.0, 0.7, 0.5, 0.3, 0.15, 0.1, 0.05]  # 필요시 사이값 추가로 좁히기

for dur in test_durations:
    print(f"\n--- {dur}초로 테스트 ---")
    drone.arm_motors_test("up")
    time.sleep(1)  # 모터 도는 거 확인할 시간
    input(f"모터 도는 거 확인됐으면 Enter (지금부터 {dur}초 Stop 전송)...")
    drone.emergency_stop(flag_hold_time=dur)
    result = input("모터가 멈췄나요? (y/n): ")
    print(f"{dur}초: {'멈춤' if result=='y' else '안 멈춤'}")

drone.end()