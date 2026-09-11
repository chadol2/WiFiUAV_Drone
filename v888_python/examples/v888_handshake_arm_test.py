"""
v888_handshake_arm_test.py

connect()가 실제로 핸드셰이크(HELLO -> envelope 12개 -> SHORT_CMD -> cmd2 ->
cmd3)를 보내는지, 그리고 그 뒤 arm_motors_test()로 모터가 정상 아밍/정지되는지
확인하기 위한 최소 스크립트. Wireshark로 캡처하면서 실행하면 됨.

안전 주의:
- 반드시 프로펠러를 제거한 상태로 진행할 것.
- arm_motors_test('up')는 지상 전용(뜨지 않고 모터만 회전)이지만,
  안전을 위해 실비행 상태에서는 절대 사용하지 말 것.

사용법:
    examples/ 폴더 안에서: python v888_handshake_arm_test.py
    프로젝트 루트에서:      python examples/v888_handshake_arm_test.py
"""
import os
import sys
import time

# Allow running this script directly from inside examples/ as well as from
# the project root, by making sure the parent folder (which contains the
# v888 package) is on sys.path.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from v888 import V888

drone = V888()
drone.connect()  # 이제 핸드셰이크 포함
print("연결 완료. 1초 대기 후 아밍...")
time.sleep(1)

drone.arm_motors_test("up")
print("아밍 명령 전송함. 모터 도는지 확인하세요.")
time.sleep(3)  # 이 3초 동안 육안으로 확인

drone.arm_motors_test("down")  # 확실히 끄고
drone.end(land_first=False)    # 바로 연결 종료 (재이륙 방지)
print("종료.")
