1.폴더 설명  
  - joystick_3d : 드론 조종기의 케이스 제작을 위한 step 파일 및 stl 파일 제공  
  - v888_joystick : esp32c3 를 이용하여 v888 드론 조종하는 조종기의 소스 코드 제공 (esp-idf ide 를 사용함)  
  - v888_python : PC에서 드론을 wifi 연결 후 테스트할 수 있는 기본 코드 제공  
    
  - esp32c3_joystick_pinmap.png : esp32c3에서 사용하는 pinmap 과 18650 밧데리 전원 사용시의 전원 연결도를 참고할 수 있음
  <img width="2010" height="1085" alt="image" src="https://github.com/user-attachments/assets/8178ac2c-19d1-4268-a59e-6e736091e101" />

2.WiFI Packet 정리  
  순수 조종 명령 패킷은 88 bytes로 구성됨(esp32c3 코드에서 아래 패킷을 드론으로 전송함)
  <img width="657" height="873" alt="image" src="https://github.com/user-attachments/assets/8507ca05-1c9b-492f-9098-bc5c50af3253" />
    
  위 조종 명령에서 추가로 영상 수신을 지속적으로 하기 위해서는 PC의 파이썬 코드 상에서 아래 값들을 드론으로 지속적으로 전송해야 함
  <img width="654" height="495" alt="image" src="https://github.com/user-attachments/assets/49aee410-b080-4e33-aed4-32c4e8c7b88e" />


