**1.폴더 설명**  

  - joystick_3d : 드론 조종기의 케이스 제작을 위한 step 파일 및 stl 파일 제공  
  - v888_joystick : esp32c3 를 이용하여 v888 드론 조종하는 조종기의 소스 코드 제공 (esp-idf ide 를 사용함)  
  - v888_python : PC에서 드론을 wifi 연결 후 테스트할 수 있는 기본 코드 제공
    
                  exmaples폴더-> v888_flight_and_video_test.py : 드론의 전방 카메라로 사진 저장,이륙 후 전진 후진 죄회전 우이동 좌이동 착륙의 기본 명령 실행하는 코드
    
                  exmaples폴더-> v888_drone_viewer.py : 드론의 전방 카메라와 하방 카메라의 전환,사진 저장 및 영상 녹화하는 코드
    
                  exmaples폴더-> v888_handshake_arm_test.py : 드론의 arming, disarming 테스트 코드    
    
 

**2.WiFI Packet 정리**    

  <img width="676" height="326" alt="image" src="https://github.com/user-attachments/assets/7e65ff35-98a6-4489-a3ff-c274fcfc5165" />


  - 순수 조종 명령 패킷은 88 bytes로 구성됨(esp32c3 코드에서 아래 패킷만 드론으로 전송하면 됨)
  <img width="657" height="873" alt="image" src="https://github.com/user-attachments/assets/8507ca05-1c9b-492f-9098-bc5c50af3253" />
    
  - 영상 수신을 위해서는 PC의 파이썬 코드 상에서 위 조종 명령과 함께 아래 값들을 드론으로 지속적으로 전송해야 함  
  
  <img width="654" height="495" alt="image" src="https://github.com/user-attachments/assets/49aee410-b080-4e33-aed4-32c4e8c7b88e" />

**3.JoyStick Controller**   

 - esp32c3_joystick_pinmap.png : esp32c3에서 사용하는 pinmap 과 18650 밧데리 전원 사용시의 전원 연결도를 참고할 수 있음
  <img width="2010" height="1085" alt="image" src="https://github.com/user-attachments/assets/8178ac2c-19d1-4268-a59e-6e736091e101" />

 - joystick controller 사진
  <img width="624" height="587" alt="image" src="https://github.com/user-attachments/assets/2667594f-4d58-4171-b274-9c3c43ed8fe5" />

 - esp32c3 f/w 전체 블럭도
   <img width="1536" height="1024" alt="image" src="https://github.com/user-attachments/assets/0f47e49e-7994-4819-ac44-237f7c596ccb" />

  


