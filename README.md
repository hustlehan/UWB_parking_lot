# UWB 실내 차량 위치 추적 프로젝트

UWB 앵커 스테이션을 활용한 차량 내 UWB 태그를 통한 실내 차량 위치 추적 프로젝트

## 프로젝트 구조

```
hanium2025/
├── park_ws/          # 주차장 관리 워크스페이스
├── uwb_ws/           # UWB 파서 워크스페이스
└── microros_ws/      # Micro ROS 워크스페이스
```

## 사용 방법

### 1. Micro ROS 개통
```bash
cd ~/microros_ws
source install/setup.bash
ls /dev/tty*
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200 -v6
```

### 2. Parser 노드 실행
```bash
cd uwb_ws
ros2 launch uwb_parser uwb_parser.launch.py
```

### 3. 경로전송 노드 실행
```bash
cd park_ws
ros2 launch uwb_parser uwb_parser.launch.py
```

### 4. 주차장 관리자 노드 실행
```bash
cd park_ws
ros2 launch parking_exe parking_exe.launch.py
```

## 실행 순서

위의 1~4단계를 순서대로 실행하면 시스템이 정상적으로 작동합니다.

## 간단한 요구사항

- ROS2(JAZZY)
- Micro ROS(JAZZY) -> BASHrc 설정 필요
- DWM1000 UWB 모듈
- 시리얼 통신 포트

