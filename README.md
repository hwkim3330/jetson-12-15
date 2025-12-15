# RSAEM Robot - ROS2 Humble Workspace

알쌤로봇 오린 나노 기본 버전 ROS2 워크스페이스

## 1. 스펙

| 구분 | 사양 |
|------|------|
| ROS2 | Humble |
| Jetpack | 6.2 |
| OpenCV | 4.8 |
| 컴퓨터 | NVIDIA Orin Nano |
| LiDAR | 2D LiDAR (12m 거리) |
| 모터 | ROBOTIS Dynamixel |
| 카메라 | 160도 어안카메라 |
| 배터리 | 8.4V Li-ion |
| 통신 | USB, WiFi, UART, I2C, SPI |

### 옵션
- IMU
- 오디오 기능 쉴드 보드

## 2. 주의사항

1. **반드시 8.4V 어댑터로 충전** (판매 시 동봉)
   - 지정된 규격 외 어댑터 사용 시 배터리/전원 시스템 손상 가능

2. **전원이 켜진 상태로 박스에 넣어 이동 금지** (쇼트 위험)

3. **Pi 카메라 케이블 연결 상태 유지**
   - 케이블이 느슨하거나 손상되면 카메라 동작 불가

4. **USB 포트에 너무 많은 장치 연결 금지**

5. **오린 나노 파워모드 15W 권장**
   - 배터리 사용 시 안정적인 동작을 위해

## 3. 빌드

```bash
cd ~/rsaembot_ws
colcon build
```

### 환경 설정 (최초 1회)

```bash
echo "source ~/rsaembot_ws/install/setup.bash" >> ~/.bashrc
echo "export LIDAR_MODEL=LDS-04" >> ~/.bashrc
echo "export RSSAEM_MODEL=rssaem" >> ~/.bashrc
source ~/.bashrc
```

## 4. 사용법

### 4.1 로봇 Bringup (로컬)

```bash
ros2 launch rssaem_bringup rssaem.launch.py
```

### 4.2 텔레옵 (키보드 조종)

```bash
# 다른 터미널에서
ros2 run rssaem_teleop teleop_keyboard
```

> **팁**: 테스트 시 종이컵 위에 로봇을 올려놓고 바퀴 동작 확인

### 4.3 토픽 및 노드 확인

```bash
ros2 topic list
rqt_graph
```

### 4.4 RViz2 시각화

```bash
ros2 launch rssaem_bringup rviz2.launch.py
```

## 5. 원격 제어 (PC에서)

### 5.1 PC 환경 설정

```bash
# ~/.bashrc에 추가 (ROS_DOMAIN_ID는 로봇과 동일하게)
export ROS_DOMAIN_ID=30
```

### 5.2 로봇에 SSH 접속 후 Bringup

```bash
ssh nvidia@<로봇_IP>
# 예: ssh nvidia@192.168.100.99

ros2 launch rssaem_bringup rssaem.launch.py
```

### 5.3 PC에서 텔레옵 실행

```bash
ros2 run rssaem_teleop teleop_keyboard
```

## 6. SLAM (지도 생성)

### 6.1 실행

```bash
# 터미널 1: 로봇 Bringup (SSH)
ros2 launch rssaem_bringup rssaem.launch.py

# 터미널 2: Cartographer
ros2 launch rssaem_cartographer cartographer.launch.py

# 터미널 3: 텔레옵으로 로봇 이동
ros2 run rssaem_teleop teleop_keyboard
```

> **팁**: 로봇 속도 0.3 이하로 천천히 이동해야 깨끗한 지도 생성 가능

### 6.2 지도 저장

```bash
ros2 run nav2_map_server map_saver_cli -f ~/map
```

## 7. Navigation2 (자율 주행)

### 7.1 실행

```bash
# 터미널 1: 로봇 Bringup (SSH)
ros2 launch rssaem_bringup rssaem.launch.py

# 터미널 2: Navigation2
ros2 launch rssaem_navigation2 navigation2.launch.py map:=$HOME/map.yaml
```

> **중요**: SLAM 때 사용한 시작 위치와 방향에 로봇을 배치

### 7.2 목표 지점 설정

RViz2에서 `Nav2 Goal` 버튼으로 목표 지점 클릭

## 8. 웹 인터페이스 (Tesla Style)

부팅 시 자동으로 웹 서버가 시작됩니다.

### 8.1 접속 방법

브라우저에서 다음 주소로 접속:
```
http://<로봇_IP>:8888
```

### 8.2 기능

- **LiDAR 시각화**: 실시간 2D LiDAR 데이터 표시
- **조이스틱 컨트롤**: 마우스/터치로 로봇 조종
- **키보드 컨트롤**: W/A/S/D 또는 방향키
- **SLAM 모드**: 지도 생성
- **Navigation 모드**: 자율 주행
- **비상 정지**: 즉시 정지 버튼

### 8.3 수동 실행

```bash
# 기본 웹 인터페이스
ros2 launch rssaem_web web_interface.launch.py

# SLAM + 웹 인터페이스
ros2 launch rssaem_web slam_web.launch.py

# Navigation + 웹 인터페이스
ros2 launch rssaem_web nav_web.launch.py map:=$HOME/map.yaml
```

### 8.4 포트 정보

| 포트 | 서비스 |
|------|--------|
| 8888 | 웹 인터페이스 |
| 9090 | ROSBridge WebSocket |
| 8080 | 비디오 스트리밍 |

### 8.5 자동 시작 서비스 관리

```bash
# 서비스 상태 확인
sudo systemctl status rssaem

# 서비스 중지
sudo systemctl stop rssaem

# 서비스 시작
sudo systemctl start rssaem

# 자동 시작 비활성화
sudo systemctl disable rssaem
```

## 9. 패키지 구조

```
rsaembot_ws/
└── src/
    ├── ldlidar_stl_ros2/     # LiDAR 드라이버
    └── rssaem/
        ├── rssaem/               # 메타 패키지
        ├── rssaem_bringup/       # 런치 파일
        ├── rssaem_cartographer/  # SLAM
        ├── rssaem_description/   # URDF/모델
        ├── rssaem_msgs/          # 커스텀 메시지
        ├── rssaem_navigation2/   # 네비게이션
        ├── rssaem_node/          # 하드웨어 인터페이스
        ├── rssaem_teleop/        # 키보드 조종
        ├── rssaem_web/           # 웹 인터페이스
        └── rf2o_laser_odometry/  # 레이저 오도메트리
```

## 10. 문의

- 제조사: 주식회사 젯슨에이아이
