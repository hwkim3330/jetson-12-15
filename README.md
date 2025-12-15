# RSAEM Robot - ROS2 Humble Workspace

알쌤로봇 오린 나노 ROS2 워크스페이스 (v1.5)

## 1. 스펙

| 구분 | 사양 |
|------|------|
| ROS2 | Humble |
| Jetpack | 6.2 |
| TensorRT | 10.3.0 |
| OpenCV | 4.5.4 |
| 컴퓨터 | NVIDIA Orin Nano 8GB |
| LiDAR | LDROBOT STL-19P (0.03-12m, 10Hz) |
| 모터 | ROBOTIS Dynamixel |
| 카메라 | 160도 어안카메라 (CSI) |
| 배터리 | 8.4V Li-ion |

### 하드웨어 가속기

| 가속기 | 용도 | 상태 |
|--------|------|------|
| CUDA | GPU 연산 | 활성 |
| DLA | 딥러닝 추론 | 준비됨 |
| nvvidconv | 비디오 변환 | 활성 |
| NVJPG | JPEG 인코딩 | 준비됨 |

## 2. 빠른 시작

### 2.1 처음 설치

```bash
# 1. 워크스페이스 클론
git clone https://github.com/hwkim3330/jetson-12-15.git ~/rsaembot_ws
cd ~/rsaembot_ws

# 2. 의존성 설치
sudo apt update
sudo apt install -y ros-humble-rosbridge-server ros-humble-web-video-server nginx

# 3. 빌드
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# 4. 환경 설정
echo "source ~/rsaembot_ws/install/setup.bash" >> ~/.bashrc
echo "export LIDAR_MODEL=LDS-04" >> ~/.bashrc
echo "export RSSAEM_MODEL=rssaem" >> ~/.bashrc
source ~/.bashrc

# 5. nginx 설정
sudo cp config/nginx/rssaem.conf /etc/nginx/sites-available/
sudo ln -sf /etc/nginx/sites-available/rssaem.conf /etc/nginx/sites-enabled/
sudo rm -f /etc/nginx/sites-enabled/default
./scripts/sync_www.sh
sudo systemctl restart nginx
sudo systemctl enable nginx
```

### 2.2 실행

```bash
# 웹 인터페이스 + 로봇 전체 실행
ros2 launch rssaem_web web_interface.launch.py
```

### 2.3 접속

브라우저에서:
```
http://<로봇_IP>/
```

예시: `http://10.10.10.213/`

## 3. 웹 인터페이스

### 3.1 기능

| 탭 | 기능 |
|----|------|
| Control | 조이스틱/키보드로 수동 조종 |
| SLAM | 지도 생성 |
| Navigate | 자율 주행 (탭해서 목표 설정) |
| AI | 사람 따라가기, 색상 추적 |

### 3.2 단축키

| 키 | 동작 |
|----|------|
| W/S | 전진/후진 |
| A/D | 좌회전/우회전 |
| Space | 비상 정지 |

### 3.3 서비스 포트

| 포트 | 서비스 | 접근 |
|------|--------|------|
| 80 | nginx (웹) | http://IP/ |
| 8080 | 카메라 스트림 | http://IP/stream |
| 9090 | WebSocket | ws://IP/rosbridge |

## 4. AI 기능

### 4.1 사용 가능한 AI 노드

```bash
# 사람 감지 (MobileNet-SSD)
ros2 run rssaem_ai person_detector.py

# YOLOv8 물체 감지 (TensorRT)
ros2 run rssaem_ai yolo_detector.py

# 제스처 인식 (MediaPipe)
ros2 run rssaem_ai gesture_detector.py

# 색상 추적
ros2 run rssaem_ai body_tracker.py
```

### 4.2 모델 설정

```bash
# AI 모델 다운로드 및 TensorRT 변환
ros2 run rssaem_ai setup_models.py --all
```

### 4.3 제스처 명령어

| 제스처 | 동작 |
|--------|------|
| 손바닥 | 정지 |
| 주먹 | 전진 |
| 왼쪽 가리키기 | 좌회전 |
| 오른쪽 가리키기 | 우회전 |
| 엄지척 | 팔로우 모드 |

## 5. SLAM (지도 생성)

```bash
# 터미널 1: 로봇 + 웹
ros2 launch rssaem_web web_interface.launch.py

# 터미널 2: Cartographer
ros2 launch rssaem_cartographer cartographer.launch.py

# 웹에서 SLAM 탭으로 이동하여 조종
# 지도 완성 후 저장:
ros2 run nav2_map_server map_saver_cli -f ~/map
```

## 6. Navigation (자율 주행)

```bash
# 저장된 지도로 네비게이션 실행
ros2 launch rssaem_navigation2 navigation2.launch.py map:=$HOME/map.yaml
```

웹 인터페이스 Navigate 탭에서 목표 지점 터치

## 7. 오프라인 모드 (WiFi 핫스팟)

로봇이 자체 WiFi를 생성하여 인터넷 없이 작동:

```bash
# 핫스팟 모드 활성화
./scripts/enable_ap_mode.sh

# 클라이언트 모드로 복귀
./scripts/enable_client_mode.sh
```

연결 정보:
- SSID: `RSAEM_Robot`
- 비밀번호: `rsaem1234`
- IP: `192.168.4.1`

자세한 설정: [docs/WIFI_HOTSPOT_SETUP.md](docs/WIFI_HOTSPOT_SETUP.md)

## 8. 패키지 구조

```
rsaembot_ws/
├── config/
│   └── nginx/              # nginx 설정
├── docs/
│   ├── apriltag/           # AprilTag 인쇄용
│   └── WIFI_HOTSPOT_SETUP.md
├── scripts/
│   ├── sync_www.sh         # 웹 파일 동기화
│   ├── enable_ap_mode.sh   # WiFi AP 모드
│   └── enable_client_mode.sh
└── src/
    ├── ldlidar_stl_ros2/   # LiDAR 드라이버
    └── rssaem/
        ├── rssaem_ai/          # AI 노드 (NEW)
        ├── rssaem_bringup/     # 런치 파일
        ├── rssaem_cartographer/ # SLAM
        ├── rssaem_description/ # URDF
        ├── rssaem_navigation2/ # 네비게이션
        ├── rssaem_node/        # 하드웨어
        ├── rssaem_teleop/      # 텔레옵
        └── rssaem_web/         # 웹 인터페이스
```

## 9. 문제 해결

### 웹 페이지가 안 열림

```bash
# nginx 상태 확인
sudo systemctl status nginx

# 웹 파일 동기화
./scripts/sync_www.sh
sudo systemctl restart nginx
```

### 카메라가 안 나옴

```bash
# 카메라 토픽 확인
ros2 topic list | grep camera

# 카메라 노드 재시작
ros2 run rssaem_ai jetson_camera.py
```

### LiDAR 데이터 없음

```bash
# LiDAR 토픽 확인
ros2 topic echo /scan --once

# USB 연결 확인
ls /dev/ttyUSB*
```

## 10. 주의사항

1. **8.4V 어댑터로만 충전** (동봉된 것 사용)
2. **전원 켠 상태로 박스 이동 금지**
3. **파워모드 15W 권장** (배터리 사용 시)
4. **Pi 카메라 케이블 연결 상태 확인**

## 11. 업데이트 방법

```bash
cd ~/rsaembot_ws
git pull
colcon build --symlink-install
./scripts/sync_www.sh
```

## 12. 라이선스

Apache-2.0

## 13. 문의

- 제조사: 주식회사 젯슨에이아이
- GitHub: https://github.com/hwkim3330/jetson-12-15
