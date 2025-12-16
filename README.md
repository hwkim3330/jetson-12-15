# RSSAEM Robot

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue?logo=ros)](https://docs.ros.org/en/humble/)
[![Platform](https://img.shields.io/badge/Platform-Jetson_Orin_Nano-green?logo=nvidia)](https://developer.nvidia.com/embedded/jetson-orin-nano)
[![License](https://img.shields.io/badge/License-Apache_2.0-orange)](LICENSE)
[![Python](https://img.shields.io/badge/Python-3.10-yellow?logo=python)](https://www.python.org/)
[![TensorRT](https://img.shields.io/badge/TensorRT-10.3.0-76B900?logo=nvidia)](https://developer.nvidia.com/tensorrt)

> **알쌤로봇** - NVIDIA Jetson Orin Nano 기반 ROS2 교육용 로봇 플랫폼

---

## Features

- **Web Interface** - Tesla-style 반응형 웹 UI (모바일/데스크톱)
- **SLAM** - Cartographer 기반 실시간 지도 생성
- **Navigation** - Nav2 자율 주행
- **AI Vision** - YOLOv8, 사람 감지, 제스처 인식
- **Hardware Acceleration** - CUDA, TensorRT, DLA 지원

---

## Specifications

| Component | Specification |
|-----------|---------------|
| Computer | NVIDIA Jetson Orin Nano 8GB |
| OS | JetPack 6.2 (Ubuntu 22.04) |
| ROS | ROS2 Humble |
| LiDAR | LDROBOT STL-19P (0.03-12m, 10Hz) |
| Motor | ROBOTIS Dynamixel |
| Camera | 160° Fisheye (CSI) |
| Battery | 8.4V Li-ion |

### Hardware Accelerators

| Accelerator | Purpose | Status |
|-------------|---------|--------|
| CUDA | GPU Compute | Active |
| TensorRT | Deep Learning Inference | Active |
| DLA | Low-power AI | Ready |
| NVJPG | JPEG Encoding | Ready |

---

## Quick Start

### Installation (One-line)

```bash
# Clone and install
git clone https://github.com/hwkim3330/jetson-12-15.git ~/rsaembot_ws
cd ~/rsaembot_ws && ./scripts/install.sh
```

### Manual Installation

```bash
# Clone repository
git clone https://github.com/hwkim3330/jetson-12-15.git ~/rsaembot_ws
cd ~/rsaembot_ws

# Install dependencies
sudo apt update
sudo apt install -y ros-humble-rosbridge-server ros-humble-web-video-server

# Build
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# Environment setup
echo "source ~/rsaembot_ws/install/setup.bash" >> ~/.bashrc
echo "export LIDAR_MODEL=LDS-04" >> ~/.bashrc
echo "export RSSAEM_MODEL=rssaem" >> ~/.bashrc
source ~/.bashrc
```

### Autostart Service

```bash
# Install service (auto-start on boot)
sudo cp rssaem.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable rssaem
sudo systemctl start rssaem

# Service commands
sudo systemctl status rssaem   # Check status
sudo systemctl stop rssaem     # Stop
sudo systemctl restart rssaem  # Restart
tail -f ~/rsaembot_ws/autostart.log  # View logs
```

### Manual Launch

```bash
ros2 launch rssaem_web web_interface.launch.py
```

### Access

1. Connect to robot WiFi: **RSSAEM_Robot** (password: `12345678`)
2. Open browser: **http://192.168.10.1:8888/**

---

## Web Interface

### Tabs

| Tab | Function |
|-----|----------|
| Control | Manual control via joystick/keyboard |
| SLAM | Real-time map building |
| Navigate | Autonomous navigation (tap to set goal) |
| AI | Person following, gesture control |

### Keyboard Shortcuts

| Key | Action |
|-----|--------|
| W / S | Forward / Backward |
| A / D | Turn Left / Right |
| Space | Emergency Stop |

### Ports

| Port | Service |
|------|---------|
| 8888 | Web Interface |
| 9090 | WebSocket (rosbridge) |
| 8080 | Camera Stream (MJPEG) |

---

## AI Features

### Available AI Nodes

```bash
# Person Detection (MobileNet-SSD)
ros2 run rssaem_ai person_detector.py

# YOLOv8 Object Detection (TensorRT)
ros2 run rssaem_ai yolo_detector.py

# Gesture Recognition (MediaPipe)
ros2 run rssaem_ai gesture_detector.py

# Body Tracking
ros2 run rssaem_ai body_tracker.py
```

### Gesture Commands

| Gesture | Action |
|---------|--------|
| Open Palm | Forward |
| Fist | Stop |
| Point Left | Turn Left |
| Point Right | Turn Right |
| Thumbs Up | Follow Mode |

---

## SLAM (Mapping)

```bash
# Terminal 1: Robot + Web
ros2 launch rssaem_web web_interface.launch.py

# Terminal 2: Cartographer
ros2 launch rssaem_cartographer cartographer.launch.py

# Save map
ros2 run nav2_map_server map_saver_cli -f ~/map
```

---

## Navigation

```bash
ros2 launch rssaem_navigation2 navigation2.launch.py map:=$HOME/map.yaml
```

Use Navigate tab to set destination by tapping on the map.

---

## WiFi (Default: AP Mode)

Robot creates its own WiFi network (default on boot):

| Property | Value |
|----------|-------|
| SSID | `RSSAEM_Robot` |
| Password | `12345678` |
| IP | `192.168.10.1` |
| Web UI | http://192.168.10.1:8888/ |

```bash
# Switch between modes
./scripts/enable_ap_mode.sh      # AP mode (default)
./scripts/enable_client_mode.sh  # Connect to external WiFi
```

See [docs/WIFI_HOTSPOT_SETUP.md](docs/WIFI_HOTSPOT_SETUP.md) for detailed setup.

---

## Package Structure

```
rsaembot_ws/
├── config/
│   └── nginx/              # nginx config (optional)
├── docs/
│   ├── apriltag/           # AprilTag printables
│   └── WIFI_HOTSPOT_SETUP.md
├── scripts/
│   ├── sync_www.sh         # Web file sync
│   ├── enable_ap_mode.sh   # WiFi AP mode
│   └── enable_client_mode.sh
└── src/
    ├── ldlidar_stl_ros2/   # LiDAR driver
    └── rssaem/
        ├── rssaem_ai/          # AI nodes (TensorRT)
        ├── rssaem_bringup/     # Launch files
        ├── rssaem_cartographer/ # SLAM
        ├── rssaem_description/ # URDF
        ├── rssaem_navigation2/ # Navigation
        ├── rssaem_node/        # Hardware interface
        ├── rssaem_teleop/      # Teleoperation
        └── rssaem_web/         # Web interface
```

---

## Troubleshooting

### Web page not loading
```bash
# Check if services are running
ros2 topic list

# Restart web interface
pkill -f web_interface
ros2 launch rssaem_web web_interface.launch.py
```

### Camera not showing
```bash
# Check camera topic
ros2 topic list | grep camera

# Restart camera node
ros2 run rssaem_ai jetson_camera.py
```

### LiDAR not working
```bash
# Check scan topic
ros2 topic echo /scan --once

# Check USB connection
ls /dev/ttyUSB*
```

---

## Safety Notes

1. **Use only 8.4V adapter** for charging (included)
2. **Do not move robot while powered on**
3. **Use 15W power mode** when on battery
4. **Check Pi camera ribbon cable** connection

---

## Update

```bash
cd ~/rsaembot_ws
git pull
colcon build --symlink-install
```

---

## License

Apache-2.0

---

## Contact

- **Manufacturer**: JetsonAI Co., Ltd.
- **GitHub**: https://github.com/hwkim3330/jetson-12-15
