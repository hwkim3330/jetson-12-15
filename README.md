# KETI Robot

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue?logo=ros)](https://docs.ros.org/en/humble/)
[![Platform](https://img.shields.io/badge/Platform-Jetson_Orin_Nano-green?logo=nvidia)](https://developer.nvidia.com/embedded/jetson-orin-nano)
[![License](https://img.shields.io/badge/License-Apache_2.0-orange)](LICENSE)
[![Python](https://img.shields.io/badge/Python-3.10-yellow?logo=python)](https://www.python.org/)
[![TensorRT](https://img.shields.io/badge/TensorRT-10.3.0-76B900?logo=nvidia)](https://developer.nvidia.com/tensorrt)

> **KETI Robot** - NVIDIA Jetson Orin Nano 기반 ROS2 자율주행 로봇 플랫폼

---

## Features

- **Web Interface** - Tesla-style 반응형 웹 UI (모바일/데스크톱)
- **SLAM** - Cartographer 기반 실시간 지도 생성
- **Navigation** - Nav2 자율 주행
- **AI Vision** - YOLOv8 TensorRT, 사람 감지, 제스처 인식
- **Hardware Acceleration** - CUDA, TensorRT, DLA, NVJPG 지원

---

## Specifications

| Component | Specification |
|-----------|---------------|
| Computer | NVIDIA Jetson Orin Nano Super 8GB |
| OS | JetPack 6.2 (Ubuntu 22.04) |
| ROS | ROS2 Humble |
| LiDAR | LDRobot LD19 (0.03-12m, 10Hz) |
| Motor | DC Motor with Encoder |
| Camera | CSI Camera (IMX219) |

### Hardware Accelerators

| Accelerator | Purpose | Status |
|-------------|---------|--------|
| CUDA | GPU Compute | Active |
| TensorRT | Deep Learning Inference | Active |
| DLA | Low-power AI | Ready |
| NVJPG | JPEG Encoding | Ready |

---

## Quick Start

### Installation

```bash
# Clone and install
git clone https://github.com/hwkim3330/jetson-12-15.git ~/ros2_ws
cd ~/ros2_ws
chmod +x scripts/install.sh
./scripts/install.sh
```

### Autostart Service

```bash
# Enable auto-start on boot
sudo systemctl enable robot
sudo systemctl start robot

# Service commands
sudo systemctl status robot   # Check status
sudo systemctl stop robot     # Stop
sudo systemctl restart robot  # Restart
journalctl -u robot -f        # View logs
```

### Manual Launch

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch robot_bringup robot.launch.py
```

### Access

Open browser: **http://<ROBOT_IP>:8888/**

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

---

## AI Features

### Available AI Nodes

```bash
# YOLOv8 Object Detection (TensorRT)
ros2 run robot_ai yolo_detector.py

# Person Detection
ros2 run robot_ai person_detector.py

# Gesture Recognition
ros2 run robot_ai gesture_detector.py

# Person Following
ros2 run robot_ai person_follower.py
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
# Launch SLAM
ros2 launch robot_slam cartographer.launch.py

# Save map
ros2 run nav2_map_server map_saver_cli -f ~/map
```

---

## Navigation

```bash
ros2 launch robot_navigation navigation.launch.py map:=$HOME/map.yaml
```

Use Navigate tab to set destination by tapping on the map.

---

## Package Structure

```
ros2_ws/
├── config/
│   ├── robot.service      # systemd service
│   └── nginx/             # nginx config (optional)
├── scripts/
│   ├── install.sh         # Installation script
│   ├── start_robot.sh     # Start script
│   ├── stop_robot.sh      # Stop script
│   └── sync_www.sh        # Web file sync
└── src/
    ├── ldlidar_stl_ros2/  # LiDAR driver
    └── keti/
        ├── robot_ai/          # AI nodes (TensorRT)
        ├── robot_bringup/     # Launch files
        ├── robot_driver/      # Motor driver
        ├── robot_description/ # URDF
        ├── robot_navigation/  # Nav2 config
        ├── robot_slam/        # SLAM config
        ├── robot_teleop/      # Teleoperation
        ├── robot_web/         # Web interface
        └── robot_msgs/        # Custom messages
```

---

## ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | geometry_msgs/Twist | Velocity command |
| `/scan` | sensor_msgs/LaserScan | LiDAR scan |
| `/odom` | nav_msgs/Odometry | Odometry |
| `/camera/image_raw` | sensor_msgs/Image | Camera image |
| `/map` | nav_msgs/OccupancyGrid | SLAM map |

---

## Troubleshooting

### Web page not loading
```bash
ros2 topic list
sudo systemctl restart robot
```

### Camera not showing
```bash
ros2 topic list | grep camera
ros2 run robot_ai jetson_camera.py
```

### LiDAR not working
```bash
ros2 topic echo /scan --once
ls /dev/ttyUSB*
```

---

## Update

```bash
cd ~/ros2_ws
git pull
colcon build --symlink-install
```

---

## License

Apache-2.0

---

## Contact

- **GitHub**: https://github.com/hwkim3330/jetson-12-15
