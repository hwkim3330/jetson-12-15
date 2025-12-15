# RSAEM Robot Hardware Optimization Plan

## System Info
- **Board**: Jetson Orin Nano (JetPack 6.2, R36)
- **CUDA**: 12.6
- **TensorRT**: 10.3.0
- **OpenCV**: 4.5.4 (CPU-only)

## Available Hardware Accelerators

| Engine | Status | Usage Plan |
|--------|--------|------------|
| **DLA (dla0, dla1)** | Available | Person/body detection with TensorRT INT8 |
| **NVDEC** | Available | Hardware video decoding for camera |
| **NVENC** | Available | Hardware H.264 encoding for streaming |
| **NVJPG** | Available | Hardware JPEG encoding for web streaming |
| **PVA** | Available | Vision preprocessing (VPI) |
| **GPU** | Available | General CUDA/AI inference |

## Optimization Opportunities

### 1. Camera Pipeline Optimization
**Current**: CPU-based image capture and JPEG encoding
**Optimized**: GStreamer with hardware acceleration

```
Camera → nvarguscamerasrc → nvv4l2h264enc (NVENC) → RTP/RTSP
                          → nvjpegenc (NVJPG) → MJPEG web stream
```

### 2. Person/Body Following (New Feature)
**Model Options**:
- YOLOv8n-pose (body + keypoints, ~6.3M params)
- MobileNet-SSD (lightweight detection)
- PoseNet (body keypoints)

**Acceleration**:
- TensorRT FP16/INT8 on DLA
- Fallback to GPU for unsupported layers

### 3. Hand Gesture Recognition (Future)
- MediaPipe hands or custom CNN
- Simple gestures: stop, follow, go

## Implementation Plan

### Phase 1: rssaem_ai Package
```
src/rssaem/rssaem_ai/
├── CMakeLists.txt
├── package.xml
├── models/
│   └── (TensorRT engine files)
├── scripts/
│   ├── person_detector.py      # TensorRT person detection
│   ├── person_follower.py      # Following behavior
│   └── gesture_control.py      # Hand gesture (future)
├── launch/
│   ├── person_follow.launch.py
│   └── gesture_control.launch.py
└── config/
    └── detector_params.yaml
```

### Phase 2: GStreamer Camera Node (Optional)
Replace `web_video_server` with optimized pipeline:
- Use `nvarguscamerasrc` for CSI camera
- Use `nvjpegenc` for MJPEG output
- Reduce CPU load significantly

## Person Following Algorithm

```
1. Detect person in camera frame (TensorRT)
2. Calculate person center (x, y) in image
3. Convert to angular error (how far from center)
4. Publish cmd_vel:
   - angular.z = proportional to x-error
   - linear.x = proportional to person size (distance)
5. Safety: stop if person too close or not detected
```

## ROS2 Topics

### New Topics (rssaem_ai)
| Topic | Type | Description |
|-------|------|-------------|
| `/person_detection` | `vision_msgs/Detection2DArray` | Detected persons |
| `/person_target` | `geometry_msgs/Point` | Target person center |
| `/follow_enabled` | `std_msgs/Bool` | Enable/disable following |

## Hardware Usage Summary

| Feature | DLA | GPU | NVDEC | NVENC | NVJPG |
|---------|-----|-----|-------|-------|-------|
| Person Detection | ✓ | fallback | - | - | - |
| Camera Decode | - | - | ✓ | - | - |
| Web Streaming | - | - | - | - | ✓ |
| Video Recording | - | - | - | ✓ | - |

## Quick Start Commands

```bash
# Install dependencies
pip3 install ultralytics onnx onnxruntime

# Convert YOLO to TensorRT
yolo export model=yolov8n.pt format=engine device=0

# Launch person following
ros2 launch rssaem_ai person_follow.launch.py
```

## Performance Targets

| Metric | Target |
|--------|--------|
| Detection FPS | 15+ |
| Latency | <100ms |
| GPU Usage | <50% |
| Power | <10W (with DLA) |
