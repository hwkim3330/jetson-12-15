# RSAEM Robot AI Capabilities Analysis

## 1. Hardware Specifications

### Jetson Orin Nano Super (8GB)
| Component | Spec |
|-----------|------|
| GPU | 1024 CUDA cores (Ampere) |
| DLA | 2x Deep Learning Accelerators |
| Memory | 7.4GB unified LPDDR5 |
| AI Performance | 67 TOPS (INT8) |
| Power | 7W - 15W |

### LDROBOT STL-19P LiDAR
| Spec | Value | SLAM Impact |
|------|-------|-------------|
| Range | 0.03m - 12m | 넓은 범위 지도 작성 가능 |
| Scan Rate | 10Hz | 실시간 SLAM 충분 |
| Points/Rev | ~450 | 실내 환경 충분한 해상도 |
| Angular Res | ~0.8° | 벽/가구 윤곽 명확 |

## 2. Current SLAM Config Issues

```lua
-- 현재 설정 (문제점)
TRAJECTORY_BUILDER_2D.min_range = 0.12  -- ❌ LiDAR는 0.03m 가능
TRAJECTORY_BUILDER_2D.max_range = 3.5   -- ❌ LiDAR는 12m 가능!
```

### 권장 최적화
```lua
-- STL-19P 최적화 설정
TRAJECTORY_BUILDER_2D.min_range = 0.05      -- 5cm (안전 마진)
TRAJECTORY_BUILDER_2D.max_range = 10.0      -- 10m (실내 최적)
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 8.0
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.025  -- 2.5cm resolution
```

## 3. AprilTag Integration

### 장점
- 정밀한 위치 보정 (mm 단위)
- SLAM drift 보정
- 특정 위치 마커 (충전소, 목적지)

### 구현 방법
```bash
# 패키지 설치
sudo apt install ros-humble-apriltag-ros

# 토픽
/apriltag/detections → SLAM landmarks로 활용
```

### AprilTag + SLAM 연동
- Cartographer의 `use_landmarks = true` 활성화
- AprilTag 감지를 landmark로 변환
- Loop closure 개선

## 4. Orin Nano AI Capabilities

### 가능한 모델 (실시간)
| Model | Size | FPS (예상) | 용도 |
|-------|------|-----------|------|
| YOLOv8n | 3.2M | 30+ | Object Detection |
| YOLOv8n-pose | 6.3M | 25+ | Pose Estimation |
| MobileNet-SSD | 6M | 40+ | Fast Detection |
| EfficientNet-B0 | 5.3M | 35+ | Classification |
| DeepLabV3-Mobile | 11M | 15+ | Segmentation |

### 어려운 모델
| Model | Size | Issue |
|-------|------|-------|
| OpenVLA | 7B | 메모리 부족 (8GB에서 불가) |
| LLaVA | 7B+ | 메모리 부족 |
| GPT-4V API | - | 네트워크 지연 |

## 5. VLA (Vision-Language-Action) 가능성

### Option A: Small Local VLA (실험적)
```
TinyLlama (1.1B) + Vision Encoder
- 메모리: ~3GB (INT4 양자화)
- 속도: ~2-5 tokens/sec
- 한계: 복잡한 명령 처리 어려움
```

### Option B: Hybrid Approach (권장)
```
Local: YOLOv8 (물체 감지) + Person Tracking
Cloud: GPT-4 API (복잡한 명령 해석)

Flow:
1. "빨간 공 가져와" → 클라우드로 전송
2. GPT-4: "red_ball" 객체 찾기 명령 반환
3. 로컬: YOLOv8로 red_ball 감지
4. 로컬: Navigation으로 이동
```

### Option C: Pre-defined Commands (가장 실용적)
```python
COMMANDS = {
    "따라와": follow_person(),
    "충전해": go_to_charger(),
    "정지": stop(),
    "앞으로": move_forward(),
}
```

## 6. 권장 구현 순서

### Phase 1: SLAM 최적화 (즉시)
- [ ] Cartographer 파라미터 STL-19P에 맞게 조정
- [ ] max_range 10m로 확장
- [ ] 맵 품질 개선

### Phase 2: AprilTag (1-2일)
- [ ] apriltag_ros 설치
- [ ] 충전소/목적지 마커 설치
- [ ] SLAM landmark 연동

### Phase 3: 고급 AI (선택)
- [ ] PyTorch 설치 (Jetson용)
- [ ] YOLOv8 TensorRT 변환
- [ ] DLA 활용 최적화

### Phase 4: VLA 실험 (장기)
- [ ] TinyLlama 테스트
- [ ] 음성 명령 → 텍스트 → 행동 파이프라인
- [ ] 또는 클라우드 API 연동

## 7. 메모리 관리 팁

```bash
# 현재 메모리 사용량 확인
free -h

# GPU 메모리 확인
tegrastats

# 불필요한 프로세스 정리
sudo systemctl stop gdm3  # GUI 비활성화 시 ~500MB 절약
```

## 8. 결론

| 기능 | 실현 가능성 | 난이도 |
|------|------------|--------|
| SLAM 최적화 | ✅ 쉬움 | 설정 변경만 |
| AprilTag | ✅ 쉬움 | 패키지 설치 |
| Person Following | ✅ 완료 | 이미 구현됨 |
| Object Detection | ✅ 가능 | YOLOv8 설치 필요 |
| Full VLA | ⚠️ 제한적 | 메모리 한계 |
| Cloud VLA | ✅ 가능 | 네트워크 필요 |
