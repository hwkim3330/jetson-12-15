# AprilTag for RSAEM Robot

AprilTag is a visual fiduciary marker system useful for robot localization and navigation.

## Recommended Family: 36h11

The `tag36h11` family is recommended for most robotics applications:
- 587 unique tags available (ID 0-586)
- Good detection robustness
- Works well at various distances

## Printable Tags

### Online Generator
- [AprilTag Generator](https://apriltag.org/)
- [AprilTag PDF Generator](https://github.com/AprilRobotics/apriltag-imgs)

### Pre-generated PDFs
Download and print from the official repository:
```bash
git clone https://github.com/AprilRobotics/apriltag-imgs.git
```

## Recommended Tag Sizes

| Use Case | Size | Detection Range |
|----------|------|-----------------|
| Docking Station | 10cm | 0.5-2m |
| Room Markers | 15cm | 1-3m |
| Long Range | 20cm | 2-5m |

## Integration with ROS2

### Install apriltag_ros
```bash
sudo apt install ros-humble-apriltag-ros
```

### Launch with camera
```bash
ros2 launch apriltag_ros apriltag_node.launch.py
```

### Published Topics
- `/apriltag/detections` - Detected tags with poses
- `/apriltag/image` - Debug image with tag overlays

## Tag Placement Suggestions

1. **Charging Station**: Tag ID 0 at charger location
2. **Home Position**: Tag ID 1 at home/start position
3. **Waypoints**: Tags 10-19 for navigation waypoints
4. **Rooms**: Tags 100-199 for room identification

## Cartographer Integration

Enable landmarks in `rssaem_lds_2d.lua`:
```lua
options.use_landmarks = true
```

Then convert AprilTag detections to Cartographer landmarks format.
