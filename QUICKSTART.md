# Quick Start Guide

## 1. Find Your Computer's IP
```bash
hostname -I
# Example output: 192.168.1.100
```

## 2. Choose Your Method

### Method A: Python Script (Easiest)
```bash
python receive_zigsim_data.py
```
- Configure ZigSim Pro: **JSON**, IP: `192.168.1.100`, Port: `8888`

### Method B: ROS2 (For Robotics)
```bash
./setup_ros2.sh
source ~/ros2_ws/install/setup.bash
ros2 launch teleop_iphone zigsim_osc.launch.py
```
- Configure ZigSim Pro: **JSON**, IP: `192.168.1.11`, Port: `3333`

### Method C: Test OSC Connection (Optional)
```bash
python test_osc_receiver.py
```
- Configure ZigSim Pro: **OSC**, IP: `192.168.1.11`, Port: `8000`
- Note: Only use this if ZigSim is configured for OSC protocol

## 3. Configure ZigSim Pro

On your iPhone:
1. Open **ZigSim Pro**
2. Tap **Settings** (gear icon)
3. Set **Target IP**: `192.168.1.11` (your computer's IP)
4. Set **Port**: `3333` (ROS2) or `8888` (Python)
5. Set **Protocol**: `JSON` or `UDP`
6. Go back and enable sensors (Accel, Gyro, ARKit, etc.)
7. Tap **Play** button to start streaming

## 4. Verify Data

### Python Script
You should see formatted output in the terminal with sensor values.

### ROS2
```bash
# List topics
ros2 topic list

# View IMU data
ros2 topic echo /iphone/imu

# View pose
ros2 topic echo /iphone/pose
```

### Test Script
You should see OSC messages with their addresses and values.

## Common Issues

### No data appearing?
- Check that iPhone and computer are on **same WiFi network**
- Verify IP address is correct
- Try disabling firewall temporarily:
  ```bash
  sudo ufw disable  # Re-enable with: sudo ufw enable
  ```

### "Port already in use"?
- Change port in both script and ZigSim Pro
- Python: `python receive_zigsim_data.py -p 9000`
- ROS2: `ros2 launch teleop_iphone zigsim_osc.launch.py port:=9000`

### "ROS2 not found"?
- Source ROS2 first: `source /opt/ros/humble/setup.bash`
- Then run setup script: `./setup_ros2.sh`

## Port Reference

| Tool | Default Port | Protocol |
|------|--------------|----------|
| Python Script | 8888 | JSON/UDP |
| ROS2 Node | 3333 | JSON/UDP |
| OSC Test | 8000 | OSC/UDP (optional) |

## ZigSim Pro Sensors

Enable these in ZigSim Pro:
- ✓ **Accelerometer** - Linear acceleration
- ✓ **Gyroscope** - Rotation rate
- ✓ **Magnetometer** - Compass heading
- ✓ **Attitude** - Roll/Pitch/Yaw
- ✓ **ARKit** - Camera pose (iOS 11+)
- ✓ **GPS** - Location data
- ✓ **Quaternion** - Rotation as quaternion

## Next Steps

- Read [README.md](README.md) for overview
- Read [ROS2_README.md](ROS2_README.md) for ROS2 details
- Record data: `ros2 bag record /iphone/imu /iphone/pose`
- Visualize in RViz2: `rviz2`
