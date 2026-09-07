# Teleop iPhone - ZMQ Version

Python implementation of iPhone teleoperation using ZMQ communication with the ARX5 arm.

## Overview

This is a Python version of the teleop_iphone node that uses ZMQ to communicate with the ARX5 robotic arm instead of ROS2 services and topics. It maintains the same control logic but communicates directly with the arm's ZMQ server.

## Features

- **Touch-based control**: Control the arm using iPhone touch input
- **Relative motion**: Motion is relative to the pose when touch begins
- **Gripper control**:
  - 1 finger touch: gripper closes
  - 2+ finger touch: gripper opens
  - 3 finger touch: reset arm to home position
- **ZMQ communication**: Direct communication with ARX5 arm via ZMQ (no ROS2 arm driver needed)
- **Configurable scaling**: Position and rotation scaling factors

## Requirements

- ROS2 (for iPhone data and TF)
- Python 3
- NumPy
- ZMQ (pyzmq)
- tf_transformations
- ARX5 SDK with ZMQ server running

## Installation

1. Build the package:
```bash
cd ~/Projects/ThuDemo
colcon build --packages-select teleop_iphone
source install/setup.bash
```

2. Make sure the ARX5 ZMQ server is running:
```bash
# Start the ARX5 ZMQ server (refer to arx5-sdk documentation)
python3 /path/to/arx5-sdk/python/server.py
```

## Usage

### Quick Start

```bash
# Launch the ZMQ version
ros2 launch teleop_iphone teleop_zmq.launch.py
```

### Configuration

Edit `config/teleop_iphone_zmq.yaml` to customize:

```yaml
teleop_iphone_zmq:
  ros__parameters:
    # ZMQ connection
    zmq_ip: "127.0.0.1"
    zmq_port: 5555

    # Frame names
    iphone_source_frame: "iphone_odom"
    iphone_target_frame: "iphone_ros"
    arm_base_frame: "base_link"
    arm_ee_frame: "link6"

    # Control parameters
    touch_threshold: 0.3        # Minimum touch radius to detect
    touch_timeout: 0.05         # Timeout for touch data (seconds)
    position_scale: 1.0         # Position mapping scale
    rotation_scale: 0.5         # Rotation mapping scale

    # Gripper positions
    gripper_open: 0.08
    gripper_close: 0.0

    # Update rates
    tf_update_rate: 100.0       # Hz
    control_rate: 50            # Hz
```

### Running Standalone

```bash
ros2 run teleop_iphone teleop_iphone_zmq.py --ros-args --params-file config/teleop_iphone_zmq.yaml
```

## Architecture

```
┌─────────────┐
│   iPhone    │
│  (Touch +   │
│   TF data)  │
└──────┬──────┘
       │ ROS2 Topics/TF
       ▼
┌─────────────────────┐
│ teleop_iphone_zmq   │
│  (Python Node)      │
│  - Touch callback   │
│  - TF monitoring    │
│  - Control logic    │
└──────┬──────────────┘
       │ ZMQ (REQ-REP)
       ▼
┌─────────────────────┐
│   ARX5 ZMQ Server   │
│  (arx5-sdk)         │
│  - set_ee_pose()    │
│  - reset_to_home()  │
└──────┬──────────────┘
       │
       ▼
┌─────────────────────┐
│     ARX5 Arm        │
└─────────────────────┘
```

## Control Logic

1. **Touch Detection**: Monitors `/iphone/touch` topic for touch input
2. **TF Monitoring**: Continuously updates iPhone and arm end-effector poses via TF
3. **On First Touch**: Records start poses (iPhone and EE)
4. **During Touch**:
   - Calculates delta from iPhone start pose
   - Applies scaled delta to EE start pose
   - Sends target pose to arm via ZMQ
5. **On Release**: Stops sending commands

## Comparison with ROS2 Version

| Feature | ROS2 Version | ZMQ Version |
|---------|--------------|-------------|
| Arm Communication | ROS2 topics/services | ZMQ REQ-REP |
| iPhone Data | ROS2 topics/TF | ROS2 topics/TF |
| Dependencies | Full ROS2 stack | ROS2 (for iPhone) + ZMQ |
| Latency | Higher (ROS2 overhead) | Lower (direct ZMQ) |
| Configuration | YAML params | YAML params |

## Troubleshooting

### "Could not get iPhone transform"
- Make sure iPhone data is being published
- Check that `iphone_source_frame` and `iphone_target_frame` are correct

### "Could not get arm transform"
- Verify that arm TF is being published
- Check that `arm_base_frame` and `arm_ee_frame` match your setup

### ZMQ Connection Error
- Ensure ARX5 ZMQ server is running
- Verify `zmq_ip` and `zmq_port` match the server configuration
- Check firewall settings if using remote connection

### Touch not detected
- Adjust `touch_threshold` parameter
- Check that `/iphone/touch` topic is publishing data

## Notes

- The gripper logic is inverted from the C++ version to match typical expectations:
  - 1 finger = close (grasp)
  - 2+ fingers = open (release)
- 3-finger touch triggers a reset with 2-second cooldown
- All motion is relative to the pose when touch begins
- TF updates run at high rate (100Hz) to minimize latency
- Control commands are sent at 50Hz by default
