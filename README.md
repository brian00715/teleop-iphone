# Teleop-iPhone

This repository contains tools for receiving ARKit and sensor data from ZigSim Pro iOS app:

1. **Python UDP/JSON Receiver** - Simple standalone script for testing
2. **ROS2 OSC Node** - Full ROS2 integration for robotics applications
3. **OSC Test Receiver** - Lightweight OSC parser for debugging

## Quick Start

### Option 1: Python Script (JSON/UDP)

Simple receiver for testing and visualization:

```bash
python receive_zigsim_data.py -p 8888
```

Configure ZigSim Pro to send **JSON** data to your computer's IP on port **8888**.

### Option 2: ROS2 Package (JSON)

For ROS2 integration:

```bash
# Quick setup and build
./setup_ros2.sh

# Source workspace
source ~/ros2_ws/install/setup.bash

# Launch node
ros2 launch teleop_iphone zigsim_osc.launch.py
```

Configure ZigSim Pro to send **JSON** data to your computer's IP on port **3333** (or your configured port).

See [ROS2_README.md](ROS2_README.md) for detailed instructions.

### Option 3: OSC Test Receiver

Test OSC connectivity:

```bash
python test_osc_receiver.py 8000
```

---

## Python UDP Receiver (receive_zigsim_data.py)

Receives JSON-formatted sensor data via UDP.

### Usage

```bash
# Basic usage (default port 8888)
python receive_zigsim_data.py

# Custom port
python receive_zigsim_data.py -p 9000

# Specific host and port
python receive_zigsim_data.py -H 192.168.1.100 -p 8888

# Help
python receive_zigsim_data.py --help
```

### ZigSim Pro Configuration

1. Open ZigSim Pro on your iPhone
2. Set Protocol: **JSON** or **UDP**
3. Set Target IP: Your computer's IP address
4. Set Target Port: **8888** (or your custom port)
5. Enable desired sensors
6. Start streaming

### Data Displayed

- **ARKit Data**: Camera position, rotation, quaternion
- **Attitude**: Roll, Pitch, Yaw
- **Accelerometer**: 3-axis acceleration
- **Gyroscope**: 3-axis rotation rate
- **Magnetometer**: 3-axis magnetic field
- **GPS/Location**: Latitude, Longitude, Altitude
- **Touch**: Touch events

---

## ROS2 Package (teleop_iphone)

Full-featured ROS2 node that receives OSC data and publishes to standard ROS2 topics.

### Quick Setup

```bash
./setup_ros2.sh
```

### Published Topics

| Topic                       | Message Type                     | Description             |
| --------------------------- | -------------------------------- | ----------------------- |
| `/iphone/imu`             | `sensor_msgs/Imu`              | IMU data (accel + gyro) |
| `/iphone/pose`            | `geometry_msgs/PoseStamped`    | Device orientation      |
| `/iphone/arkit_pose`      | `geometry_msgs/PoseStamped`    | ARKit 6DOF pose         |
| `/iphone/gps`             | `sensor_msgs/NavSatFix`        | GPS location            |
| `/iphone/magnetic_field`  | `sensor_msgs/MagneticField`    | Magnetometer            |
| `/iphone/gravity`         | `geometry_msgs/Vector3Stamped` | Gravity vector          |
| `/iphone/pressure`        | `sensor_msgs/FluidPressure`    | Barometric pressure     |
| `/iphone/compass_heading` | `std_msgs/Float64`             | Compass heading         |

**See [TOPICS.md](TOPICS.md) for complete topic documentation.**

### ZigSim Pro Configuration

1. Open ZigSim Pro on your iPhone
2. Set Protocol: **JSON** or **UDP**
3. Set Target IP: Your computer's IP address
4. Set Target Port: **3333** (ROS2) or **8888** (Python)
5. Enable desired sensors
6. Start streaming

For detailed instructions, see [ROS2_README.md](ROS2_README.md).

---

## OSC Test Receiver (test_osc_receiver.py)

Lightweight script to verify OSC data is arriving correctly. Note: Use this only if ZigSim Pro is configured to send OSC data.

### Usage

```bash
# Default port 8000
python test_osc_receiver.py

# Custom port
python test_osc_receiver.py 9000
```

Configure ZigSim Pro to send **OSC** data to the specified port (if using OSC protocol).

---

## Finding Your Computer's IP Address

### Linux

```bash
hostname -I
# or
ip addr show
```

### Mac

```bash
ifconfig | grep "inet " | grep -v 127.0.0.1
```

### Windows

```cmd
ipconfig
```

Look for your local network IP (usually starts with `192.168.x.x` or `10.x.x.x`).

---

## Troubleshooting

### No Data Received

1. **Check network**: Ensure iPhone and computer are on the same WiFi network
2. **Check firewall**: Allow UDP traffic on the specified port
   ```bash
   sudo ufw allow 8000/udp
   sudo ufw allow 8888/udp
   ```
3. **Verify data is arriving**:
   ```bash
   sudo tcpdump -i any -n port 8000
   ```
4. **Check IP address**: Make sure you're using the correct IP in ZigSim Pro

### Port Already in Use

Change the port number in both the script and ZigSim Pro configuration.

---

## File Structure

```
zigsim_tele/
├── receive_zigsim_data.py    # Python UDP/JSON receiver
├── test_osc_receiver.py      # OSC test script
├── setup_ros2.sh             # ROS2 quick setup script
├── README.md                 # This file
├── ROS2_README.md           # Detailed ROS2 documentation
├── package.xml              # ROS2 package manifest
├── CMakeLists.txt           # ROS2 build configuration
├── src/
│   └── osc_node.cpp  # ROS2 node source
├── launch/
│   └── zigsim_osc.launch.py     # ROS2 launch file
└── config/
    └── zigsim_params.yaml       # ROS2 parameters
```

---

## License

MIT
