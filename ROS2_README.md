# ROS2 Package

ROS2 package for receiving JSON data from ZigSim Pro iOS app and publishing to ROS2 topics.

## Features

- Receives OSC data via UDP from ZigSim Pro
- Publishes sensor data to standard ROS2 message types
- Supports multiple sensor streams simultaneously
- Configurable UDP port and frame ID

## Supported Sensors

| ZigSim Data               | ROS2 Topic                 | Message Type                  |
| ------------------------- | -------------------------- | ----------------------------- |
| Quaternion                | `/iphone/pose`           | `geometry_msgs/PoseStamped` |
| Attitude (Roll/Pitch/Yaw) | `/iphone/pose`           | `geometry_msgs/PoseStamped` |
| Accelerometer + Gyroscope | `/iphone/imu`            | `sensor_msgs/Imu`           |
| Magnetometer              | `/iphone/magnetic_field` | `sensor_msgs/MagneticField` |
| GPS/Location              | `/iphone/gps`            | `sensor_msgs/NavSatFix`     |

## JSON Field Mappings

The node parses these JSON fields from ZigSim Pro:

- `quaternion` - Quaternion orientation `{x, y, z, w}`
- `attitude` - Euler angles `{roll, pitch, yaw}`
- `gyroscope` or `gyro` - Angular velocity `{x, y, z}`
- `accelerometer` or `accel` - Linear acceleration `{x, y, z}`
- `magnetometer` or `mag` - Magnetic field `{x, y, z}`
- `gps` or `location` - GPS coordinates `{latitude, longitude, altitude}`

## Installation

### Prerequisites

- ROS2 (Humble, Iron, or newer)
- C++ compiler with C++14 support
- colcon build tool

### Build Instructions

1. **Navigate to your ROS2 workspace** (or create one):

   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```
2. **Copy or symlink this package**:

   ```bash
   # Option 1: Copy
   cp -r /path/to/iphone_tele ./teleop_iphone

   # Option 2: Symlink
   ln -s /path/to/iphone_tele ./teleop_iphone
   ```
3. **Build the package**:

   ```bash
   cd ~/ros2_ws
   colcon build --packages-select teleop_iphone
   ```
4. **Source the workspace**:

   ```bash
   source install/setup.bash
   ```

## Usage

### Quick Start

1. **Launch the node with default settings** (port 8000):

   ```bash
   ros2 launch teleop_iphone zigsim_osc.launch.py
   ```
2. **Configure ZigSim Pro on your iPhone**:

   - Open ZigSim Pro
   - Go to Settings
   - Set Protocol: **JSON** or **UDP**
   - Set Target IP: **Your computer's IP address**
   - Set Target Port: **3333** (or your custom port)
   - Enable the sensors you want to stream
   - Start streaming
3. **View published topics**:

   ```bash
   ros2 topic list
   ros2 topic echo /iphone/imu
   ros2 topic echo /iphone/pose
   ```

### Custom Port

Launch with a custom port:

```bash
ros2 launch teleop_iphone zigsim_osc.launch.py port:=9000
```

### Custom Frame ID

Change the frame ID for the sensor data:

```bash
ros2 launch teleop_iphone zigsim_osc.launch.py frame_id:=mobile_device
```

### Run Node Directly

Run the node without launch file:

```bash
ros2 run teleop_iphone osc_node --ros-args -p port:=8000 -p frame_id:=iphone
```

### Using Parameter File

```bash
ros2 run teleop_iphone osc_node --ros-args --params-file $(ros2 pkg prefix teleop_iphone)/share/teleop_iphone/config/iphone_params.yaml
```

## Configuration

### Parameters

| Parameter    | Type   | Default     | Description                  |
| ------------ | ------ | ----------- | ---------------------------- |
| `host`     | string | `0.0.0.0` | UDP host address to bind to  |
| `port`     | int    | `8000`    | UDP port to listen on        |
| `frame_id` | string | `iphone`  | Frame ID for sensor messages |

### Modifying Config File

Edit `config/iphone_params.yaml`:

```yaml
osc_node:
  ros__parameters:
    host: "0.0.0.0"
    port: 8000
    frame_id: "iphone"
```

## Finding Your Computer's IP Address

ZigSim Pro needs your computer's IP address to send data:

```bash
# Linux
ip addr show | grep "inet "

# Or use hostname
hostname -I
```

Look for your local network IP (usually starts with `192.168.x.x` or `10.x.x.x`).

## Visualization with RViz2

Visualize IMU orientation:

```bash
rviz2
```

In RViz2:

1. Set Fixed Frame to `iphone` (or your custom frame_id)
2. Add → By topic → `/iphone/pose` → PoseStamped
3. Add → By topic → `/iphone/imu` → Imu

## Troubleshooting

### No data received

1. **Check firewall**: Make sure UDP port is open

   ```bash
   sudo ufw allow 8000/udp
   ```
2. **Verify iPhone and computer are on same network**
3. **Check if data is arriving**:

   ```bash
   sudo tcpdump -i any -n port 8000
   ```
4. **Increase log level**:

   ```bash
   ros2 run teleop_iphone osc_node --ros-args --log-level debug
   ```

### Port already in use

Change the port number:

```bash
ros2 launch teleop_iphone zigsim_osc.launch.py port:=8001
```

### Topics not publishing

Verify the node is running:

```bash
ros2 node list
ros2 node info /osc_node
```

## Integration Examples

### Record Data

Record all sensor data to a bag file:

```bash
ros2 bag record /iphone/imu /iphone/pose /iphone/gps
```

### Remap Topics

Remap topics when launching:

```bash
ros2 run teleop_iphone osc_node --ros-args -r iphone/imu:=/mobile/imu
```

### Use in Launch File

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_iphone',
            executable='osc_node',
            name='zigsim_receiver',
            parameters=[{
                'port': 8000,
                'frame_id': 'iphone'
            }]
        ),
        # Add your other nodes here
    ])
```

## License

MIT

## Contributing

Feel free to open issues or submit pull requests for improvements.
