# ROS2 Topics

Complete list of all published topics and their message types.

## Published Topics

| Topic                       | Message Type                     | Rate  | Description                                        |
| --------------------------- | -------------------------------- | ----- | -------------------------------------------------- |
| `/iphone/imu`             | `sensor_msgs/Imu`              | ~60Hz | Combined IMU data (accelerometer + gyroscope)      |
| `/iphone/pose`            | `geometry_msgs/PoseStamped`    | ~60Hz | Device orientation (quaternion from device sensor) |
| `/iphone/arkit_pose`      | `geometry_msgs/PoseStamped`    | ~60Hz | ARKit 6DOF pose (position + orientation)           |
| `/iphone/gps`             | `sensor_msgs/NavSatFix`        | ~1Hz  | GPS location (latitude, longitude, altitude)       |
| `/iphone/magnetic_field`  | `sensor_msgs/MagneticField`    | ~60Hz | Magnetometer 3-axis data                           |
| `/iphone/gravity`         | `geometry_msgs/Vector3Stamped` | ~60Hz | Gravity vector                                     |
| `/iphone/pressure`        | `sensor_msgs/FluidPressure`    | ~1Hz  | Barometric pressure (in Pascals)                   |
| `/iphone/compass_heading` | `std_msgs/Float64`             | ~60Hz | Compass heading (degrees, 0-360)                   |

## Topic Details

### IMU (`/iphone/imu`)

Contains:

- **Linear acceleration** (m/s²): `linear_acceleration.{x,y,z}`
- **Angular velocity** (rad/s): `angular_velocity.{x,y,z}`
- **Orientation** (optional): `orientation.{x,y,z,w}`

### Pose (`/iphone/pose`)

Quaternion orientation from device motion sensor:

- Device quaternion sensor data
- Format: `pose.orientation.{x,y,z,w}`
- Position is set to origin (0,0,0)

### ARKit Pose (`/iphone/arkit_pose`)

Full 6DOF pose from ARKit (position + orientation):

- **Position** (meters): `pose.position.{x,y,z}` - Camera position in world space
- **Orientation** (quaternion): `pose.orientation.{x,y,z,w}` - Camera rotation
- Only published when ARKit tracking is active
- Provides both translation and rotation in a single message

### GPS (`/iphone/gps`)

GPS location data:

- **Latitude** (degrees): `latitude`
- **Longitude** (degrees): `longitude`
- **Altitude** (meters): `altitude`
- **Status**: `status.status` (STATUS_FIX when valid)

### Magnetic Field (`/iphone/magnetic_field`)

3-axis magnetometer readings:

- **Magnetic field** (Tesla): `magnetic_field.{x,y,z}`

### Gravity (`/iphone/gravity`)

Gravity vector in device frame:

- **Gravity** (g-force): `vector.{x,y,z}`
- Normalized to ~1.0 magnitude

### Pressure (`/iphone/pressure`)

Barometric pressure:

- **Pressure** (Pascals): `fluid_pressure`
- Note: Converted from hPa (multiply by 100)

### Compass Heading (`/iphone/compass_heading`)

Magnetic heading:

- **Heading** (degrees): `data`
- Range: 0-360, where 0/360 is North

## Viewing Topics

### List all topics:

```bash
ros2 topic list
```

### Echo a specific topic:

```bash
# IMU data
ros2 topic echo /iphone/imu

# Device orientation
ros2 topic echo /iphone/pose

# ARKit 6DOF pose (position + orientation)
ros2 topic echo /iphone/arkit_pose

# GPS location
ros2 topic echo /iphone/gps

# Gravity vector
ros2 topic echo /iphone/gravity

# Compass heading
ros2 topic echo /iphone/compass_heading
```

### Check topic rates:

```bash
ros2 topic hz /iphone/imu
ros2 topic hz /iphone/pose
```

### Topic info:

```bash
ros2 topic info /iphone/imu -v
```

## Recording Data

### Record all ZigSim topics:

```bash
ros2 bag record /iphone/imu /iphone/pose /iphone/arkit_pose /iphone/gps /iphone/gravity /iphone/pressure /iphone/compass_heading /iphone/magnetic_field
```

### Record specific topics:

```bash
# Record only IMU and ARKit pose
ros2 bag record /iphone/imu /iphone/arkit_pose

# Record with custom name
ros2 bag record -o my_recording /iphone/imu /iphone/arkit_pose

# Record all pose data (device + ARKit)
ros2 bag record /iphone/pose /iphone/arkit_pose
```

## Visualization in RViz2

### Launch RViz2:

```bash
rviz2
```

### Configure:

1. Set **Fixed Frame** to `iphone` (or your custom frame_id)
2. Add displays:
   - **Imu**: Add → By topic → `/iphone/imu` → Imu
   - **Device Pose**: Add → By topic → `/iphone/pose` → PoseStamped
   - **ARKit Pose**: Add → By topic → `/iphone/arkit_pose` → PoseStamped
   - **GPS**: Add → By topic → `/iphone/gps` → NavSatFix
3. Configure ARKit Pose display:
   - Set **Axes Length** to 0.1 to visualize orientation
   - Set **Axes Radius** to 0.01
   - This shows both position and orientation of the camera

## Example: Monitor All Data

Create a simple monitor script:

```bash
#!/bin/bash
# monitor_zigsim.sh

# Open multiple terminals with topic echoing
gnome-terminal --tab --title="IMU" -- bash -c "ros2 topic echo /iphone/imu; bash"
gnome-terminal --tab --title="Pose" -- bash -c "ros2 topic echo /iphone/pose; bash"
gnome-terminal --tab --title="GPS" -- bash -c "ros2 topic echo /iphone/gps; bash"
gnome-terminal --tab --title="Gravity" -- bash -c "ros2 topic echo /iphone/gravity; bash"
```

## Frame ID

All topics use the same frame ID (default: `iphone`), configurable via the `frame_id` parameter:

```bash
ros2 launch teleop_iphone zigsim_osc.launch.py frame_id:=mobile_device
```

## Troubleshooting

### No topics appearing?

1. Check if node is running: `ros2 node list`
2. Verify ZigSim Pro is streaming (check phone)
3. Check firewall: `sudo ufw allow 3333/udp`

### Topics not updating?

```bash
# Check topic rates (should be >0 Hz)
ros2 topic hz /iphone/imu

# Check for errors in node
ros2 node info /osc_node
```

### Incorrect data?

```bash
# Enable debug logging
ros2 run teleop_iphone osc_node --ros-args --log-level debug
```
