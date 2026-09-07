#!/usr/bin/env python3

import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from tf2_ros import Buffer, TransformException, TransformListener
import tf_transformations

# Add the path to arx5-sdk python module
sys.path.append('/home/unitree/Projects/ThuDemo/arx_ros2/third_party/arx5-sdk/python')
from communication.zmq_client import Arx5Client


class TeleopIphoneZMQ(Node):
    def __init__(self):
        super().__init__('teleop_iphone_zmq')

        # Declare parameters
        self.declare_parameter('zmq_ip', '127.0.0.1')
        self.declare_parameter('zmq_port', 5555)
        self.declare_parameter('iphone_source_frame', 'iphone_odom')
        self.declare_parameter('iphone_target_frame', 'iphone_ros')
        self.declare_parameter('arm_base_frame', 'base_link')
        self.declare_parameter('arm_ee_frame', 'link6')
        self.declare_parameter('touch_threshold', 0.3)
        self.declare_parameter('touch_timeout', 0.05)
        self.declare_parameter('position_scale', 1.0)
        self.declare_parameter('rotation_scale', 1.0)
        self.declare_parameter('gripper_open', 0.3)
        self.declare_parameter('gripper_close', 0.0)
        self.declare_parameter('tf_update_rate', 100.0)
        self.declare_parameter('control_rate', 50)

        # Get parameters
        zmq_ip = self.get_parameter('zmq_ip').value
        zmq_port = self.get_parameter('zmq_port').value
        self.iphone_source_frame = self.get_parameter('iphone_source_frame').value
        self.iphone_target_frame = self.get_parameter('iphone_target_frame').value
        self.arm_base_frame = self.get_parameter('arm_base_frame').value
        self.arm_ee_frame = self.get_parameter('arm_ee_frame').value
        self.touch_threshold = self.get_parameter('touch_threshold').value
        self.touch_timeout = self.get_parameter('touch_timeout').value
        self.position_scale = self.get_parameter('position_scale').value
        self.rotation_scale = self.get_parameter('rotation_scale').value
        self.gripper_open = self.get_parameter('gripper_open').value
        self.gripper_close = self.get_parameter('gripper_close').value
        tf_update_rate = self.get_parameter('tf_update_rate').value
        control_rate = self.get_parameter('control_rate').value

        # Initialize ZMQ client for arm control
        self.get_logger().info(f'Connecting to arm via ZMQ at {zmq_ip}:{zmq_port}')
        self.arx_client = Arx5Client(zmq_ip, zmq_port)
        self.get_logger().info('ZMQ connection established')

        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Locks for thread safety
        self.tf_lock = threading.Lock()
        self.touch_lock = threading.Lock()

        # TF state variables
        self.iphone_tf_valid = False
        self.iphone_x = 0.0
        self.iphone_y = 0.0
        self.iphone_z = 0.0
        self.iphone_roll = 0.0
        self.iphone_pitch = 0.0
        self.iphone_yaw = 0.0

        self.arm_tf_valid = False
        self.ee_x = 0.0
        self.ee_y = 0.0
        self.ee_z = 0.0
        self.ee_roll = 0.0
        self.ee_pitch = 0.0
        self.ee_yaw = 0.0

        # Touch state variables
        self.has_valid_touch = False
        self.touch_count = 0
        self.has_touch_message = False
        self.last_touch_time = self.get_clock().now()

        # Control state
        self.is_pressing = False
        self.reset_sent = False
        self.reset_block_active = False
        self.reset_cooldown_until = self.get_clock().now()

        # Start poses (recorded when touch begins)
        self.start_iphone_x = 0.0
        self.start_iphone_y = 0.0
        self.start_iphone_z = 0.0
        self.start_iphone_roll = 0.0
        self.start_iphone_pitch = 0.0
        self.start_iphone_yaw = 0.0

        self.start_ee_x = 0.0
        self.start_ee_y = 0.0
        self.start_ee_z = 0.0
        self.start_ee_roll = 0.0
        self.start_ee_pitch = 0.0
        self.start_ee_yaw = 0.0

        # Subscribers
        self.touch_sub = self.create_subscription(
            PoseArray,
            '/iphone/touch',
            self.touch_callback,
            10
        )

        # Timers
        self.tf_timer = self.create_timer(1.0 / tf_update_rate, self.tf_timer_callback)
        self.control_timer = self.create_timer(1.0 / control_rate, self.control_timer_callback)

        self.get_logger().info('TeleopIphoneZMQ node started')
        self.get_logger().info(f'  iPhone frames: {self.iphone_source_frame} -> {self.iphone_target_frame}')
        self.get_logger().info(f'  Arm frames: {self.arm_base_frame} -> {self.arm_ee_frame}')
        self.get_logger().info(f'  Touch threshold: {self.touch_threshold:.2f}')
        self.get_logger().info(f'  Touch timeout: {self.touch_timeout:.2f} s')
        self.get_logger().info(f'  Position scale: {self.position_scale:.2f}')
        self.get_logger().info(f'  Rotation scale: {self.rotation_scale:.2f}')
        self.get_logger().info(f'  TF update rate: {tf_update_rate:.1f} Hz')
        self.get_logger().info(f'  Control rate: {control_rate} Hz')

    def tf_timer_callback(self):
        """Update TF transforms at high rate"""
        with self.tf_lock:
            # Update iPhone TF
            try:
                iphone_tf = self.tf_buffer.lookup_transform(
                    self.iphone_source_frame,
                    self.iphone_target_frame,
                    rclpy.time.Time()
                )

                self.iphone_x = iphone_tf.transform.translation.x
                self.iphone_y = iphone_tf.transform.translation.y
                self.iphone_z = iphone_tf.transform.translation.z

                q = [
                    iphone_tf.transform.rotation.x,
                    iphone_tf.transform.rotation.y,
                    iphone_tf.transform.rotation.z,
                    iphone_tf.transform.rotation.w
                ]
                self.iphone_roll, self.iphone_pitch, self.iphone_yaw = tf_transformations.euler_from_quaternion(q)

                self.iphone_tf_valid = True
            except TransformException as ex:
                if not hasattr(self, '_last_iphone_warn') or time.time() - self._last_iphone_warn > 2.0:
                    self.get_logger().warn(f'Could not get iPhone transform: {ex}')
                    self._last_iphone_warn = time.time()
                self.iphone_tf_valid = False

            # Update arm EE TF
            try:
                arm_tf = self.tf_buffer.lookup_transform(
                    self.arm_base_frame,
                    self.arm_ee_frame,
                    rclpy.time.Time()
                )

                self.ee_x = arm_tf.transform.translation.x
                self.ee_y = arm_tf.transform.translation.y
                self.ee_z = arm_tf.transform.translation.z

                q = [
                    arm_tf.transform.rotation.x,
                    arm_tf.transform.rotation.y,
                    arm_tf.transform.rotation.z,
                    arm_tf.transform.rotation.w
                ]
                self.ee_roll, self.ee_pitch, self.ee_yaw = tf_transformations.euler_from_quaternion(q)

                self.arm_tf_valid = True
            except TransformException as ex:
                if not hasattr(self, '_last_arm_warn') or time.time() - self._last_arm_warn > 2.0:
                    self.get_logger().warn(f'Could not get arm transform: {ex}')
                    self._last_arm_warn = time.time()
                self.arm_tf_valid = False

    def control_timer_callback(self):
        """Main control loop"""
        now = self.get_clock().now()

        # Check reset cooldown
        if self.reset_block_active:
            if now >= self.reset_cooldown_until:
                self.reset_block_active = False
            else:
                return

        # Get touch state snapshot
        with self.touch_lock:
            has_valid_touch = self.has_valid_touch
            touch_count = self.touch_count
            has_touch_msg = self.has_touch_message
            last_touch_time = self.last_touch_time

        # Check touch timeout
        if not has_touch_msg or (now - last_touch_time).nanoseconds * 1e-9 > self.touch_timeout:
            if self.is_pressing:
                if not hasattr(self, '_last_timeout_warn') or time.time() - self._last_timeout_warn > 1.0:
                    self.get_logger().info('Touch data timeout')
                    self._last_timeout_warn = time.time()
            self.is_pressing = False
            return

        # Handle release
        if not has_valid_touch:
            if self.is_pressing:
                self.get_logger().info('Touch released')
            self.is_pressing = False
            self.reset_sent = False
            return

        # Handle 3-finger reset
        if touch_count == 3:
            if not self.reset_sent:
                self.reset_block_active = True
                self.reset_cooldown_until = now + rclpy.duration.Duration(seconds=2.0)
                self.send_reset_to_home()
                self.reset_sent = True
            return

        # Get TF snapshots
        with self.tf_lock:
            iphone_tf_valid = self.iphone_tf_valid
            arm_tf_valid = self.arm_tf_valid
            if iphone_tf_valid:
                iphone_x = self.iphone_x
                iphone_y = self.iphone_y
                iphone_z = self.iphone_z
                iphone_roll = self.iphone_roll
                iphone_pitch = self.iphone_pitch
                iphone_yaw = self.iphone_yaw
            if arm_tf_valid:
                ee_x = self.ee_x
                ee_y = self.ee_y
                ee_z = self.ee_z
                ee_roll = self.ee_roll
                ee_pitch = self.ee_pitch
                ee_yaw = self.ee_yaw

        if not iphone_tf_valid:
            if not hasattr(self, '_last_iphone_warn2') or time.time() - self._last_iphone_warn2 > 1.0:
                self.get_logger().warn('iPhone TF not available')
                self._last_iphone_warn2 = time.time()
            return

        # Handle first press - record start poses
        if not self.is_pressing:
            if not arm_tf_valid:
                self.get_logger().warn('Arm TF not available on press')
                return

            self.is_pressing = True
            self.reset_sent = False

            # Record iPhone start pose
            self.start_iphone_x = iphone_x
            self.start_iphone_y = iphone_y
            self.start_iphone_z = iphone_z
            self.start_iphone_roll = iphone_roll
            self.start_iphone_pitch = iphone_pitch
            self.start_iphone_yaw = iphone_yaw

            # Record EE start pose
            self.start_ee_x = ee_x
            self.start_ee_y = ee_y
            self.start_ee_z = ee_z
            self.start_ee_roll = ee_roll
            self.start_ee_pitch = ee_pitch
            self.start_ee_yaw = ee_yaw

            self.get_logger().info(
                f'Touch started - iPhone: [{self.start_iphone_x:.3f}, {self.start_iphone_y:.3f}, {self.start_iphone_z:.3f}], '
                f'EE: [{self.start_ee_x:.3f}, {self.start_ee_y:.3f}, {self.start_ee_z:.3f}]'
            )

        # Calculate relative motion from iPhone start pose
        delta_x = (iphone_x - self.start_iphone_x) * self.position_scale
        delta_y = (iphone_y - self.start_iphone_y) * self.position_scale
        delta_z = (iphone_z - self.start_iphone_z) * self.position_scale
        delta_roll = (iphone_roll - self.start_iphone_roll) * self.rotation_scale
        delta_pitch = (iphone_pitch - self.start_iphone_pitch) * self.rotation_scale
        delta_yaw = (iphone_yaw - self.start_iphone_yaw) * self.rotation_scale

        # Apply delta to EE start pose
        target_x = self.start_ee_x + delta_x
        target_y = self.start_ee_y + delta_y
        target_z = self.start_ee_z + delta_z
        target_roll = self.start_ee_roll + delta_roll
        target_pitch = self.start_ee_pitch + delta_pitch
        target_yaw = self.start_ee_yaw + delta_yaw

        # Gripper control: 2 touches = open, 1 touch = close
        gripper_position = self.gripper_open if touch_count >= 2 else self.gripper_close

        # Send command via ZMQ
        target_pose = np.array([target_x, target_y, target_z, target_roll, target_pitch, target_yaw])
        try:
            self.arx_client.set_ee_pose(target_pose, gripper_position)

            if not hasattr(self, '_last_log_time') or time.time() - self._last_log_time > 0.5:
                self.get_logger().info(
                    f'Cmd: pos[{target_x:.3f}, {target_y:.3f}, {target_z:.3f}] '
                    f'delta[{delta_x:.3f}, {delta_y:.3f}, {delta_z:.3f}] '
                    f'gripper={gripper_position:.3f} touches={touch_count}'
                )
                self._last_log_time = time.time()
        except Exception as e:
            self.get_logger().error(f'Failed to send command: {e}')

    def send_reset_to_home(self):
        """Send reset to home command via ZMQ"""
        try:
            self.get_logger().info('Sending reset to home command')
            self.arx_client.reset_to_home()
            self.get_logger().info('Reset to home succeeded')
            self.reset_block_active = True
            self.reset_cooldown_until = self.get_clock().now() + rclpy.duration.Duration(seconds=2.0)
        except Exception as e:
            self.get_logger().error(f'Reset to home failed: {e}')

    def touch_callback(self, msg: PoseArray):
        """Handle touch messages from iPhone"""
        # Check if any touch has radius > threshold (stored in orientation.w)
        has_valid_touch = False
        touch_count = 0

        if msg.poses:
            for pose in msg.poses:
                if pose.orientation.w > self.touch_threshold:
                    has_valid_touch = True
                    touch_count += 1

        with self.touch_lock:
            self.has_valid_touch = has_valid_touch
            self.touch_count = touch_count
            self.last_touch_time = self.get_clock().now()
            self.has_touch_message = True


def main(args=None):
    rclpy.init(args=args)
    node = TeleopIphoneZMQ()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
