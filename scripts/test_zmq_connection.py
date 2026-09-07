#!/usr/bin/env python3
"""
Simple test script to verify ZMQ connection to ARX5 arm.
Usage: python3 test_zmq_connection.py [zmq_ip] [zmq_port]
"""

import sys
import time

sys.path.append('/home/unitree/Projects/ThuDemo/arx_ros2/third_party/arx5-sdk/python')
from communication.zmq_client import Arx5Client


def main():
    # Get connection parameters from command line or use defaults
    zmq_ip = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
    zmq_port = int(sys.argv[2]) if len(sys.argv) > 2 else 5555

    print(f"Testing ZMQ connection to {zmq_ip}:{zmq_port}")
    print("-" * 50)

    try:
        # Create client
        client = Arx5Client(zmq_ip, zmq_port)
        print("✓ Connection established")

        # Get initial state
        state = client.get_state()
        print(f"\n✓ Current state:")
        print(f"  Timestamp: {state['timestamp']:.3f}")
        print(f"  EE Pose: {state['ee_pose']}")
        print(f"  Joint Pos: {state['joint_pos']}")
        print(f"  Gripper Pos: {state['gripper_pos']:.3f}")

        # Test property access
        print(f"\n✓ Property access:")
        print(f"  EE Pose (property): {client.ee_pose}")
        print(f"  TCP Pose: {client.tcp_pose}")
        print(f"  Gripper: {client.gripper_pos:.3f}")

        print("\n" + "=" * 50)
        print("✓ ZMQ connection test PASSED")
        print("=" * 50)

        return 0

    except Exception as e:
        print(f"\n✗ Connection test FAILED: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
