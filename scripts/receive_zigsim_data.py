#!/usr/bin/env python3
"""
ZigSim Pro UDP Data Receiver
Receives ARKit and sensor data from ZigSim Pro iOS app via UDP
"""

import socket
import json
import argparse
from datetime import datetime


def parse_zigsim_data(data):
    """Parse and format ZigSim Pro data packet"""
    try:
        # ZigSim Pro sends JSON formatted data
        json_data = json.loads(data.decode('utf-8'))
        return json_data
    except json.JSONDecodeError:
        # If not JSON, try to decode as plain text
        return {"raw": data.decode('utf-8', errors='ignore')}
    except Exception as e:
        return {"error": str(e), "raw": str(data)}


def print_formatted_data(data):
    """Print data in a readable format"""
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    print(f"\n{'='*60}")
    print(f"[{timestamp}] Received Data:")
    print(f"{'='*60}")

    # Common ZigSim Pro data fields
    if isinstance(data, dict):
        # ARKit / Device Pose
        if 'attitude' in data:
            att = data['attitude']
            print(f"Attitude (Roll/Pitch/Yaw): {att.get('roll', 'N/A'):.4f}, "
                  f"{att.get('pitch', 'N/A'):.4f}, {att.get('yaw', 'N/A'):.4f}")

        if 'quaternion' in data:
            q = data['quaternion']
            print(f"Quaternion (x,y,z,w): {q.get('x', 'N/A'):.4f}, "
                  f"{q.get('y', 'N/A'):.4f}, {q.get('z', 'N/A'):.4f}, "
                  f"{q.get('w', 'N/A'):.4f}")

        # Accelerometer
        if 'accelerometer' in data or 'accel' in data:
            accel = data.get('accelerometer') or data.get('accel')
            print(f"Accelerometer (x,y,z): {accel.get('x', 'N/A'):.4f}, "
                  f"{accel.get('y', 'N/A'):.4f}, {accel.get('z', 'N/A'):.4f}")

        # Gyroscope
        if 'gyroscope' in data or 'gyro' in data:
            gyro = data.get('gyroscope') or data.get('gyro')
            print(f"Gyroscope (x,y,z): {gyro.get('x', 'N/A'):.4f}, "
                  f"{gyro.get('y', 'N/A'):.4f}, {gyro.get('z', 'N/A'):.4f}")

        # Magnetometer
        if 'magnetometer' in data or 'mag' in data:
            mag = data.get('magnetometer') or data.get('mag')
            print(f"Magnetometer (x,y,z): {mag.get('x', 'N/A'):.4f}, "
                  f"{mag.get('y', 'N/A'):.4f}, {mag.get('z', 'N/A'):.4f}")

        # GPS
        if 'gps' in data or 'location' in data:
            gps = data.get('gps') or data.get('location')
            print(f"GPS: Lat={gps.get('latitude', 'N/A')}, "
                  f"Lon={gps.get('longitude', 'N/A')}, "
                  f"Alt={gps.get('altitude', 'N/A')}")

        # ARKit Camera Transform
        if 'camera' in data:
            cam = data['camera']
            if 'position' in cam:
                pos = cam['position']
                print(f"Camera Position (x,y,z): {pos.get('x', 'N/A'):.4f}, "
                      f"{pos.get('y', 'N/A'):.4f}, {pos.get('z', 'N/A'):.4f}")
            if 'eulerAngles' in cam:
                euler = cam['eulerAngles']
                print(f"Camera Euler (x,y,z): {euler.get('x', 'N/A'):.4f}, "
                      f"{euler.get('y', 'N/A'):.4f}, {euler.get('z', 'N/A'):.4f}")

        # Touch data
        if 'touch' in data:
            touch = data['touch']
            print(f"Touch: {touch}")

        # Print full JSON for debugging
        print(f"\nFull JSON Data:")
        print(json.dumps(data, indent=2))
    else:
        print(data)


def start_udp_server(host='0.0.0.0', port=8888, buffer_size=4096):
    """Start UDP server to receive ZigSim Pro data"""

    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    # Bind to address
    server_address = (host, port)
    print(f"Starting ZigSim Pro UDP receiver on {host}:{port}")
    print(f"Configure your iPhone ZigSim Pro app to send data to this address")
    print(f"Press Ctrl+C to stop\n")

    try:
        sock.bind(server_address)

        while True:
            # Receive data
            data, client_address = sock.recvfrom(buffer_size)

            if data:
                # Parse and print the data
                parsed_data = parse_zigsim_data(data)
                print_formatted_data(parsed_data)

    except KeyboardInterrupt:
        print("\n\nShutting down UDP receiver...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        sock.close()


def main():
    parser = argparse.ArgumentParser(
        description='Receive ARKit and sensor data from ZigSim Pro via UDP',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python receive_zigsim_data.py                    # Use defaults (0.0.0.0:8888)
  python receive_zigsim_data.py -p 9000            # Custom port
  python receive_zigsim_data.py -H 192.168.1.100   # Specific host
  python receive_zigsim_data.py -H 0.0.0.0 -p 8888 # Explicit host and port
        """
    )

    parser.add_argument(
        '-H', '--host',
        type=str,
        default='0.0.0.0',
        help='UDP host address to bind to (default: 0.0.0.0 for all interfaces)'
    )

    parser.add_argument(
        '-p', '--port',
        type=int,
        default=8888,
        help='UDP port to listen on (default: 8888)'
    )

    parser.add_argument(
        '-b', '--buffer-size',
        type=int,
        default=4096,
        help='UDP buffer size in bytes (default: 4096)'
    )

    args = parser.parse_args()

    start_udp_server(
        host=args.host,
        port=args.port,
        buffer_size=args.buffer_size
    )


if __name__ == '__main__':
    main()
