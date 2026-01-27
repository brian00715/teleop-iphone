#!/usr/bin/env python3
"""
Simple OSC receiver for testing ZigSim Pro connection
Run this to verify OSC data is arriving before testing the ROS2 node
"""

import socket
import struct
import sys

def parse_osc_string(data, pos):
    """Parse OSC string (null-terminated, padded to 4-byte boundary)"""
    end = data.find(b'\x00', pos)
    if end == -1:
        return None, pos
    string = data[pos:end].decode('utf-8', errors='ignore')
    # Align to 4-byte boundary
    pos = ((end + 4) // 4) * 4
    return string, pos

def parse_osc_message(data):
    """Parse OSC message and return address and arguments"""
    try:
        # Parse address
        address, pos = parse_osc_string(data, 0)
        if not address or pos >= len(data):
            return None

        # Parse type tag string
        if data[pos:pos+1] != b',':
            return None
        pos += 1

        type_tags, pos = parse_osc_string(data, pos - 1)
        if not type_tags:
            return None
        type_tags = type_tags[1:]  # Remove leading comma

        # Parse arguments
        args = []
        for tag in type_tags:
            if pos + 4 > len(data):
                break

            if tag == 'f':  # float
                value = struct.unpack('>f', data[pos:pos+4])[0]
                args.append(value)
                pos += 4
            elif tag == 'i':  # int32
                value = struct.unpack('>i', data[pos:pos+4])[0]
                args.append(value)
                pos += 4
            elif tag == 's':  # string
                string, pos = parse_osc_string(data, pos)
                args.append(string)

        return {'address': address, 'args': args}

    except Exception as e:
        print(f"Parse error: {e}")
        return None

def main():
    port = 8000
    if len(sys.argv) > 1:
        port = int(sys.argv[1])

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', port))

    print(f"OSC Test Receiver listening on port {port}")
    print("Configure ZigSim Pro to send OSC data to this computer's IP")
    print("Press Ctrl+C to stop\n")

    try:
        while True:
            data, addr = sock.recvfrom(4096)
            msg = parse_osc_message(data)

            if msg:
                args_str = ', '.join([f'{arg:.4f}' if isinstance(arg, float) else str(arg)
                                     for arg in msg['args']])
                print(f"[{addr[0]}:{addr[1]}] {msg['address']} -> [{args_str}]")
            else:
                print(f"[{addr[0]}:{addr[1]}] Received {len(data)} bytes (failed to parse)")

    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        sock.close()

if __name__ == '__main__':
    main()
