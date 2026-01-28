#!/usr/bin/env python3
"""
Raw data inspector - shows exactly what ZigSim Pro is sending
"""

import socket
import sys


def hex_dump(data, length=64):
    """Print hex dump of data"""
    preview = data[:length]

    # Hex representation
    hex_str = " ".join(f"{b:02x}" for b in preview)

    # ASCII representation
    ascii_str = "".join(chr(b) if 32 <= b < 127 else "." for b in preview)

    return hex_str, ascii_str


def main():
    port = 3333
    if len(sys.argv) > 1:
        port = int(sys.argv[1])

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", port))

    print(f"Raw Data Inspector - Listening on port {port}")
    print("=" * 70)
    print()

    packet_count = 0
    try:
        while True:
            data, addr = sock.recvfrom(4096)
            packet_count += 1

            print(f"Packet #{packet_count} from {addr[0]}:{addr[1]} - {len(data)} bytes")
            print("-" * 70)

            # Show first few bytes
            hex_str, ascii_str = hex_dump(data, 64)
            print(f"HEX (first 64 bytes):")
            print(hex_str)
            print()
            print(f"ASCII:")
            print(ascii_str)
            print()

            # Try to detect format
            if data[0:1] == b"{":
                print("Format: Looks like JSON")
                try:
                    import json

                    j = json.loads(data.decode("utf-8"))
                    print(f"JSON Keys: {list(j.keys())}")
                except:
                    print("Failed to parse as JSON")
            elif data[0:1] == b"#":
                print("Format: OSC Bundle (starts with '#bundle')")
            elif data[0:1] == b"/":
                print("Format: OSC Message (starts with '/')")
            else:
                print(f"Format: Unknown (first byte: 0x{data[0]:02x})")

            print("=" * 70)
            print()

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
