#!/bin/bash
# Debug script to check UDP connectivity

PORT=${1:-3333}

echo "==================================="
echo "ZigSim UDP Debug Tool"
echo "==================================="
echo ""

echo "1. Checking if port $PORT is listening..."
ss -uln | grep ":$PORT" || echo "   ⚠ Port $PORT is NOT listening"
echo ""

echo "2. Checking firewall status..."
sudo ufw status | grep "$PORT" || echo "   ⚠ No firewall rule found for port $PORT"
echo ""

echo "3. Network interfaces and IPs:"
ip -4 addr show | grep inet
echo ""

echo "4. Testing UDP reception (press Ctrl+C to stop)..."
echo "   Run this on your iPhone to test:"
echo "   echo 'test' | nc -u YOUR_COMPUTER_IP $PORT"
echo ""
echo "Listening on port $PORT..."
nc -u -l -p $PORT 2>/dev/null || netcat -u -l -p $PORT 2>/dev/null || {
    echo "⚠ nc/netcat not available, trying socat..."
    socat -u UDP-LISTEN:$PORT,reuseaddr -
} || {
    echo "⚠ No UDP listening tools available (nc, netcat, socat)"
    echo "   Using tcpdump instead (requires sudo)..."
    sudo tcpdump -i any -n port $PORT
}
