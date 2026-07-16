#!/usr/bin/env bash
# Apply Linux kernel network tuning for reliable ROS 2 / Zenoh over WiFi (Doodle).
# Run on BOTH the robot host and the PC/monitor host before starting containers.
#
#   sudo ./scripts/tune_network_for_wifi.sh
#
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
  echo "Must run as root (sudo)." >&2
  exit 1
fi

echo "=== Kernel network tuning for WiFi/ROS2 ==="

# ── IP fragment reassembly ──────────────────────────
# Default 30s: a single lost fragment blocks the entire
# receive buffer for 30s → UDP/DDS frames pile up → hang.
# Reduce to 3s: discard incomplete fragments quickly.
sysctl -w net.ipv4.ipfrag_time=3
echo "  net.ipv4.ipfrag_time = 3s (was 30s)"

# Increase fragment reassembly memory from 256KB to 4MB.
sysctl -w net.ipv4.ipfrag_high_thresh=4194304
echo "  net.ipv4.ipfrag_high_thresh = 4MB (was 256KB)"

# ── Socket buffer sizes (UDP + TCP) ──────────────────
# Default 208KB: overflows easily with lidar/zed frames.
sysctl -w net.core.rmem_max=4194304
sysctl -w net.core.wmem_max=4194304
echo "  net.core.rmem_max = 4MB (was 208KB)"
echo "  net.core.wmem_max = 4MB (was 208KB)"

# ── TCP congestion control (BBR for WiFi) ────────────
# BBR handles variable-bandwidth, lossy WiFi links much
# better than CUBIC (which interprets packet loss as
# congestion and reduces cwnd aggressively).
if lsmod | grep -q tcp_bbr 2>/dev/null; then
  sysctl -w net.ipv4.tcp_congestion_control=bbr
  sysctl -w net.core.default_qdisc=fq
  echo "  TCP congestion control: BBR (with fq qdisc)"
else
  echo "  [SKIP] tcp_bbr module not available; keeping $(sysctl -n net.ipv4.tcp_congestion_control)"
fi

# ── TCP keepalive (detect dead WiFi faster) ──────────
# Default 7200s (2h) between keepalive probes → too slow.
sysctl -w net.ipv4.tcp_keepalive_time=60
sysctl -w net.ipv4.tcp_keepalive_intvl=10
sysctl -w net.ipv4.tcp_keepalive_probes=3
echo "  TCP keepalive: 60s idle, 10s interval, 3 probes"

echo ""
echo "=== To make permanent, run:"
echo "  cat >> /etc/sysctl.d/90-wifi-ros2.conf << 'EOF'"
echo "net.ipv4.ipfrag_time = 3"
echo "net.ipv4.ipfrag_high_thresh = 4194304"
echo "net.core.rmem_max = 4194304"
echo "net.core.wmem_max = 4194304"
echo "net.ipv4.tcp_congestion_control = bbr"
echo "net.core.default_qdisc = fq"
echo "net.ipv4.tcp_keepalive_time = 60"
echo "net.ipv4.tcp_keepalive_intvl = 10"
echo "net.ipv4.tcp_keepalive_probes = 3"
echo "EOF"
echo "  sysctl --system"
