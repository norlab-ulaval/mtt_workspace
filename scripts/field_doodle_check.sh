#!/usr/bin/env bash
set -euo pipefail

robot_ip="${DOODLE_ROBOT_IP:-192.168.50.2}"
pc_ip="${DOODLE_PC_IP:-192.168.50.101}"

echo "Doodle field check"
echo "  robot_ip=${robot_ip}"
echo "  pc_ip=${pc_ip}"
echo
echo "1) Identify the real Doodle interface before assigning any IP:"
echo "   nmcli device status"
echo "   ip -br addr"
echo "   ip neigh"
echo "   ethtool <candidate_iface>"
echo
echo "2) If unclear, unplug/replug the Doodle Ethernet cable and watch:"
echo "   watch -n 0.5 'ip -br addr; echo; nmcli device status'"
echo
echo "3) Backup before any network change:"
echo "   ip route > /tmp/routes_before_doodle.txt"
echo "   nmcli connection show > /tmp/nmcli_connections_before_doodle.txt"
echo
echo "4) Only after identifying the interface:"
echo "   Robot side: set ${robot_ip}/24 on the Doodle-connected interface only."
echo "   PC side:    set ${pc_ip}/24 on the Doodle-connected interface only."
echo
echo "5) Verify after every change:"
echo "   ip route get 1.1.1.1"
echo "   ping -c 3 1.1.1.1"
echo
echo "6) Emergency Internet rollback on robot:"
echo "   sudo ip route replace default via 192.168.2.101 dev enp5s0"
echo
echo "7) Doodle link test:"
echo "   ping -c 5 ${pc_ip}      # from robot"
echo "   ping -c 5 ${robot_ip}   # from PC"
echo "   iperf3 -s               # PC"
echo "   iperf3 -c ${pc_ip}      # robot"
echo "   iperf3 -c ${pc_ip} -R   # robot reverse test"
