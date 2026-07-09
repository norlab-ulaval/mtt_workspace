# Doodle Field Network Handoff

This note captures the current Doodle / Mesh Rider setup after the July 2026
field-network debugging session. It is meant for the next operator, teammate,
or AI agent continuing the work.

## Current working state

The Doodle link is working between the operator computer and the robot.

| Side | Interface | IP address | Role |
| --- | --- | --- | --- |
| Operator computer | `enp152s0` | `192.168.50.101/24` | Doodle Ethernet profile, PC side |
| Operator computer | `enp152s0` | `10.223.1.101/16` | Temporary management IP for Doodle web UI |
| Robot | `enp0s31f6` | `192.168.50.2/24` | Doodle robot side |
| Local Doodle radio | management bridge | `10.223.63.173/16` | Web/SSH management |
| Remote Doodle radio | management bridge | `10.223.228.225/16` | Web/SSH management over the mesh |

The `10.223.1.101/16` address on the operator computer is temporary. It is only
needed to reach the Doodle management interfaces from the browser.

## Important aliases and addresses

Use the Doodle-specific target for the Doodle link:

```bash
ssh robot_mtt_doodle
```

This points to:

```text
mohamed@192.168.50.2
```

Do not use `mtt-high-level-mohamed` to test Doodle. That alias points to the
normal robot LAN:

```text
mohamed@192.168.2.2
```

Useful SSH targets:

```bash
ssh mtt-high-level-mohamed       # normal robot LAN, 192.168.2.2
ssh robot_mtt_doodle             # Doodle network, 192.168.50.2
ssh robot_mtt_remote             # private Tailscale target, operator-specific
ssh mtt-high-level-mohamed-eth   # direct Ethernet profile, 192.168.3.102
```

## Doodle web UI

Before opening the web UI, make sure the operator computer has a management IP
on the Doodle Ethernet interface:

```bash
ip -br addr show dev enp152s0
```

Expected:

```text
enp152s0 UP 192.168.50.101/24 10.223.1.101/16 ...
```

If `10.223.1.101/16` is missing, add it temporarily:

```bash
sudo ip addr add 10.223.1.101/16 dev enp152s0
```

Then open:

```text
https://10.223.63.173/meshrider/
```

for the Doodle radio plugged into the operator computer, and:

```text
https://10.223.228.225/meshrider/
```

for the Doodle radio on the robot side.

The browser will likely show a certificate warning. That is expected for these
embedded radios.

When finished, remove the temporary management IP if desired:

```bash
sudo ip addr del 10.223.1.101/16 dev enp152s0
```

## Doodle SSH access

The radios are OpenWrt / LuCI based SmartRadio / Mesh Rider devices. SSH is
available as `root`, but the radios only offer an old `ssh-rsa` host key type,
so modern OpenSSH needs explicit compatibility options.

Local operator-side Doodle:

```bash
ssh -o HostKeyAlgorithms=+ssh-rsa \
  -o PubkeyAcceptedAlgorithms=+ssh-rsa \
  root@10.223.63.173
```

Remote robot-side Doodle:

```bash
ssh -o HostKeyAlgorithms=+ssh-rsa \
  -o PubkeyAcceptedAlgorithms=+ssh-rsa \
  root@10.223.228.225
```

IPv6 link-local access also worked during debugging:

```text
fe80::230:1aff:fe3b:3fae%enp152s0  -> smartradio-301a3b3fad, local side
fe80::230:1aff:fe3b:e4e2%enp152s0  -> smartradio-301a3be4e1, remote side
```

The IPv6 form is useful for discovery, but the IPv4 management addresses above
are easier for normal use.

Avoid using `192.168.153.1` for management even though both radios advertise it.
Both radios have that same address, so it is ambiguous.

## Verification checklist

Operator computer:

```bash
nmcli connection show --active
ip -br addr show dev enp152s0
ip route get 192.168.50.2
ping -c 5 192.168.50.2
```

Expected:

```text
ethernet-doodle active on enp152s0
192.168.50.101/24 on enp152s0
route to 192.168.50.2 goes through enp152s0
0% packet loss
```

Robot:

```bash
ssh robot_mtt_doodle 'ip -br addr show enp0s31f6'
ssh robot_mtt_doodle 'ip route get 192.168.50.101'
ssh robot_mtt_doodle 'ip neigh show dev enp0s31f6'
ssh robot_mtt_doodle 'ping -c 5 192.168.50.101'
```

Expected:

```text
enp0s31f6 UP 192.168.50.2/24
route to 192.168.50.101 goes through enp0s31f6
0% packet loss
```

Check that the Doodle profile does not become the robot default gateway:

```bash
ssh robot_mtt_doodle 'nmcli -f GENERAL.DEVICE,GENERAL.STATE,GENERAL.CONNECTION,IP4.ADDRESS,IP4.GATEWAY device show enp0s31f6'
ssh robot_mtt_doodle 'nmcli -f connection.id,connection.interface-name,ipv4.method,ipv4.addresses,ipv4.gateway,ipv4.never-default connection show doodle-mesh'
```

Expected:

```text
GENERAL.CONNECTION: doodle-mesh
IP4.ADDRESS: 192.168.50.2/24
IP4.GATEWAY: --
ipv4.never-default: yes
```

## Ports observed during the session

Run:

```bash
nmap -sT -Pn -p 22,80,443,5201,7070,7447,8765 \
  192.168.50.2 192.168.50.101 10.223.63.173 10.223.228.225
```

Observed:

```text
192.168.50.2 robot:
  22/tcp    ssh
  7447/tcp  Zenoh
  8765/tcp  Foxglove

192.168.50.101 operator computer:
  22/tcp    ssh
  7070/tcp  local service

10.223.63.173 local Doodle:
  22/tcp    ssh
  80/tcp    http
  443/tcp   https

10.223.228.225 remote Doodle:
  22/tcp    ssh
  80/tcp    http
  443/tcp   https
```

## Measured link performance

The link was stable, but not at a comfortable 100 Mbit/s application throughput.
The `100 Mbit/s` expectation should be treated as a PHY/marketing/ideal figure,
not a guaranteed useful ROS/Foxglove throughput in the field.

Measured values:

```text
PC -> robot iperf3 TCP:       about 6.9 Mbit/s
robot -> PC iperf3 TCP:       about 6.5 Mbit/s
Foxglove real traffic:        about 16.3 Mbit/s robot -> PC
Ping with Foxglove only:      avg 11.7 ms, max 22 ms, 0% loss
Ping while iperf3 was active: avg 13-22 ms, max 42 ms, 0% loss
```

Interface counters after stress testing:

```text
Robot enp0s31f6: 0 RX errors, 0 RX drops, 0 TX errors, 0 TX drops
PC enp152s0:     0 RX errors/drops, very small historical TX errors/drops
```

Interpretation:

- The link is usable and stable.
- The link is already stressed when Foxglove displays heavy sensor streams.
- High-rate raw point clouds or depth streams can consume the practical margin.
- For range, prefer lower sustained bandwidth over maximum visual richness.

## Stress-test commands

Start a one-shot server on the robot:

```bash
ssh robot_mtt_doodle 'iperf3 -s -1'
```

PC to robot:

```bash
iperf3 -c 192.168.50.2 -t 20 -i 2
```

Robot to PC:

```bash
ssh robot_mtt_doodle 'iperf3 -s -1'
iperf3 -c 192.168.50.2 -R -t 20 -i 2
```

Parallel TCP streams, useful to test aggregate capacity:

```bash
ssh robot_mtt_doodle 'iperf3 -s -1'
iperf3 -c 192.168.50.2 -P 4 -t 30
```

UDP probing, increase carefully:

```bash
ssh robot_mtt_doodle 'iperf3 -s -1'
iperf3 -c 192.168.50.2 -u -b 10M -t 20
iperf3 -c 192.168.50.2 -u -b 20M -t 20
```

Latency during a load test:

```bash
ping -c 30 -i 0.2 192.168.50.2
```

Traffic measurement without packet capture:

```bash
bash -lc 'a=$(cat /sys/class/net/enp152s0/statistics/rx_bytes); \
  b=$(cat /sys/class/net/enp152s0/statistics/tx_bytes); \
  sleep 5; \
  c=$(cat /sys/class/net/enp152s0/statistics/rx_bytes); \
  d=$(cat /sys/class/net/enp152s0/statistics/tx_bytes); \
  awk -v rx=$((c-a)) -v tx=$((d-b)) \
  "BEGIN {printf \"5s avg: RX %.2f Mbit/s TX %.2f Mbit/s\n\", rx*8/5/1000000, tx*8/5/1000000}"'
```

If the operator can provide sudo, a useful passive capture summary is:

```bash
sudo tshark -i enp152s0 -a duration:20 -q \
  -z io,stat,1 \
  -z conv,ip \
  -f "not port 22"
```

## Foxglove and Zenoh observations

When Foxglove was open, the robot had an active TCP connection:

```text
192.168.50.2:8765 -> 192.168.50.101
```

At one point the Foxglove TCP send queue was close to `0.9 MB`, and TCP RTT was
observed around `47-82 ms`. That is a warning sign that Foxglove can queue data
when heavy topics are displayed.

The robot exposes:

```text
7447/tcp  Zenoh
8765/tcp  Foxglove
```

Next work should focus on keeping only field-useful data on the Doodle link.

Recommended Foxglove display policy:

- Keep low-bandwidth control/state topics visible:
  - `/tf`
  - `/tf_static`
  - `/mtt_odometry`
  - `/mtt_tachometer`
  - `/mtt_health`
  - `/mtt_status`
  - `/mtt_control/*`
  - `/gps_front/*`
  - selected diagnostics
- Prefer downsampled or voxelized point clouds.
- Avoid streaming multiple raw heavy topics at once:
  - `/hesai_lidar/points`
  - `/rsairy_ns/points`
  - `/zed/zed_node/point_cloud/cloud_registered`
  - `/zed/zed_node/depth/*`
  - raw RGB/depth images unless compressed and rate-limited

Possible next engineering steps:

1. Create a "field Doodle" Foxglove profile that subscribes only to lightweight
   topics by default.
2. Add or expose downsampled LiDAR topics for remote visualization.
3. Rate-limit or compress heavy image/depth topics before they reach Foxglove.
4. Review the Zenoh router/session configuration so Doodle mode only forwards
   the topics needed by the operator.
5. Add a repeatable bandwidth health check to the demo stack, using iperf3 plus
   route/interface counter checks.
6. If range is the priority, target sustained traffic around `5-10 Mbit/s`
   rather than trying to push raw sensor streams continuously.

## Doodle radio settings to review

In the Mesh Rider web UI, record these values before changing anything:

- channel / center frequency
- channel width
- transmit power
- MCS / data rate mode
- distance / ACK timeout / range parameter
- noise floor
- RSSI / SNR per peer
- mesh ID / encryption settings
- QoS or traffic shaping settings
- firmware version

Change both radios consistently. If a radio setting is changed on only one
side, the mesh link can disappear.

For long range, physical setup matters as much as configuration:

- line of sight
- Fresnel zone clearance
- antenna gain and frequency compatibility
- same polarization on both radios
- antenna height
- cable/connector quality
- separation from USB3, GPU, LiDAR, motors, and metal chassis surfaces

## Safe rollback / cleanup

Remove only the temporary Doodle management IP from the operator computer:

```bash
sudo ip addr del 10.223.1.101/16 dev enp152s0
```

Do not remove `192.168.50.101/24` from `enp152s0` while using the Doodle link.

If the operator computer loses normal Internet after experiments, inspect:

```bash
ip route show default
nmcli connection show --active
```

The Doodle Ethernet profile should not be the normal Internet default route.

## Minimal handoff summary

The Doodle link is up and usable:

```text
PC enp152s0        192.168.50.101/24
Robot enp0s31f6    192.168.50.2/24
Local Doodle UI    https://10.223.63.173/meshrider/
Remote Doodle UI   https://10.223.228.225/meshrider/
Robot SSH          ssh robot_mtt_doodle
Sync over Doodle   ./scripts/autosync_ws --doodle
```

Current concern:

```text
Foxglove can push around 16 Mbit/s over the link and may queue data.
The next optimization should reduce what Foxglove/Zenoh sends over Doodle.
```
