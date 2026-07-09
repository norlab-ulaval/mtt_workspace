# MTT Networking Reference

This document describes the intended network layout for the MTT robot, the
operator computer, the sensor subnets, and the helper scripts used to move data
or share Internet access.

For the current Doodle field setup, web UI addresses, measured throughput, and
next optimization work, also read
[Doodle Field Network Handoff](./doodle_field_handoff.md).

The goal is simple: robot control and ROS data should stay predictable, sensor
links should not steal default routes, and temporary Internet sharing should be
easy to enable and easy to undo.

## Network model

The robot has several independent networks. Treat them as separate purposes,
not as interchangeable ports.

This document uses placeholders for commands that should work for any operator:

| Placeholder | Meaning | MTT lab default or example |
| --- | --- | --- |
| `<robot-user>` | Linux account used over SSH on the robot | `robot` or `mohamed` |
| `<robot-lan-ip>` | robot IP on the local robot LAN / `norlab_mtt` | `192.168.2.2` |
| `<pc-lan-ip>` | operator computer IP on the robot LAN | `192.168.2.101` |
| `<pc-wifi-iface>` | operator Wi-Fi interface connected to `norlab_mtt` | often `wlp0s20f3` |
| `<pc-ethernet-iface>` | operator Ethernet interface | varies, for example `enp152s0` |
| `<robot-internet-iface>` | robot interface that currently has Internet | for example `enp7s0` |
| `<tailscale-ip>` | optional Tailscale IP for a private operator setup | site/user-specific |

| Link | Typical interface | Address | Purpose | Default route |
| --- | --- | --- | --- | --- |
| Robot LAN / `norlab_mtt` | robot `enp5s0` | `192.168.2.2/24` | PC to robot, SSH, ROS tools, Foxglove, runtime access | No |
| Operator Wi-Fi | PC `<pc-wifi-iface>` | usually `192.168.2.101/24` | PC side of `norlab_mtt` | Usually no, except temporary Internet sharing |
| Robot wall Internet | robot `<robot-internet-iface>` | DHCP, for example `10.68.14.198/24` | Robot Internet uplink | Yes, when plugged |
| PC Ethernet | PC `<pc-ethernet-iface>` | DHCP or static depending on profile | Wall Internet, direct cable, or Doodle depending on what is plugged | Depends on active profile |
| Doodle / mesh | robot `enp0s31f6` | `192.168.50.2/24` | Mesh/Doodle link | No |
| Doodle / mesh | PC side | `192.168.50.101/24` usually | PC side of Doodle link | No, unless explicitly sharing Internet |
| Hesai LiDAR | robot `enp4s0` | `192.168.2.102/24` plus sensor host route | Hesai traffic | No |
| RoboSense RS-Airy | robot `enp8s0` | `192.168.1.102/24` | RoboSense / RS-Airy real sensor | No |
| Tailscale | robot `tailscale0` | `<tailscale-ip>` | Optional private remote fallback access | Usually managed by Tailscale |

Only one interface should own the robot default route at a time. In the MTT lab
wall-Internet setup, the robot default route should look like this:

```bash
default via 10.68.14.1 dev enp7s0 proto dhcp
```

The `192.168.2.0/24`, `192.168.1.0/24`, and `192.168.50.0/24` networks are
robot/local networks. They should not become accidental Internet uplinks unless
you intentionally run an Internet-sharing command.

## Golden rules

1. Do not put a default gateway on a LiDAR-only interface.
2. Do not put a default gateway on the Doodle static profile unless the Doodle
   really provides Internet.
3. Keep `norlab_mtt` as `/24`. A ghost `/16` address such as
   `192.168.2.103/16` on the PC Wi-Fi can break routing decisions.
4. Prefer explicit routes for temporary Internet sharing. Do not make them
   permanent unless the physical setup is permanent.
5. Tailscale can be a recovery path for a configured private operator setup.
   Keep `tailscaled` enabled only on robots where Tailscale is part of the
   deployment.
6. When in doubt, check the current state before changing anything:

```bash
ip -br addr
ip route
nmcli device status
nmcli connection show --active
ip neigh
```

## Reference robot state

Use these commands from the PC:

```bash
ssh <robot-user>@<robot-lan-ip> 'ip -br addr'
ssh <robot-user>@<robot-lan-ip> 'ip route'
ssh <robot-user>@<robot-lan-ip> 'nmcli device status'
ssh <robot-user>@<robot-lan-ip> 'nmcli connection show --active'
```

For the MTT lab default, that becomes either:

```bash
ssh robot@192.168.2.2 'ip -br addr'
ssh mohamed@192.168.2.2 'ip -br addr'
```

Expected important points:

- `enp5s0` is up with `192.168.2.2/24`.
- `enp7s0` has DHCP Internet when the wall cable is plugged.
- the robot default route uses `enp7s0` when wall Internet is available.
- `tailscale0` is present only when the robot is enrolled in Tailscale.
- LiDAR and Doodle interfaces do not own the default route.

Check Tailscale boot persistence only on setups that intentionally use
Tailscale:

```bash
ssh <robot-user>@<robot-lan-ip> 'systemctl is-enabled tailscaled'
ssh <robot-user>@<robot-lan-ip> 'systemctl is-active tailscaled'
ssh <robot-user>@<robot-lan-ip> 'tailscale status'
```

Expected:

```text
enabled
active
```

## Reference operator computer state

For the Wi-Fi robot LAN mode:

```bash
ip -br addr show dev <pc-wifi-iface>
ip route get <robot-lan-ip>
ping -c 2 <robot-lan-ip>
```

Expected:

- `<pc-wifi-iface>` is up.
- it has `192.168.2.101/24`.
- route to `<robot-lan-ip>` goes through `<pc-wifi-iface>`.
- there is no extra `192.168.x.x/16` address on that interface.

If the PC is on wall Ethernet, it is normal to see:

```bash
default via 10.68.14.1 dev <pc-ethernet-iface>
```

In that case the PC is using its own wall Internet directly, not the robot as a
router.

## Resetting the PC default route

If you previously ran:

```bash
sudo ip route replace default via <robot-lan-ip> dev <pc-wifi-iface>
```

then the PC sends Internet traffic through the robot.

That is fine while you want robot-to-PC Internet sharing. It is not dangerous by
itself, but it can be confusing after you unplug or change networks.

Check the current default route:

```bash
ip route show default
```

If it still points to the robot and you want to undo it:

```bash
sudo ip route del default via <robot-lan-ip> dev <pc-wifi-iface>
```

Then reconnect the normal network profile so NetworkManager recreates the right
default route:

```bash
nmcli connection show --active
nmcli device status
```

For example, reconnect Wi-Fi or Ethernet from the desktop UI, or use the exact
active connection name:

```bash
nmcli connection up "<connection-name>"
```

For the MTT lab Wi-Fi defaults, the rollback command is:

```bash
sudo ip route del default via 192.168.2.2 dev wlp0s20f3
```

If the PC already shows a default route through a real Internet uplink, such as
Ethernet or eduroam, you do not need to reset anything.

## Script: `scripts/internet_via_usb`

Despite the historical name, this script is now a general temporary Internet
sharing helper. It supports both directions:

- PC has Internet, robot needs Internet.
- robot has Internet, PC needs Internet.

It uses standard Linux routing:

- `net.ipv4.ip_forward=1`
- `iptables` NAT masquerade
- explicit default route replacement on the client side

The script does not permanently rewrite NetworkManager profiles. Its route
changes are runtime route table changes. They are easy to undo and usually
disappear after reconnect or reboot.

### Dry run first

Always dry-run when changing physical setup:

```bash
./scripts/internet_via_usb --dry-run
./scripts/internet_via_usb --robot-to-pc --dry-run
```

The dry run prints detected interfaces, target host, NAT subnet, and the route
command that would be needed.

### PC to robot Internet

Use this when the PC has Internet through a phone, eduroam, wall Ethernet, or
another uplink, and the robot only reaches the PC over `norlab_mtt`.

Run on the PC:

```bash
./scripts/internet_via_usb
```

The script configures NAT on the PC and prints the route command to run on the
robot. It usually looks like:

```bash
ssh <robot-user>@<robot-lan-ip> "sudo ip route replace default via <pc-lan-ip> dev enp5s0"
```

If autodetection picks the wrong interfaces, force them:

```bash
UP_IFACE=<pc-internet-iface> DOWN_IFACE=<pc-wifi-iface> ./scripts/internet_via_usb
```

Examples:

```bash
UP_IFACE=<pc-ethernet-iface> DOWN_IFACE=<pc-wifi-iface> ./scripts/internet_via_usb
UP_IFACE=<phone-usb-iface> DOWN_IFACE=<pc-wifi-iface> ./scripts/internet_via_usb
```

Verify on the robot:

```bash
ssh <robot-user>@<robot-lan-ip> 'ip route show default'
ssh <robot-user>@<robot-lan-ip> 'ping -c 2 1.1.1.1'
ssh <robot-user>@<robot-lan-ip> 'resolvectl query google.com'
```

### Robot to PC Internet over `norlab_mtt`

Use this when the robot has Internet from the wall cable and the PC is connected
to `norlab_mtt`.

Run on the PC:

```bash
./scripts/internet_via_usb --robot-to-pc
```

This configures forwarding and NAT on the robot over SSH. The sudo password
prompt is for the remote robot account shown in the log, for example:

```text
[INFO] Sudo = mot de passe du compte ROBOT <robot-user>@<robot-lan-ip>, pas celui du PC
```

Then run the printed route command on the PC:

```bash
sudo ip route replace default via <robot-lan-ip> dev <pc-wifi-iface>
```

This second sudo prompt is local to the PC.

You can ask the script to apply the PC route too:

```bash
./scripts/internet_via_usb --robot-to-pc --apply-route
```

That mode may ask for two sudo passwords:

- robot sudo first, through SSH
- PC sudo second, for the local route replacement

Verify on the PC:

```bash
ip route show default
ping -c 2 1.1.1.1
resolvectl query google.com
```

If raw IP works but DNS fails:

```bash
resolvectl dns <pc-wifi-iface> 1.1.1.1 8.8.8.8
resolvectl query google.com
```

Rollback:

```bash
sudo ip route del default via <robot-lan-ip> dev <pc-wifi-iface>
```

### Robot to PC Internet over Doodle or direct Ethernet

Use this when the PC reaches the robot through the Doodle/static network and the
robot has Internet through another uplink.

Run from the PC:

```bash
ROBOT_HOST=192.168.50.2 \
ROBOT_ROUTE_IFACE=enp0s31f6 \
CLIENT_SUBNET=192.168.50.0/24 \
ROBOT_UP_IFACE=<robot-internet-iface> \
DOWN_IFACE=<pc-ethernet-iface> \
./scripts/internet_via_usb --robot-to-pc
```

Then apply the route printed by the script on the PC, usually:

```bash
sudo ip route replace default via 192.168.50.2 dev <pc-ethernet-iface>
```

Rollback:

```bash
sudo ip route del default via 192.168.50.2 dev <pc-ethernet-iface>
```

## Script: `scripts/autosync_ws`

`autosync_ws` copies the workspace to a robot target with `rsync`. It does not
set up Internet sharing. It only chooses the SSH path used for syncing.

Default target:

```bash
./scripts/autosync_ws
```

Uses the normal robot host from `.env`, usually:

```text
<robot-user>@192.168.2.2
```

Tailscale target:

```bash
./scripts/autosync_ws --remote
```

Uses `ROBOT_SSH_TARGET_REMOTE` from `.env`. This is private to the operator or
deployment. Do not assume a public default Tailscale address.

```text
<robot-user>@<tailscale-ip>
```

Doodle target:

```bash
./scripts/autosync_ws --doodle
```

Uses:

```text
192.168.50.2
```

Ethernet target:

```bash
./scripts/autosync_ws --eth
```

Uses:

```text
192.168.3.102
```

Important: `autosync_ws --eth` does not mean "give the PC Internet". It means
"sync over the Ethernet robot target configured as `192.168.3.102`". Use
`internet_via_usb` for Internet sharing.

## Script: `scripts/create_env`

`create_env` writes the local `.env` used by the scripts and compose files.

Set the normal robot target:

```bash
./scripts/create_env --robot-target <robot-user>@192.168.2.2
```

Examples:

```bash
./scripts/create_env --robot-target robot@192.168.2.2
./scripts/create_env --robot-target mohamed@192.168.2.2
```

Useful generated variables:

```text
ROBOT_USER
ROBOT_HOST
ROBOT_SSH_TARGET
ROBOT_HOST_ETH
ROBOT_SSH_TARGET_ETH
ROBOT_HOST_DOODLE
ROBOT_SSH_TARGET_DOODLE
ROBOT_HOST_REMOTE
ROBOT_SSH_TARGET_REMOTE
ROBOT_WORKSPACE
ZENOH_ROUTER_ENDPOINT
FOXGLOVE_WS_URL
```

After changing the main robot target, inspect `.env`:

```bash
grep -E '^(ROBOT_|ZENOH_|FOXGLOVE_)' .env
```

## Script: `scripts/audit_network_field.sh`

This script collects a field network and ROS snapshot:

```bash
./scripts/audit_network_field.sh
```

It prints:

- host and time
- addresses and routes
- neighbors
- NetworkManager status
- Ethernet link stats
- Docker status
- ROS graph, bandwidth, and topic frequency

Run it on the machine you want to inspect. To run it on the robot, sync the
workspace first or call it through SSH from a robot workspace path.

## Same physical Ethernet port: wall Internet vs Doodle

It is acceptable to reuse the same physical Ethernet connector for different
purposes, but only one physical cable and one NetworkManager profile should be
active for that connector at a time.

Recommended model:

- wall Internet profile:
  - DHCP address
  - default route allowed
  - DNS allowed
- Doodle profile:
  - static address, for example `192.168.50.2/24` on the robot
  - no default gateway
  - no DNS priority
  - used only for Doodle/mesh traffic unless explicitly sharing Internet

If the Doodle is plugged into the same port that normally receives wall
Internet, the robot cannot simultaneously use wall Internet on that same port.
It can still have Internet from another interface, for example Tailscale,
another Ethernet port, Wi-Fi, or a PC sharing Internet.

## Sensor networks

The LiDAR interfaces are not general LAN interfaces.

Expected behavior:

- Hesai traffic stays on the Hesai interface.
- RoboSense / RS-Airy real traffic stays on the RoboSense interface.
- no LiDAR interface should have a default route.
- high-bandwidth point cloud topics should remain local to the robot when
  possible.

Useful checks:

```bash
ssh <robot-user>@<robot-lan-ip> 'ip route'
ssh <robot-user>@<robot-lan-ip> 'ip neigh'
ssh <robot-user>@<robot-lan-ip> 'ip -s link show enp4s0'
ssh <robot-user>@<robot-lan-ip> 'ip -s link show enp8s0'
```

If a sensor is renamed in docs or config, verify the actual device by traffic
and IP, not by old labels:

```bash
ssh <robot-user>@<robot-lan-ip> 'ip neigh'
ssh <robot-user>@<robot-lan-ip> 'sudo tcpdump -ni enp4s0 -c 20'
ssh <robot-user>@<robot-lan-ip> 'sudo tcpdump -ni enp8s0 -c 20'
```

## NetworkManager diagnostics

PC:

```bash
ip -br addr
ip route
ip route get <robot-lan-ip>
ip neigh
nmcli device status
nmcli connection show --active
resolvectl status
journalctl -u NetworkManager -b --no-pager | tail -n 120
```

Robot:

```bash
ssh <robot-user>@<robot-lan-ip> 'ip -br addr'
ssh <robot-user>@<robot-lan-ip> 'ip route'
ssh <robot-user>@<robot-lan-ip> 'ip route get 1.1.1.1'
ssh <robot-user>@<robot-lan-ip> 'ip neigh'
ssh <robot-user>@<robot-lan-ip> 'nmcli device status'
ssh <robot-user>@<robot-lan-ip> 'nmcli connection show --active'
ssh <robot-user>@<robot-lan-ip> 'resolvectl status'
ssh <robot-user>@<robot-lan-ip> 'journalctl -u NetworkManager -b --no-pager | tail -n 120'
```

DHCP on a specific interface:

```bash
ssh <robot-user>@<robot-lan-ip> 'journalctl -u NetworkManager -b --no-pager | grep -E "enp7s0|dhcp|default route"'
```

## Quick decision table

| Situation | Run this |
| --- | --- |
| PC has Internet, robot needs Internet | `./scripts/internet_via_usb` on the PC, then apply the printed robot route |
| Robot has wall Internet, PC needs Internet over `norlab_mtt` | `./scripts/internet_via_usb --robot-to-pc`, then apply the printed PC route |
| Same as above, apply PC route automatically | `./scripts/internet_via_usb --robot-to-pc --apply-route` |
| Sync over normal robot LAN | `./scripts/autosync_ws` |
| Sync over Tailscale, only if the robot is enrolled for this operator/deployment | `./scripts/autosync_ws --remote` |
| Sync over Doodle | `./scripts/autosync_ws --doodle` |
| Sync over configured Ethernet target | `./scripts/autosync_ws --eth` |
| Check robot network health | `ssh <robot-user>@<robot-lan-ip> 'ip -br addr; ip route; nmcli device status'` |
| Undo PC route through robot | `sudo ip route del default via <robot-lan-ip> dev <pc-wifi-iface>` |

## Known failure patterns

### PC cannot ping the robot LAN IP

Check:

```bash
ip -br addr show dev <pc-wifi-iface>
ip route get <robot-lan-ip>
ip neigh show dev <pc-wifi-iface>
ping -c 2 <robot-lan-ip>
```

Common causes:

- PC Wi-Fi is down.
- PC is not on `norlab_mtt`.
- PC has the wrong mask, especially a stale `/16`.
- robot `enp5s0` is down.
- another PC route is capturing `192.168.2.0/24`.

### Internet sharing script says OK but PC has no Internet

Check whether the PC default route was actually changed:

```bash
ip route show default
```

For robot-to-PC sharing over Wi-Fi, expect:

```text
default via <robot-lan-ip> dev <pc-wifi-iface>
```

If `ping -c 2 1.1.1.1` works but DNS does not:

```bash
resolvectl dns <pc-wifi-iface> 1.1.1.1 8.8.8.8
resolvectl query google.com
```

### Route points through robot after the test is over

Delete it:

```bash
sudo ip route del default via <robot-lan-ip> dev <pc-wifi-iface>
```

Then reconnect the normal network.

### `Connection to <robot-lan-ip> closed.` after remote sudo

This is normal when the SSH command finished. If the script prints `[OK]`, the
remote command returned successfully.

### Duplicate ping packets

One duplicate packet during a route change is not automatically a failure. If it
continues, inspect routes and neighbors:

```bash
ip route
ip neigh
```

## Reboot checklist

After rebooting the robot:

```bash
ssh <robot-user>@<robot-lan-ip> 'ip -br addr'
ssh <robot-user>@<robot-lan-ip> 'ip route'
ssh <robot-user>@<robot-lan-ip> 'ping -c 2 1.1.1.1'
ssh <robot-user>@<robot-lan-ip> 'resolvectl query google.com'
```

If this deployment uses Tailscale, also check:

```bash
ssh <robot-user>@<robot-lan-ip> 'systemctl is-active tailscaled'
ssh <robot-user>@<tailscale-ip> 'ip -br addr'
```

After rebooting the PC:

```bash
ip -br addr
ip route
ping -c 2 <robot-lan-ip>
```

Then run the exact Internet-sharing script again only if you need temporary
sharing for that session.
