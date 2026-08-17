# Zenoh on the MTT

This is the operational reference for ROS 2 over Zenoh in the MTT workspace.

## Purpose

Zenoh carries ROS 2 traffic between the robot and the operator computer without
depending on DDS multicast discovery across the field network.

The robot runs one Zenoh router. Operator-side containers run as Zenoh clients.
Foxglove can be used in two ways:

- direct: Foxglove Studio connects to the robot bridge on port 8765;
- local: the laptop monitor receives ROS through Zenoh and exposes a local
  Foxglove bridge on port 8766.

```text
robot ROS nodes
    |
    | rmw_zenoh_cpp, ROS_DOMAIN_ID=2
    v
robot Zenoh router :7447
    |
    | TCP over normal LAN or Doodle
    v
laptop rmw_zenoh client
    |
    +-> local Foxglove bridge :8766 -> Foxglove Studio

robot Foxglove bridge :8765 --------> Foxglove Studio, direct mode
```

## Defaults

| Setting | Normal LAN | Doodle field mode |
| --- | --- | --- |
| Robot address | `192.168.2.2` | `192.168.50.2` |
| Zenoh router | `tcp/192.168.2.2:7447` | `tcp/192.168.50.2:7447` |
| Robot Foxglove | `ws://192.168.2.2:8765` | `ws://192.168.50.2:8765` |
| Local monitor Foxglove | `ws://localhost:8766` | `ws://localhost:8766` |
| ROS domain | `2` | `2` |
| RMW on monitor | `rmw_zenoh_cpp` | `rmw_zenoh_cpp` |

The general workstation `.env` may use `ROS_DOMAIN_ID=0` for local work.
Monitor and live services explicitly use `LIVE_ROBOT_DOMAIN_ID`, default `2`.
Both ends of the live Zenoh session must use the same value.

## Files that control the setup

| File | Responsibility |
| --- | --- |
| `.env` | generated host, robot, endpoint and port values |
| `scripts/create_env` | creates and updates `.env` |
| `demos/live_robot/config/runtime.env` | selects field mode and bind address |
| `demos/live_robot/compose.yaml` | robot router and robot-side Foxglove services |
| `demos/monitor/compose.yaml` | laptop Zenoh clients and local bridge |
| `scripts/run_with_zenoh_session.sh` | validates endpoint and renders client config |
| `demos/live_robot/config/zenoh_session.template.json5` | laptop client transport policy |
| `src/external/norlab_robot/scripts/config/zenoh_router_field_doodle.template.json5` | robot field router policy |
| `demos/live_robot/config/foxglove_bridge_field_light.yaml` | reduced field topic allowlist |

Do not edit the rendered file under `$HOME/.ros`. It is regenerated from the
template.

## Endpoint selection

`scripts/run_with_zenoh_session.sh` uses this order:

1. if `FIELD_NETWORK_MODE=true`, use
   `tcp/${DOODLE_ROBOT_IP}:${ZENOH_FIELD_PORT:-7447}`;
2. otherwise use `ROBOT_ZENOH_ENDPOINT`;
3. otherwise use `ZENOH_ROUTER_ENDPOINT`;
4. final fallback: `tcp/192.168.2.2:7447`.

The script rejects endpoints that are not in `tcp/<host>:<port>` form. It then
exports both variables used by current and older `rmw_zenoh_cpp` versions:

```text
ZENOH_SESSION_CONFIG_URI
RMW_ZENOH_CONFIG_FILE
```

## Client behavior

The laptop template uses client mode with one explicit TCP endpoint.

- multicast scouting is disabled;
- gossip scouting is disabled;
- peer-to-peer discovery is not used;
- connection retry starts at 500 ms and increases to 3 s;
- the process can start before the robot router is reachable;
- transport shared memory is disabled on the laptop;
- receive buffer is 4 MiB;
- sensor data may be dropped under congestion instead of blocking the link.

This makes the route deterministic on a field link. It also means a wrong IP,
blocked TCP port or wrong ROS domain will not be repaired by multicast discovery.

## Start the robot side

On the robot:

```bash
cd demos/live_robot
docker compose up
```

This is a hardware command. The `robot` service can initialize CAN and teleop.
For router-only diagnostics:

```bash
docker compose up zenoh
```

Robot-side infrastructure including Foxglove:

```bash
docker compose --profile infra up
```

## Start the laptop side

On the operator computer:

```bash
cd demos/monitor
docker compose up monitor
```

Connect Foxglove Studio to:

```text
ws://localhost:8766
```

For Doodle:

```bash
FIELD_NETWORK_MODE=true DOODLE_ROBOT_IP=192.168.50.2 docker compose up monitor
```

The field mode selects the reduced Foxglove configuration automatically.

## Verification

Network only, from the laptop:

```bash
ip route get 192.168.50.2
ping -c 3 192.168.50.2
nc -vz 192.168.50.2 7447
nc -vz 192.168.50.2 8765
```

Normal LAN uses `192.168.2.2` instead.

Check containers:

```bash
docker compose -f demos/live_robot/compose.yaml ps
docker compose -f demos/monitor/compose.yaml ps
docker compose -f demos/monitor/compose.yaml logs --tail=100 monitor
```

Check the ROS graph from the monitor client:

```bash
cd demos/monitor
docker compose --profile debug run --rm bash
ros2 node list
ros2 topic list
```

Only run the ROS graph checks when it is acceptable to connect to the live
robot session. Listing is read-only, but the client joins the operational graph.

## Foxglove choice

Use direct robot Foxglove when:

- the link is good;
- only one operator needs access;
- the selected panels use lightweight topics.

Use the local monitor bridge when:

- laptop tools also need ROS access through Zenoh;
- the Doodle route must be explicit;
- the field topic allowlist is required.

Do not run both paths with multiple raw image and point-cloud panels unless the
link has been measured with that exact load.

## Field bandwidth policy

The July 2026 Doodle measurements were about 6.5 to 6.9 Mbit/s for sustained
iperf TCP. Foxglove traffic was observed above that rate in bursts and built a
large TCP send queue. Treat the link as constrained.

Keep remote:

- `/tf`, `/tf_static`;
- odometry, tachometer, health and mode topics;
- GPS and selected diagnostics;
- `/debug/hesai_points_voxel` at a low rate.

Keep robot-local unless actively debugging:

- raw Hesai and RS-Airy clouds;
- ZED registered point cloud;
- raw RGB, depth and uncompressed image streams;
- full ICP map updates at high frequency.

The live runtime exposes a voxelized field cloud configured by:

```text
LIDAR_DEBUG_VOXEL_SIZE=0.30
LIDAR_DEBUG_MAX_RATE_HZ=2.0
LIDAR_DEBUG_MAX_POINTS=25000
```

## QoS and congestion

ROS topic QoS remains a ROS configuration concern. Zenoh transports the
resulting traffic; it does not make incompatible ROS publisher/subscriber QoS
compatible.

Recording uses explicit topic lists and QoS overrides under:

```text
src/external/norlab_robot/config/rosbag_record/
```

When a topic is absent remotely, check in this order:

1. publisher exists on the robot;
2. publisher and subscriber QoS are compatible;
3. both sides use `rmw_zenoh_cpp` for this session;
4. both sides use the same live ROS domain;
5. the field Foxglove allowlist includes the topic;
6. the network is not saturated.

## Failure table

| Symptom | Check | Fix |
| --- | --- | --- |
| connection retries forever | route and TCP 7447 | correct robot IP, field mode or firewall |
| nodes absent but TCP works | `LIVE_ROBOT_DOMAIN_ID` and RMW | use domain 2 and `rmw_zenoh_cpp` on both ends |
| local Foxglove opens but has no topics | monitor logs and Zenoh endpoint | fix client endpoint, then restart monitor |
| direct Foxglove works, local bridge does not | port 7447 and monitor config | fix Zenoh path; port 8765 is a separate path |
| controls lag when clouds are open | link rate and TCP queue | close heavy panels and use voxelized cloud |
| `Unable to connect to scouted peer` noise | wrong or old config loaded | verify the rendered client config disables scouting |
| duplicate data or high bandwidth | two bridges or duplicate publishers | stop unused bridge and inspect topic publisher count |

## Security boundary

The checked configuration uses plain TCP endpoints. TLS, user authentication
and topic-level authorization are not configured in this workspace. Treat
ports 7447, 8765 and 8766 as trusted-LAN services. Do not expose them directly
to a public network. Use a private tunnel or a separately secured deployment for
remote access.
