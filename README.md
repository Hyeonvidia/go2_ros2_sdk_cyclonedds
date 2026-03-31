# Go2 ROS2 SDK — CycloneDDS (Ethernet)

Standalone ROS2 Humble workspace for the **Unitree Go2 EDU** robot, using
**CycloneDDS** over direct ethernet. No WiFi or WebRTC dependencies.

Runs on **Jetson Orin NX** mounted on the Go2.

> **Based on [go2_ros2_sdk](https://github.com/abizovnuralem/go2_ros2_sdk) by abizovnuralem.**
> This project adapts the original WebRTC-based SDK to work over CycloneDDS ethernet,
> enabling low-latency direct DDS communication without WiFi dependency.

## Key Changes from Original

- **Transport**: WebRTC (WiFi) → CycloneDDS (Ethernet, direct DDS)
- **LiDAR**: WASM voxel decoder → Go2 native `cloud_deskewed` (11k pts, 15Hz, odom frame)
- **Network**: Dual IP setup (DHCP for internet + static IP for Go2 ethernet)
- **Sport API**: RELIABLE QoS (BEST_EFFORT causes leg collapse)
- **Nav2**: `static_layer` removed from costmaps (Go2 internal DDS publishes malformed `/map`)
- **Docker**: Multi-stage build optimized for Jetson Orin NX (ARM64)
- **CycloneDDS config**: Templated with `envsubst` for flexible deployment

## Quick Start

```bash
# 1. Network setup (one-time, persists across reboots)
make setup-network

# 2. Build the Docker image
make build

# 3. Start (safe mode — LiDAR, SLAM, Nav2, no robot movement)
make up

# 4. Start with robot control enabled (robot must be standing + sport mode active)
ENABLE_CMD_VEL=true make up
```

## Makefile Commands

| Command | Description |
|---------|-------------|
| `make build` | Build Docker image |
| `make up` | Start container (safe mode, cmd_vel disabled) |
| `make safe` | Explicit safe mode (same as `make up`) |
| `make down` | Stop container |
| `make logs` | Tail container logs |
| `make shell` | Open bash inside container |
| `make topics` | List ROS2 topics |
| `make rebuild` | Force rebuild (no cache) |
| `make setup-network` | Add static IP for Go2 communication |
| `make check-network` | Check network and Go2 connectivity |

## Network Setup

The Go2 EDU communicates via ethernet on `192.168.123.0/24`. The Jetson
connects via `enP8p1s0` with a secondary static IP (`192.168.123.18/24`)
alongside DHCP for internet access.

```bash
# One-time setup (persists across reboots)
sudo bash scripts/setup_network.sh

# Verify
sudo bash scripts/setup_network.sh --check-only
```

CycloneDDS config is templated (`docker/cyclonedds.xml.template`). Override
via environment variables:

```bash
GO2_IP=192.168.123.200 CYCLONE_IFACE=eth0 make up
```

## Architecture

```
Go2 EDU (DDS)              Driver Node                  ROS2 Topics
───────────────            ────────────                 ───────────
utlidar/cloud_deskewed ──→ QoS bridge ────────────────→ go2/cloud (odom, 11k pts, 15Hz)
                                                     └→ /scan (via pointcloud_to_laserscan)
sportmodestate ─────────→ Go2DriverNode ──────────────→ odom + TF (odom → base_link)
                                                     ──→ imu
lf/lowstate ────────────→ Go2DriverNode ──────────────→ joint_states
cmd_vel ────────────────→ Go2DriverNode ──────────────→ api/sport/request (RELIABLE QoS)
```

**LiDAR Pipeline:**
- `/go2/cloud` — Dense 3D point cloud (11k pts, `frame_id=odom`, from `cloud_deskewed`)
- `/scan` — 2D LaserScan at 15Hz (from `pointcloud_to_laserscan`, `range_min=0.35m`)

**Nav2 cmd_vel Flow:**
```
Nav2 controller → /cmd_vel_nav → velocity_smoother → /cmd_vel → driver → api/sport/request
```

## Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `rviz2` | `true` | Launch RViz2 |
| `foxglove` | `true` | Launch Foxglove Bridge |
| `slam` | `true` | Launch SLAM Toolbox |
| `nav2` | `true` | Launch Nav2 Navigation |
| `joystick` | `false` | Launch joystick node |
| `teleop` | `false` | Launch teleop twist mux |
| `z_offset` | `0.07` | Z-axis offset for odom/TF |
| `enable_cmd_vel` | `false` | Enable cmd_vel forwarding to Go2 |

Override via Docker Compose environment:
```bash
SLAM=false NAV2=false RVIZ=false make up   # Minimal mode
ENABLE_CMD_VEL=true make up                # With robot control
```

## Key Technical Notes

- **Sport API requires RELIABLE QoS** — Using BEST_EFFORT causes Go2 legs to collapse
- **`cloud_deskewed` is the best LiDAR source** — 11k pts, already in odom frame, no transform needed
- **Go2 LiDAR detects own body at ~0.2m** — `range_min=0.35m` filters self-detection
- **Go2 DDS exposes all internal topics** — `static_layer` removed from Nav2 costmaps to avoid malformed `/map` conflict
- **Nav2 bond stability** — Docker healthcheck disabled; `bond_timeout=30s`
- **CycloneDDS** — `MaxAutoParticipantIndex=120` to support Nav2's many nodes
- **Robot must be in sport mode** for cmd_vel to work (activate via remote controller)
- **Nav2 params match original WebRTC SDK** — only CycloneDDS-specific changes applied

## Credits

- Original SDK: [go2_ros2_sdk](https://github.com/abizovnuralem/go2_ros2_sdk) by [@abizovnuralem](https://github.com/abizovnuralem)
- Unitree Go2 message definitions from [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2)

## License

BSD-3-Clause
