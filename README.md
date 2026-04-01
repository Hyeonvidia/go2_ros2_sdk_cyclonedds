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

# 3. Start (SLAM + Nav2 + cmd_vel auto-enabled)
make up

# 4. Robot control is auto-enabled when NAV2=true (default)
# To disable: ENABLE_CMD_VEL=false make up
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
- `/scan` — 2D LaserScan at 15Hz, 720 bins / 0.5° resolution (from `pointcloud_to_laserscan`, `range_min=0.35m`)

**Nav2 cmd_vel Flow:**
```
Nav2 controller → /cmd_vel_nav → velocity_smoother → /cmd_vel → driver → api/sport/request
```

**Speech Feedback:**
- `nav2_status_node` — Nav2 lifecycle/goal 상태 한국어 음성 안내 (via `/tts`)
- `waypoint_nav_node` — Waypoint 저장/로드/실행, 음성 진행 안내

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
| `enable_cmd_vel` | `true` | Enable cmd_vel forwarding to Go2 (auto-enabled when NAV2=true) |

Override via Docker Compose environment:
```bash
SLAM=false NAV2=false RVIZ=false make up   # Minimal mode
ENABLE_CMD_VEL=false make up               # Read-only mode (no robot movement)
```

## Key Technical Notes

- **Sport API requires RELIABLE QoS** — Using BEST_EFFORT causes Go2 legs to collapse
- **`cloud_deskewed` is the best LiDAR source** — 11k pts, already in odom frame, no transform needed
- **Go2 LiDAR detects own body at ~0.2m** — `range_min=0.35m` filters self-detection
- **Go2 DDS exposes all internal topics** — `static_layer` removed from Nav2 costmaps to avoid malformed `/map` conflict
- **Nav2 bond stability** — Docker healthcheck disabled; `bond_timeout=30s`
- **CycloneDDS** — `MaxAutoParticipantIndex=120` to support Nav2's many nodes
- **Sport mode auto-activation** — Driver sends BalanceStand (API 1002) on startup, no remote Start button needed
- **Nav2 velocity tuning** — DWB max 0.8 m/s, acc 1.0 m/s² for smooth quadruped motion

## SLAM Toolbox Plugin (RViz2)

RViz2 좌측 패널의 **SlamToolboxPlugin**으로 조작합니다.

### 매핑 (기본 모드)

컨테이너 시작 시 SLAM Toolbox가 자동으로 온라인 비동기 매핑을 수행합니다.
로봇을 이동시키면 실시간으로 `/map`이 생성됩니다.

### 맵 저장 및 불러오기

| 기능 | 설명 |
| --- | --- |
| **Save Map** | 현재 맵을 PGM/YAML 파일로 저장 (Nav2 map_server 호환) |
| **Serialize Map** | SLAM 내부 포즈그래프 저장 → 나중에 이어서 매핑 가능 |
| **Deserialize Map** | 이전에 저장한 포즈그래프 불러오기 |
| **Continue Mapping** | 저장된 맵에서 이어서 매핑 |
| **Localization Mode** | 매핑 중단, 기존 맵 기반 위치추정만 수행 |
| **Interactive Mode** | 맵의 노드를 수동으로 이동/조정 |

### 사용 시나리오

1. 최초 매핑: 로봇 이동 → 맵 생성 → **Serialize Map**으로 저장
2. 다음 세션: **Deserialize Map** → **Continue Mapping** (확장) 또는 **Localization Mode** (위치추정만)
3. Nav2 연동: Localization 모드에서 자율주행

## Nav2 자율 내비게이션 (RViz2)

### 단일 목표 내비게이션

1. RViz2 상단 툴바에서 **"Nav2 Goal"** 클릭
2. 맵 위에 목표 지점 클릭 (화살표 방향 = 도착 시 로봇 방향)
3. 로봇이 자동으로 경로 계획 및 이동

### Waypoint 내비게이션 (다중 목표)

1. RViz2 좌측 **"Navigation 2"** 패널에서 **"Waypoint / Nav Through Poses Mode"** 체크
2. 상단 툴바에서 **"Nav2 Goal"** 클릭
3. 맵 위에 **여러 지점을 순서대로 클릭** (클릭할 때마다 화살표 추가)
4. 모든 웨이포인트를 찍은 후:
   - **"Start Waypoint Following"** — 각 웨이포인트를 순서대로 방문 (정지 후 다음 이동)
   - **"Start Nav Through Poses"** — 모든 포즈를 하나의 경로로 통과 (정지 없음)

> **주의**: 일반 "2D Goal Pose"가 아닌 반드시 **"Nav2 Goal"** 도구를 사용해야 합니다.

### 서비스 기반 Waypoint 내비게이션

```bash
# 현재 로봇 위치를 웨이포인트로 저장
ros2 service call /waypoint_nav_node/save_waypoint std_srvs/srv/Trigger

# 저장된 웨이포인트로 내비게이션 시작
ros2 service call /waypoint_nav_node/start_waypoint_nav std_srvs/srv/Trigger

# 내비게이션 취소
ros2 service call /waypoint_nav_node/stop_waypoint_nav std_srvs/srv/Trigger

# 웨이포인트 초기화
ros2 service call /waypoint_nav_node/clear_waypoints std_srvs/srv/Trigger
```

### 음성 안내

Nav2 활성화 시 자동으로 한국어 음성 안내가 제공됩니다 (ElevenLabs TTS).

| 이벤트 | 음성 메시지 |
| --- | --- |
| Nav2 활성화 | "자율 내비게이션이 활성화 되었습니다" |
| 목표 이동 시작 | "목표 지점으로 이동을 시작합니다" |
| 목표 도착 | "목표 지점에 도착했습니다" |
| 내비게이션 실패 | "내비게이션이 실패했습니다" |
| 내비게이션 취소 | "내비게이션이 취소되었습니다" |

## Technical Notes

- **Sport API requires RELIABLE QoS** — Using BEST_EFFORT causes Go2 legs to collapse
- **`cloud_deskewed` is the best LiDAR source** — 11k pts, already in odom frame, no transform needed
- **Go2 LiDAR detects own body at ~0.2m** — `range_min=0.35m` filters self-detection
- **Go2 DDS exposes all internal topics** — `static_layer` removed from Nav2 costmaps to avoid malformed `/map` conflict
- **Nav2 bond stability** — Docker healthcheck disabled; `bond_timeout=30s`
- **CycloneDDS** — `MaxAutoParticipantIndex=120` to support Nav2's many nodes

## Credits

- Original SDK: [go2_ros2_sdk](https://github.com/abizovnuralem/go2_ros2_sdk) by [@abizovnuralem](https://github.com/abizovnuralem)
- Unitree Go2 message definitions from [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2)

## License

BSD-3-Clause
