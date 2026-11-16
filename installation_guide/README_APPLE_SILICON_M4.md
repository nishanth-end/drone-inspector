# Running the simulation on an Apple Silicon Mac (M4)

The simulation was written for Ubuntu 22.04 on x86_64. On an M4 Mac the
supported route is an **arm64 Ubuntu 22.04 container**. Apple Silicon runs it
natively, so there is no slow emulation.

## What works where

| Part | On the Mac (M4) | Notes |
| --- | --- | --- |
| PX4 SITL, DDS agent, ROS 2 Humble, Gazebo | In the arm64 container | Built from the same pinned versions |
| Gazebo window | Shown through XQuartz | Optional, set `VIEW=0` to skip |
| YOLO detection | Container uses CPU; the Mac can use the GPU (MPS) | Run `perception/yolo` natively for speed |
| Ground station (UDP telemetry) | Your Mac, port 14560 | `nc -ul 14560` shows live events |

> This has not been run on hardware yet. Treat the first start as a test and
> report what fails.

## 1. Install the tools (once)

```bash
# Docker Desktop or OrbStack (either works). Then in Terminal:
brew install --cask xquartz      # only for the Gazebo window
```

Open XQuartz, go to **Settings → Security**, tick **Allow connections from network clients**, then restart XQuartz and run:

```bash
xhost + 127.0.0.1
```

## 2. Get the code

```bash
git clone https://github.com/BigCoder-ops/drone-px4-ros2-yolo-sim.git ~/drone-inspector
cd ~/drone-inspector && git lfs pull
```

## 3. Build and run in the arm64 container

```bash
docker compose -f docker-compose.mac.yml build      # long first build (~20 GB)
docker compose -f docker-compose.mac.yml run --rm sim
# inside the container:
./sim.sh                         # orbit inspection
./sim.sh --sar                   # search and rescue in the mine world
```

Without a display (for example to check the mission logic only): `VIEW=0 docker compose -f docker-compose.mac.yml run --rm sim`.

## 4. Predefined-area patrol

```bash
ros2 run px4_orbit_inspection area_patrol_mission   # in a new tab, after the sim is up
ros2 service call /patrol/start std_srvs/srv/Trigger "{}"
```

The area and altitude are set in `ros2_ws/src/px4_orbit_inspection/config/area_patrol.yaml`.

## 5. Detection on the Mac GPU (optional)

```bash
cd perception/yolo
python3 -m venv .venv && source .venv/bin/activate
pip install ultralytics opencv-python
GROUND_STATION_HOST=127.0.0.1 python uav_camera_det.py
```

`YOLO_DEVICE` defaults to `mps` when available, so you only need to set it to force a device.
