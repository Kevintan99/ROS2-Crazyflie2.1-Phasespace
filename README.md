# Crazyflie + Phasespace (ROS 2)
<p align="center">
  <img src="https://github.com/user-attachments/assets/d01af698-3d7d-4b2d-9b22-d2172bf69498"
       alt="Quadrotor with suspended load"
       width="400">
</p>

Robust ROS 2 workspace for flying Crazyflie nano-quadrotors using external motion capture from a Phasespace system. The project provides nodes and utilities for closed-loop control, log capture/replay for offline analysis, and PID tuning workflows.

---

## Project Overview
This repository provides a ROS 2 based framework for integrating Crazyflie nano quadrotors with Phasespace motion capture to enable precise, closed loop flight control and reproducible research workflows. It includes nodes and utilities to stream mocap poses into ROS 2, command one or more Crazyflies with position or velocity set points, and tune control parameters using a centralized YAML configuration for PID gains. To support experimental rigor, the project offers logging and replay capabilities so flight data can be captured, analyzed offline, and simulated without powering real hardware. The codebase is organized for rapid iteration, equally suitable for hardware in the loop testing or log driven development, and is designed to be extended with additional controllers, sensors, and multi drone behaviors. Typical use cases include benchmarking control strategies, validating perception to control pipelines, and teaching autonomous flight concepts with motion capture ground truth. Overall, the project aims to lower the barrier to reliable Crazyflie experiments while maintaining clarity, modularity, and repeatability.

## Repository Structure

```
├── build/ # colcon build artifacts (generated)
├── cache/ # build cache (generated)
├── install/ # colcon install space (generated)
├── log/ # colcon logs (generated)
├── src/ # ROS 2 packages / nodes (your source code)
├── flight_logs/
│ └── session_20250901_181718/ # example captured logs
├── launch_pid_optimization.yaml # PID tuning/launch parameters
├── test.py # example/test script for local checks
├── two_drone_logs.zip # example dataset (multi-CF run)
├── LICENSE # AGPL-3.0
└── README.md
```

## Key Features

- **Phasespace → ROS 2 bridge** for streaming precise pose into your control loop.
- **Crazyflie ROS 2 control** targeting position/velocity set-points.
- **PID tuning support** via `launch_pid_optimization.yaml` to iterate on gains.
- **Log capture & replay** for offline analysis and experiment reproducibility (see `flight_logs/`).
- **Multi-drone readiness** suggested by the included two-drone logs.

---

## Quick Start

### Prerequisites

- **OS:** Ubuntu 22.04 (recommended)  
- **ROS 2:** Humble (or your chosen distro; adjust commands accordingly)
- **Build tools:** `colcon`, `rosdep`, `vcstool` (optional)
- **Crazyflie stack:**  
  - USB radio (e.g., Crazyradio PA) and udev rules  
  - Crazyflie firmware & PC libraries as needed (e.g., `cflib` for Python tooling)
- **Phasespace system:** Arena/OWL server access (or a compatible ROS driver/bridge)  
- **Python:** 3.10+ for helper scripts

### Setup Instructions
1) Create a workspace if you don't already have one
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2) Clone the repository
```bash
git clone https://github.com/Kevintan99/Crazyflie-Phasespace-ROS2.git
```

3) Resolve dependencies (adjust distro name if not Humble
```bash
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -y
```

4) Build workspace
```bash
cd ~/ros2_ws
colcon build --symlink-install
# Source the workspace (add to your shell rc if you like)
source install/setup.bash
```

## Real-Flight Workflow (Hardware in the loop)
1. Power the Crazyflie and ensure the radio link is up (CLI or GUI tool).
2. Start Phasespace tracking and confirm rigid body streams.
3. Launch the ROS 2 Phasespace bridge node and check pose topics.
4. Launch the controller node with launch_pid_optimization.yaml (tune gains if needed).
5. Arm & take off sequence (manual or auto), then monitor telemetry.
6. Record logs (recommended) for later analysis/replay.

## Acknowledgements
- Bitcraze (Crazyflie ecosystem)
- Phasespace (motion capture)
- ROS 2 community
