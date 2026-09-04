# Architecture

## Overview

KD_3570 follows the standard eYRC Krishi Drone control architecture: a WhyCon marker-based localization system publishes drone pose, a PID controller node converts pose error into RC commands, and (from Task 2A onward) an action-server/action-client pair sequences the drone through a list of waypoints.

## Node & Topic Graph

See [`docs/figures/KD_3570_rqt_graph.png`](figures/KD_3570_rqt_graph.png) for the live `rqt_graph` snapshot captured during Task 2A.

### Simulation phase

| Node | Publishes | Subscribes |
|---|---|---|
| `whycon` (official pkg) | `/whycon/poses` | camera feed |
| `pico_controller` | `/drone_command`, `/pid_error` | `/whycon/poses` |
| `waypoints_service` | (service: `waypoints`) | — |
| `waypoint_server` (action server) | action feedback/result | `/whycon/poses`, calls `waypoints` service |
| `waypoint_client` (action client) | action goals | — |

### Hardware phase

| Node | Publishes | Subscribes |
|---|---|---|
| `whycon` | `/whycon/poses` | camera feed |
| `pico_controller` (hw) | `/drone/rc_command`, `/pid_error` | `/whycon/poses`, calls `/drone/cmd/arming` |

## Data Flow (Task 2A mission sequence)

1. `waypoint_client` requests the waypoint list from `waypoints_service`.
2. For each waypoint, `waypoint_client` sends a `NavToWaypoint` action goal to `waypoint_server`.
3. `waypoint_server` runs the PID/LQR loop, using live `/whycon/poses` data, until the drone stays inside the target sphere for the required dwell time.
4. On success, `waypoint_server` returns a result and `waypoint_client` advances to the next waypoint.

## PID Control Loop

Both the simulation (`task_1c_pico_controller.py`) and hardware (`task_4b_hw_pico_controller.py`) controllers follow the same structure:

1. Read raw pose from `/whycon/poses`.
2. Apply an EMA (exponential moving average) filter to smooth noisy pose readings.
3. Compute per-axis error against the desired setpoint `[x, y, z]`.
4. Run independent PID loops for roll, pitch, and throttle.
5. Clamp output to the valid RC range (`1000`–`2000`).
6. Publish the RC command and the raw error (for tuning/telemetry).

The hardware variant adds:
- An explicit arming service call (`/drone/cmd/arming`) before allowing any throttle above idle.
- A safety clamp on throttle immediately after arming.
- A conditional integral (Ki) boost on throttle when the positive altitude error exceeds 2 m, to overcome the extra real-world drag/weight the simulation doesn't model.
