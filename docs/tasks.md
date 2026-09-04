# Task Notes

## Task 1A — Plant Infection Detection

**File:** [`simulation_ws/src/swift_pico/scripts/task_1a_plant_infection_detection.py`](../simulation_ws/src/swift_pico/scripts/task_1a_plant_infection_detection.py)

Detects 4 ArUco markers (`DICT_4X4_100`) bounding the field of interest, computes their centers, extracts the region of interest between them, and analyzes it for signs of plant infection. Runs headless (matplotlib `Agg` backend) so it can be used in a grading pipeline without a display.

## Task 1C — Position Hold Controller

**File:** [`simulation_ws/src/swift_pico/scripts/task_1c_pico_controller.py`](../simulation_ws/src/swift_pico/scripts/task_1c_pico_controller.py)
**Evidence:** [bag file](../simulation_ws/src/swift_pico/bagfiles/task_1c)

ROS 2 node `pico_controller`. Holds the drone at a fixed setpoint `[-7, 0, 20]` using a per-axis PID loop on `/whycon/poses`, with an EMA filter (`alpha = 0.4`) smoothing the raw pose before it reaches the controller.

## Task 2A — Waypoint Navigation

**Files:**
- [`task_2a_waypoints_service.py`](../simulation_ws/src/swift_pico/scripts/task_2a_waypoints_service.py) — serves the fixed list of mission waypoints.
- [`task_2a_waypoint_action_server.py`](../simulation_ws/src/swift_pico/scripts/task_2a_waypoint_action_server.py) — `NavToWaypoint` action server; runs the PID/LQR loop per waypoint and reports success once the drone dwells inside the target sphere long enough.
- [`task_2a_waypoint_action_client.py`](../simulation_ws/src/swift_pico/scripts/task_2a_waypoint_action_client.py) — requests the waypoint list, then sends each waypoint as a goal in sequence.

**Evidence:** [bag file](../simulation_ws/src/swift_pico/bagfiles/task_2a), [rqt_graph](figures/KD_3570_rqt_graph.png)

Mission waypoints used for this run:

| # | x | y | z |
|---|---|---|---|
| hover | -7.00 | 0.00 | 29.22 |
| wp1 | -7.64 | 3.06 | 29.22 |
| wp2 | -8.22 | 6.02 | 29.22 |
| wp3 | -9.11 | 9.27 | 29.27 |
| wp4 | -5.98 | 8.81 | 29.27 |
| wp5 | -3.26 | 8.41 | 29.88 |
| wp6 | 0.87 | 8.18 | 29.50 |
| wp7 | 3.93 | 7.35 | 29.05 |

## Task 3C — *(pending)*

The submitted archive for this task (`KD_3570_task_3c.zip`) was empty when uploaded. Re-export the actual files and place them:
- raw submission → `task_submissions/task_3c/`
- working code → the matching package under `simulation_ws/` or `hardware_ws/`

Then update this file and the root `README.md` task table.

## Task 4B — Hardware PID Controller

**File:** [`hardware_ws/src/swift_pico_hw/scripts/task_4b_hw_pico_controller.py`](../hardware_ws/src/swift_pico_hw/scripts/task_4b_hw_pico_controller.py)
**Evidence:** [bag file](../hardware_ws/src/swift_pico_hw/bagfiles/task_4b)

Adapts the Task 1C controller for the real Swift Pico hardware:
- Uses `/drone/cmd/arming` service (with an `aux1` fallback) and forces low throttle throughout arming.
- Adds a throttle safety clamp immediately after arming.
- Uses a lighter EMA filter (`alpha = 0.2`) tuned for the noisier real WhyCon feed.
- Boosts the throttle Ki term when the positive altitude error exceeds 2 m, compensating for real drag/weight the simulator doesn't model.
- New setpoint for the hardware arena: `[0.0, -5.5, 19.0]`.
