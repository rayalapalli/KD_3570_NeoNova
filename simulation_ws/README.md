# simulation_ws

This is the ROS 2 workspace for the **simulation phase** of KD_3570.

## 1. Pull in the official eYRC simulation packages

From inside `simulation_ws/`:

```bash
git clone -b kd_sim https://github.com/eYantra-Robotics-Competition/eyrc-25-26_krishi_drone.git --recursive src
```

This creates `simulation_ws/src/` with the official Gazebo world, drone model, WhyCon package, message definitions (`swift_msgs`, `error_msg`, `controller_msg`, `waypoint_navigation`, etc.).

## 2. Merge in this repo's own code

This repo ships its own authored scripts separately, under:

```
simulation_ws/src_ours/swift_pico/scripts/
```

Copy (or symlink) them into the equivalent official package folder inside the freshly cloned `src/`:

```bash
cp src_ours/swift_pico/scripts/*.py src/swift_pico/scripts/
```

> Adjust the destination path if the official package's script folder is named differently — check `src/swift_pico/` after cloning.

## 3. Build & source

```bash
colcon build
source install/setup.bash
```

## 4. Run

```bash
ros2 launch <official_launch_package> <sim_world_launch_file>   # bring up Gazebo + WhyCon
ros2 run swift_pico task_1c_pico_controller.py                   # Task 1C
ros2 run swift_pico task_2a_waypoints_service.py                 # Task 2A
ros2 run swift_pico task_2a_waypoint_action_server.py
ros2 run swift_pico task_2a_waypoint_action_client.py
```

## 5. Replay recorded evidence

```bash
ros2 bag play src_ours/swift_pico/bagfiles/task_1c
ros2 bag play src_ours/swift_pico/bagfiles/task_2a
```
