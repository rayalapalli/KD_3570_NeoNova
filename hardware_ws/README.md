# hardware_ws

This is the ROS 2 workspace for the **hardware phase** of KD_3570.

## 1. Pull in the official eYRC hardware packages

From inside `hardware_ws/`:

```bash
git clone -b kd_hw https://github.com/eYantra-Robotics-Competition/eyrc-25-26_krishi_drone.git --recursive src
```

This creates `hardware_ws/src/` with the official hardware-side packages:
`swift_pico_hw`, `swift_testing`, `local_grader`, `local_grader_5a`, `local_grader_5b`, plus the `whycon`, `controller_tuner`, and `crsf-ros2` submodules, and the drone `firmware.bin`.

## 2. Merge in this repo's own code

This repo's own authored controller lives under:

```
hardware_ws/src_ours/swift_pico_hw/scripts/
```

Copy it into the equivalent official package folder inside the freshly cloned `src/`:

```bash
cp src_ours/swift_pico_hw/scripts/*.py src/swift_pico_hw/scripts/
```

## 3. Build & source

```bash
colcon build
source install/setup.bash
```

## 4. Flash / bring-up

Follow the theme's official hardware bring-up checklist to flash `src/firmware.bin` to the flight controller and connect the companion computer over the CRSF link (`crsf-ros2`) before running any ROS 2 nodes.

## 5. Run (on the drone's companion computer)

```bash
ros2 run swift_pico_hw task_4b_hw_pico_controller.py
```

## 6. Replay recorded evidence

```bash
ros2 bag play src_ours/swift_pico_hw/bagfiles/task_4b
```
