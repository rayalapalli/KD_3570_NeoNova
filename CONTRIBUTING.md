# Contributing

This repository holds Team Neo Nova's (KD_3570) submissions for the eYRC 2025-26 Krishi Drone theme. It's mainly a personal/team record, but if you're a teammate contributing to it:

## Workflow

1. **Branch per task/feature**: `git checkout -b task/2a-waypoint-tuning`
2. **Keep sim and hardware code separate** — sim code lives under `simulation_ws/src/`, hardware code under `hardware_ws/src/`. Don't cross-import between them.
3. **Never overwrite `task_submissions/`** — that folder is a frozen, verbatim copy of what was actually graded. If you improve the code afterward, edit the copy under `simulation_ws/` or `hardware_ws/` instead.
4. **Bag files**: only add a bag file if it corresponds to a real recorded run you want to keep as evidence. Don't commit throwaway test bags.
5. **Commit messages**: `<task-id>: <short description>`, e.g. `2a: fix waypoint tolerance radius`.
6. Open a Pull Request into `main` and request a review before merging.

## Reporting issues

Open a GitHub Issue describing:
- Which task/package is affected
- Steps to reproduce
- Expected vs. actual behavior
- ROS 2 distro / OS you're running
