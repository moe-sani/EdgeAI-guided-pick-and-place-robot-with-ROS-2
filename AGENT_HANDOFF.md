# Agent Handoff – ABB IRB-120 Sim on ROS 2 Jazzy

Context for resuming this work in a new session. Read `README.md` first for how to set up and run.

## Goal (from user)

The user wants a simulated ABB robot in ROS 2 doing something simple, such as pick-and-place.
Take whatever route is easiest. The user chose IFRA-Cranfield's `irb120_CranfieldRobotics`,
whose docs target Humble, but the machine runs Jazzy.

## Status: WORKING (2026-09-29)

- The full cube pick-and-place (`CubePP_irb120_sim`, 10 steps) runs successfully in Gazebo Harmonic.
- **Verified end-to-end twice**: by the agent on 2026-09-29, and by the user from an SSH terminal on
  2026-09-30. The robot picked the red cube and placed it in the box.
- The cube pose was checked before and after the run, so the physical move is confirmed, not just a "SUCCESS" message.
- `~/ros2_ws` is a git repo holding only our own files. Upstream clones in `src/` are git-ignored
  and pinned in `ros2_ws.repos`. No upstream code has been modified.

## Machine facts

- Ubuntu 24.04, ROS 2 Jazzy at `/opt/ros/jazzy`, 8 cores, 34 GB RAM, passwordless sudo.
- GNOME **Wayland** desktop. `env.sh` sets `DISPLAY=:0`, `XAUTHORITY` (a glob on
  `/run/user/1000/.mutter-Xwaylandauth.*`, because the name changes every login) and `XDG_RUNTIME_DIR`
  when `DISPLAY` is empty, so GUIs launched from SSH or VS Code terminals appear on the PC's monitor.
  This needs someone logged in on the desktop. `env.sh` also sets `QT_QPA_PLATFORM=xcb`.
  The user works mostly over SSH (VS Code remote) and confirmed on 2026-09-30 that this works.
- `~/.bashrc` runs `brew shellenv` several times (lines ~120-123) and sources ROS on line ~129.
  Homebrew's python3 then shadows `/usr/bin/python3`. We deliberately did **not** edit `.bashrc`.
  `env.sh` works around it instead.
- Installed via apt this session: ros-dev-tools, ros2-control, ros2-controllers, gripper-controllers,
  ros-gz, gz-ros2-control, rmw-cyclonedds-cpp. The OSRF Gazebo apt repo was also added.
  Gazebo itself comes via the `ros-jazzy-gz-*-vendor` packages; the `gz-harmonic` metapackage
  was NOT installed separately. MoveIt and xacro were already installed.
- `rosdep init` and `rosdep update` have been done.

## Key decisions and why

1. **Used upstream `jazzy` branches.** Both IFRA repos have `humble`, `humble-gzfortress` and `jazzy`
   branches, so no Humble to Jazzy porting was needed. The Jazzy versions use gz-harmonic, gz_ros2_control
   and IFRA_ObjectPose; IFRA_LinkAttacher is gone.
2. **Skipped the `move_group_interface_improved.h` step.** The Jazzy `ros2srrc_execution` code uses
   `std::unique_ptr<MoveGroupInterface>`, so the empty-constructor hack isn't needed. The provided
   header also targets an old MoveIt API (`group_name_` and similar) and would break `/opt/ros`.
3. **Only `abb_ros2_msgs` instead of the full PickNik `abb_ros2` driver.** `ros2srrc_execution` needs
   only `find_package(abb_robot_msgs)` at build time. The real-robot driver is only for `bringup_abb.launch.py`.
4. **Skipped the UR driver.** It's real-hardware only.
5. **Symlinked `~/dev_ws -> ~/ros2_ws`** instead of patching upstream. These files hardcode `~/dev_ws/src/...`:
   - `ros2srrc_launch/moveit2/moveit2.launch.py:126` and `gazebo/simulation.launch.py:126`. This
     path checks for end-effector controller configs. Without it, the gripper controllers don't load.
   - multi-arm launch files, and `irb120cranfield_ope` Python (YOLO model path).

## Architecture cheat-sheet

- Launch: `ros2srrc_launch moveit2.launch.py package:=<pkg> config:=<ID>`. Configs are listed in
  `<pkg>/config/configurations.yaml`. For irb120cranfield: `_1` = robot only, `_2` = robot + EGP-64
  gripper (pick-and-place), `_3` = gripper + camera (pose-estimation use case, needs a YOLO venv).
- The launch starts gz sim (world `ros2srrc_GzWorld`), ros2_control (`joint_trajectory_controller`,
  `joint_state_broadcaster`, `egp64_finger_left/right_controller` of type GripperActionController),
  move_group, RViz, and the custom action servers `/Move` (MoveJ/L/R/ROT/RP/G) and `/Robmove`
  (PTP/LIN to a pose), plus the `/Robpose` publisher.
- Programs are YAML files in `irb120cranfield/programs/`. Step types include MoveJ, RobMove and
  ParallelGripper. They run via `ros2 run ros2srrc_execution ExecuteProgram.py package:=... program:=...`.
- Objects: `SpawnObject.py --package irb120cranfield --sdf <Colour>Cube.sdf --name ... --x --y --z`.
  The pose is published on `/<Name>/ObjectPose`. Cubes available: Red, Green, Blue, White, Black.
- `CubePP_irb120_sim.yaml` picks at (0.60, 0.70, z 0.873) and places at (0.525, 0.125, z 0.925),
  using quaternion (0,1,0,0), i.e. the gripper pointing down.

## Ideas for next steps (not started)

- Write a custom program YAML (e.g. stack or sort several cubes), or a Python node that calls `/Robmove`.
- Try `irb120cranfield_3` with object pose estimation (needs a venv with opencv and ultralytics;
  see the upstream `instructions/Installation.md`).
- Add a one-shot launch or script that starts the sim, spawns the cube and runs the program.
