# ROS 2 Jazzy – Simulated ABB IRB-120 Pick-and-Place

A simulated ABB IRB-120 with a Schunk EGP-64 gripper does a cube pick-and-place in
**Gazebo Harmonic**. The robot is driven by **MoveIt 2** on **ROS 2 Jazzy** (Ubuntu 24.04).

This workspace uses the `jazzy` branches of IFRA-Cranfield's
[irb120_CranfieldRobotics](https://github.com/IFRA-Cranfield/irb120_CranfieldRobotics) and
[ros2_SimRealRobotControl](https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl).
It was verified working on 2026-09-29: the red cube moved from (0.60, 0.70) to
(0.53, 0.13), and the program's place target is (0.525, 0.125).

A **local LLM** can now drive it with natural language: *"Pick up the red cube and place it in the
blue tray."* See [LLM control](#llm-control-local-llm-via-geniex).

## Repository layout

| Path | Purpose |
|---|---|
| `ros2_ws.repos` | Upstream source repos pinned to the verified commits |
| `env.sh` | Environment setup. **Always** `source` it before using the workspace. From SSH terminals it also points GUIs at the PC's logged-in desktop (`DISPLAY=:0`) |
| `AGENT_HANDOFF.md` | Detailed notes and gotchas for continuing the work (human or AI agent) |
| `src/llm_commander/` | **Our package**: natural-language control via a local LLM, plus the coloured-tray scene |
| `src/*` (everything else) | Upstream clones (git-ignored; restored via `vcs import`) |

## Setup from scratch

```bash
# 1. System dependencies (MoveIt + xacro are also required)
sudo apt install ros-jazzy-moveit ros-jazzy-xacro ros-dev-tools \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-gripper-controllers \
  ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-rmw-cyclonedds-cpp

# 2. Sources
cd ~/ros2_ws
vcs import src < ros2_ws.repos
source /opt/ros/jazzy/setup.bash
rosdep install -r --from-paths src --ignore-src --rosdistro jazzy -y   # gz-math7 warning is harmless

# 3. Upstream launch files hardcode ~/dev_ws, so point it at this workspace
ln -sfn ~/ros2_ws ~/dev_ws   # -n: safe to re-run (plain `ln -s` would nest a link inside)

# 4. Build. /usr/bin must come first in PATH so Homebrew's python3 isn't picked up
export PATH=/usr/bin:$PATH
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

## Run the pick-and-place demo

Terminal 1 starts Gazebo, MoveIt and RViz:

```bash
source ~/ros2_ws/env.sh
ros2 launch ros2srrc_launch moveit2.launch.py package:=irb120cranfield config:=irb120cranfield_2
```

Terminal 2 spawns a cube and runs the program. Wait until "You can start planning now!" appears
and the gripper controllers are active.

```bash
source ~/ros2_ws/env.sh
ros2 run ros2srrc_execution SpawnObject.py --package irb120cranfield --sdf RedCube.sdf \
  --name RedCube --x 0.6 --y 0.7 --z 0.95
ros2 run ros2srrc_execution ExecuteProgram.py package:=irb120cranfield program:=CubePP_irb120_sim
```

Useful checks:

```bash
ros2 control list_controllers        # expect joint_trajectory_controller + 2x egp64_finger_* active
ros2 topic echo /RedCube/ObjectPose  # cube position
ros2 topic echo /Robpose             # end-effector pose
```

To change the motion, edit
`src/irb120_CranfieldRobotics/irb120cranfield/programs/CubePP_irb120_sim.yaml`.
For single moves, you can send `/Move` or `/Robmove` actions directly. See the upstream
`instructions/Examples.md`.

## LLM control (local LLM via GenieX)

The `llm_commander` package turns commands like *"Pick up the red cube and place it in the blue tray"*
into robot motions. It works like this:

1. The node sees which cubes exist from their live `/<Name>/ObjectPose` topics in Gazebo.
2. It sends the command, plus the list of objects and destinations, to a local LLM through GenieX's
   OpenAI-compatible server.
3. The LLM replies with a small JSON plan. The node checks it against the known object and
   destination names; the LLM never supplies coordinates.
4. Each step (pick, place, pick-and-place or home) runs through `/Robmove` and `/Move`, using the
   cube's live pose. The gripper rotates to match the cube's yaw.
5. The node checks the result from the cube's pose: the cube must lift with the gripper, and must
   end up inside the destination. If a step fails, the remaining steps are skipped and the failure is
   reported.

Scene: RedCube, GreenCube and BlueCube on the table, a **BlueTray** and a **GreenTray** (spawned), and
the grey **Box** that's part of the robot cell. The positions and motion settings are in
`src/llm_commander/config/scene.yaml`.

```bash
# Terminal 1: the LLM server (listens on 127.0.0.1:18181)
geniex serve

# Terminal 2: the simulation (same command as above)
source ~/ros2_ws/env.sh
ros2 launch ros2srrc_launch moveit2.launch.py package:=irb120cranfield config:=irb120cranfield_2

# Terminal 3: spawn the trays and cubes, then talk to the robot
source ~/ros2_ws/env.sh
ros2 run llm_commander spawn_scene            # safe to re-run
ros2 run llm_commander commander              # interactive prompt; type 'exit' to quit
```

Example session:

```text
> Pick up the red cube and place it in the blue tray.
[llm 5.3s] I will pick up the red cube and place it in the blue tray.
[step 1/2] pick RedCube (hold)
  done: Holding RedCube.
[step 2/2] place held object -> BlueTray
  done: RedCube is now in BlueTray at (0.682, 0.398).
[done] All steps completed.
```

"Pick up X" on its own **only picks it up**: the robot lifts the cube and holds it in the air. A later
command such as "put it in the green tray" places it. The robot holds one cube at a time.

The commander works out what's held from the scene (a cube well above the table must be in the
gripper), so this also works across separate one-shot runs or after restarting the commander.

Other ways to use it:

```bash
ros2 run llm_commander spawn_scene --reset                          # put all cubes back at their start positions
                                                                    # (works while the commander runs; run it between
                                                                    # commands, not while the robot is moving)
ros2 run llm_commander commander "put the green cube in the box"    # run one command and exit
ros2 run llm_commander commander --dry-run                          # plan only, robot doesn't move
ros2 run llm_commander commander --model unsloth/Qwen3.5-2B-GGUF:Q4_0   # try another model
# From any ROS node or terminal (while the commander is running):
ros2 topic pub --once /llm_command std_msgs/msg/String "{data: 'go home'}"
ros2 topic echo /llm_response
```

The default model is `unsloth/Qwen3-4B-Instruct-2507-GGUF:Q4_0`. Planning took about 2–9 s per command
in testing. Supported actions are `pick`, `place`, `pick_and_place` and `home`. The LLM is told never
to invent a destination you didn't ask for. Plans that are impossible (placing while holding nothing,
or picking while already holding a cube) are rejected before the robot moves. Requests
for things that don't exist (e.g. "the yellow cube") or aren't robot tasks are politely refused.

**Current limitation:** every cube is dropped at the destination's centre, so putting several cubes
into the same tray will stack or knock them together.

## Known gotchas

- **Running over SSH**: `env.sh` sends the GUIs to the PC's own monitor (`DISPLAY=:0`). This only
  works while a user is logged in on that desktop. To view it remotely, use VNC or GNOME Remote Desktop.
- **Homebrew Python**: `~/.bashrc` runs `brew shellenv`, which puts Homebrew's `python3` first.
  That breaks colcon (`No module named 'catkin_pkg'`) and Python ROS nodes. `env.sh` fixes it.
- **Hardcoded `~/dev_ws`**: without the symlink, the gripper controllers silently don't load.
  `MoveG` and the gripper steps then fail.
- **Do not** apply the upstream "move_group_interface_improved.h" step. The Jazzy code doesn't
  need it, and it would break the installed MoveIt headers.
- The ABB and UR real-robot drivers aren't needed for simulation. Only `abb_robot_msgs`
  (from `abb_ros2_msgs`) is needed, so that the code compiles.
- Harmless log noise: "No 3D sensor plugin(s) defined for octomap updates",
  "/recognize_objects not available", "Publisher already registered", and segfaults
  (exit -11) on shutdown.
