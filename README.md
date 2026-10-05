# ROS 2 Jazzy – Simulated ABB IRB-120 Pick-and-Place

A simulated ABB IRB-120 with a Schunk EGP-64 gripper does a cube pick-and-place in
**Gazebo Harmonic**. The robot is driven by **MoveIt 2** on **ROS 2 Jazzy** (Ubuntu 24.04).

This workspace uses the `jazzy` branches of IFRA-Cranfield's
[irb120_CranfieldRobotics](https://github.com/IFRA-Cranfield/irb120_CranfieldRobotics) and
[ros2_SimRealRobotControl](https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl).
It was verified working on 2026-09-29: the red cube moved from (0.60, 0.70) to
(0.53, 0.13), and the program's place target is (0.525, 0.125).

A **local LLM** can now drive it with natural language: *"Pick up the red cube and place it in the
blue tray."* See [LLM control](#llm-control-local-llm-via-geniex). Optionally, a **safety camera** checked
by a local vision-language model refuses commands when a person is in the cell. See
[Safety camera](#safety-camera-vlm).

## Repository layout

| Path | Purpose |
|---|---|
| `ros2_ws.repos` | Upstream source repos pinned to the verified commits |
| `env.sh` | Environment setup. **Always** `source` it before using the workspace. From SSH terminals it also points GUIs at the PC's logged-in desktop (`DISPLAY=:0`) |
| `AGENT_HANDOFF.md` | Detailed notes and gotchas for continuing the work (human or AI agent) |
| `src/llm_commander/` | **Our package**: natural-language control via a local LLM, plus the coloured-tray scene |
| `src/*` (everything else) | Upstream clones (git-ignored; restored via `vcs import`) |
| `patches/`, `apply_patches.sh` | Our small changes to the upstream clones (no ABB logo, optional RViz); re-applied by the script |

## Setup from scratch

```bash
# 1. System dependencies (MoveIt + xacro are also required)
sudo apt install ros-jazzy-moveit ros-jazzy-xacro ros-dev-tools \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-gripper-controllers \
  ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-rmw-cyclonedds-cpp

# 2. Sources
cd ~/ros2_ws
vcs import src < ros2_ws.repos
./apply_patches.sh        # small upstream tweaks: hide the ABB logo, add rviz:=false (safe to re-run)
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
# add rviz:=false to skip the RViz window (everything else still works)
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

Scene: RedCube, GreenCube and BlueCube on the table, a **BlueTray** and a **GreenTray** (spawned), the
grey **Box** that's part of the robot cell, and a **SafetyCamera** in the top corner of the enclosure
(used only with `--safety`, see below). The positions and motion settings are in
`src/llm_commander/config/scene.yaml`.

```bash
# Terminal 1: the LLM server (listens on 127.0.0.1:18181)
geniex serve

# Terminal 2: the simulation (same command as above; RViz isn't needed here)
source ~/ros2_ws/env.sh
ros2 launch ros2srrc_launch moveit2.launch.py package:=irb120cranfield config:=irb120cranfield_2 rviz:=false

# Terminal 3: spawn the trays, cubes and camera, then talk to the robot
source ~/ros2_ws/env.sh
ros2 run llm_commander spawn_scene            # safe to re-run
ros2 run llm_commander commander              # interactive prompt; type 'exit' to quit
                                              # add --safety to enable the camera check (see below)
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
ros2 run llm_commander commander --model unsloth/Qwen3-4B-Instruct-2507-GGUF:Q4_0   # try another model
# From any ROS node or terminal (while the commander is running):
ros2 topic pub --once /llm_command std_msgs/msg/String "{data: 'go home'}"
ros2 topic echo /llm_response
```

The default model is `ai-hub-models/Qwen3-VL-4B-Instruct` (GenieX, NPU). It's a vision-language model
that does both the planning and the camera check. Planning took about 1–5 s per command in testing.
Leave off the `:W4A16` suffix that `/v1/models` lists; GenieX rejects it with an HTTP 500. The earlier
text-only default, `unsloth/Qwen3-4B-Instruct-2507-GGUF:Q4_0`, also works with `--model`.
Supported actions are `pick`, `place`, `pick_and_place` and `home`. The LLM is told never
to invent a destination you didn't ask for. Plans that are impossible (placing while holding nothing,
or picking while already holding a cube) are rejected before the robot moves. Requests
for things that don't exist (e.g. "the yellow cube") or aren't robot tasks are politely refused.

**Current limitations:**
- Every cube is dropped at the destination's centre, so putting several cubes into the same tray will
  stack or knock them together.
- In the simulation, a held cube slowly slides out of the gripper (it drops within roughly 15–60 s).
  "Pick up X" followed by "put it in…" should be done promptly.

## Safety camera (VLM)

With `--safety`, every command starts with a camera check before the LLM plans and before the robot
moves. The latest SafetyCamera frame goes to the same VLM with the question:

> *Is there a person, or anything else risky, on the way of the robot? reply with yes or no only*

**Changing the question:** in the web UI's safety card (see below), or permanently with `safety.prompt`
in `scene.yaml`. The question must make the model answer **yes** for risky and **no** for safe; any other
answer is treated as risky. A custom question applies until the commander restarts.

On `yes`, the command is refused and the robot doesn't move. Anything other than a clear `no` (no
camera image, a stale image, server error, unexpected answer) also counts as risky. The check takes
about 0.7 s. The commander starts the gz → ROS image bridge for `/safety_camera/image` itself.

To try it, sit a person (Gazebo Fuel's *Rescue Randy Sitting*, child-sized) on the table in the robot's
path, between the red cube and the blue tray:

```bash
ros2 run llm_commander commander --safety                 # terminal 3
ros2 run llm_commander spawn_person                       # terminal 4: person appears (re-run to put them back)
ros2 run llm_commander spawn_person --remove              # ...and goes away again
ros2 run llm_commander spawn_person --x 0.6 --y 0.3 --yaw 90   # somewhere else
ros2 run llm_commander safety_check --repeat 3            # just ask the VLM, no robot involved
ros2 run llm_commander safety_check --prompt "Is there a blue tray? reply with yes or no only"   # try a question
```

```text
> put the red cube in the box
[command] put the red cube in the box
[safety] RISK (VLM answered 'yes', 0.7s). Command refused; the robot will not move.
```

To see what the camera sees:
- **Gazebo window (live, nothing extra needed):** open the **⋮** menu at the top right, add
  **Image Display**, then choose `/safety_camera/image` in its topic dropdown.
- **rqt (live):** `ros2 run rqt_image_view rqt_image_view /safety_camera/image`. This needs the image
  bridge, which `commander --safety` starts. Otherwise run
  `ros2 run ros_gz_bridge parameter_bridge "/safety_camera/image@sensor_msgs/msg/Image[gz.msgs.Image"`.
- **What the VLM actually judged:** the last checked frame is saved to `/tmp/llm_commander_safety.jpg`.
  Over SSH, open it in VS Code with `code /tmp/llm_commander_safety.jpg`; it's rewritten after every
  check.

Notes:
- The check happens once, when a command starts. The robot isn't monitored while it moves.
- **The person has a physics body**, so you can see the risk. Turn safety off, spawn the person and send
  *"Pick up the red cube and place it in the blue tray."* The arm runs into them, shoves and turns them
  (about 8 cm and 35° in testing), and knocks the cube out of the gripper. The command then fails with
  *"RedCube did not end up inside BlueTray"*. With safety on, the same command is refused.
  - Physics uses one 14 × 14 × 40 cm box around the torso and head (5 kg). The legs are visual-only,
    so the person can sit over cubes without shoving them.
  - MoveIt doesn't know about the person, so it plans straight through them.
- With the question worded only as *"anything risky…"*, Qwen3-VL-4B answered `no` even with a person
  in view. Naming "a person" explicitly fixed that, with no false alarms on the normal scene (including
  while the arm moves near the camera).

## Web UI

A one-page browser UI built into the commander. It uses only the Python standard library and plain
HTML/JS, so there's nothing to install.

```bash
# GenieX and the simulation running as above, then:
source ~/ros2_ws/env.sh
ros2 run llm_commander commander --web                    # http://localhost:8080
ros2 run llm_commander commander --web --safety           # same, with the safety check already on
ros2 run llm_commander commander --web --host 0.0.0.0     # reachable as http://<robot-PC-IP>:8080 on the LAN
```

- **Left:** chat with the robot. Click an example to fill in the box, or type your own command. It shows the
  same messages as the terminal, colour-coded: your commands in blue, LLM replies in purple, safety
  refusals in red.
- **Right:**
  - the live safety-camera view;
  - the result of the last safety check (CLEAR/RISK), with the exact frame the VLM judged;
  - the buttons: **Reset scene** (gripper open, robot home, cubes back, and any missing trays,
    cubes or camera re-spawned, which is handy after restarting the sim), **Robot home**,
    **Check now**, **Spawn person** and **Remove person**.
- **Safety check button** (in the safety card, also the `safety ON/OFF` pill at the top right): turns
  the camera check on or off without restarting. It's green when on.
- **Safety question** (in the safety card): the question asked about each frame. If you clear the box,
  the default question shows as placeholder text. **Editable only while the safety check is off**; the
  server enforces this too, so the rules can't change while they're being applied.
  - Edit it, then press **Save** or Enter; **Default** goes back to the built-in question.
  - Use **Check now** to try it before turning the check on.
  - The change is logged in the chat, and lasts until the commander restarts.

Commands and the Reset/Home buttons queue up behind the robot's current job, so they never collide.
The person buttons and **Check now** run immediately. The terminal prompt keeps working alongside the page.

**Opening it from your own machine:** the default `127.0.0.1` only accepts local connections, so you
have two options:
- Use VS Code's port forwarding (**Ports** panel → forward port 8080, then open `http://localhost:8080`).
- Start it with `--host 0.0.0.0` and open `http://<robot-PC-IP>:8080`. There's no login, so anyone on
  that network can drive the robot. That's fine on a trusted LAN, but don't expose it further.

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
