# Agent Handoff – ABB IRB-120 Sim on ROS 2 Jazzy

Context for resuming this work in a new session. Read `README.md` first for how to set up and run.

## Goal (from user)

The user wants a simulated ABB robot in ROS 2 doing something simple, such as pick-and-place.
Take whatever route is easiest. The user chose IFRA-Cranfield's `irb120_CranfieldRobotics`,
whose docs target Humble, but the machine runs Jazzy.

**Phase 2 (2026-09-30):** the user wants a local LLM to command the robot in natural language, e.g.
"Pick up the red cube and place it in the blue tray." They run LLMs with **GenieX** (Qualcomm).
The user chose to add coloured trays to the scene so that command works literally. They then asked to
skip the harder multi-cube case and commit this stage.

**Phase 3 (2026-09-30):** the user wants a camera in the scene, with a local VLM asked whether anything
risky is in the robot's way, used to abort the command. The test hazard is a simulated child. The
existing demo must keep working unchanged (so the feature is opt-in via `--safety`). The user asked
to use `ai-hub-models/Qwen3-VL-4B-Instruct` **for everything**, for planning too. The user then
**explicitly simplified the scope**:
- one check at the start of each command, with no mid-motion monitoring or watchdog;
- the short yes/no question;
- *Rescue Randy Sitting* as the person model, because Casual female was too large for the scene.

**Phase 4 (2026-10-01):** a simple GUI to chat, reset the scene and robot, spawn the person and see
safety triggers. Agent proposed and user approved a **web page built into the commander** (`--web`)
rather than rqt or RViz, because the user works over SSH from another machine: a browser page reaches
them, whereas X windows land on the PC's monitor. The user opens it at **http://192.168.1.184:8080**
(the robot PC's LAN IP), which needs `--host 0.0.0.0`; VS Code didn't auto-forward the port.

## Status: WORKING (2026-09-30)

- **Phase 1:** the upstream `CubePP_irb120_sim` program (10 steps) runs in Gazebo Harmonic. Verified
  by the agent (2026-09-29) and by the user over SSH (2026-09-30).
- **Phase 2: `src/llm_commander` works end to end.** These were verified from the cube poses after
  each run:
  - "Pick up the red cube and place it in the blue tray." ended 2 mm from the tray centre.
  - "put the green one in the green tray, then move the blue cube to the box" (two steps).
  - A cube rotated 30° was picked with a yaw-aligned grasp; its final yaw was 29.8°, which confirms the rotation sign.
  - "go home", "pick up the yellow cube" (correctly refused), `--dry-run`, the interactive prompt
    (tested through a pty), `/llm_command` → `/llm_response` topic mode (dry run), and `spawn_scene --reset`.
  - **Pick and hold** (added after the user found that "Pick up the red cube" went on to put it in the
    Box): "Pick up the red cube" held it at z 1.007, then a *separate* one-shot run of "put it in the
    blue tray" placed it 3 mm from the tray centre. A 15-case planner matrix (with and without a held
    cube) was all correct.
- **Phase 3: safety camera works** (agent-verified in the sim, with `--safety`):
  - Empty cell: `clear` (0.7 s), and the full pick-and-place completed. There were no false alarms,
    including frames with the arm close to the camera (checked during an earlier watchdog prototype).
  - With the child spawned: `RISK`, the command was refused and the cube didn't move. After
    `spawn_person --remove`, the same command ran normally. Without `--safety`, no camera is needed.
  - The planning matrix with Qwen3-VL-4B was **15/15** after two fixes: JSON repair for a missing
    closing brace, and restoring the prompt (see the gotchas).
- **Phase 4: web UI works** (agent-verified by clicking through the real page in a browser):
  - Safety switch on, then **Spawn child**, then the example command: a red RISK line and badge,
    the judged frame showing the child, and no motion.
  - **Remove person**, then **Reset scene** (gripper open, home, cubes reset), then the same command:
    CLEAR, and RedCube ended up in BlueTray.
  - Restarting the commander while a page is open: the page detects it (`boot` id) and reloads the log.
- **Commit rule (from user):** commit only as the configured git identity (Moe Sani). Never use an
  AI or agent name as author or committer, and never add `Co-authored-by` trailers.
- `~/ros2_ws` is a git repo (the user pushes it to their GitHub). Upstream clones in `src/*` are
  git-ignored and pinned in `ros2_ws.repos`. `src/llm_commander` is ours and committed.
  No upstream code has been modified.

## Machine facts

- Ubuntu 24.04 on **aarch64 (Snapdragon)**, ROS 2 Jazzy at `/opt/ros/jazzy`, 8 cores, 34 GB RAM, passwordless sudo.
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

## llm_commander (phase 2) design

- **Files** (`src/llm_commander/`, ament_python):
  - `config/scene.yaml` is the single source of truth: cube and tray positions, the Box, motion
    settings, and the LLM URL and model.
  - `llm_commander/scene.py`: loads the config, computes destination geometry, and generates the tray
    SDF (a static floor plus 4 walls, coloured).
  - `llm_commander/spawn_scene.py`: spawns via ROS `/world/ros2srrc_GzWorld/create`. It checks
    `gz model --list` first, because Gazebo's create silently ignores duplicate names but still
    returns success. `--reset` teleports cubes back using the `gz service .../set_pose` CLI, since
    `set_pose` is **not** bridged to ROS.
  - `llm_commander/llm.py`: the prompt, the HTTP call (stdlib `urllib`; the `openai` package isn't
    installed), JSON extraction, and strict validation. Names are matched case- and
    punctuation-insensitively.
  - `llm_commander/robot.py`: `/Robmove` and `/Move` action clients, plus the skills `pick` (grasp,
    lift and hold), `place`, `pick_and_place` and `home`. `holding()` is **stateless**: any cube with
    z > table + 0.015 + 0.05 is in the gripper. That's why it survives restarts, one-shot runs and resets.
  - `llm_commander/commander.py`: the node. It discovers `objectpose_msgs/ObjectPose` topics every
    1 s, has a worker queue, an interactive prompt (when stdin is a TTY), a single-command CLI mode,
    `/llm_command` in, `/llm_response` out, `--dry-run`, and `--safety` (a camera check at command start,
    *before* planning, so a refusal isn't preceded by the LLM's past-tense reply).
  - `llm_commander/safety.py`: `SafetyMonitor` subscribes to `/safety_camera/image`, downsizes the
    latest frame to 640 px JPEG, and asks the VLM a yes/no question (`max_tokens=5`). It is
    **fail-safe**: no frame, a frame older than 2 s, a server error or a non-yes/no answer all count as
    risky. `start_bridge()` runs `ros_gz_bridge parameter_bridge` for the image topic unless something
    already publishes it; it's killed at exit. The `safety_check` CLI is for tuning.
  - `llm_commander/spawn_person.py`: a visual-only *Rescue Randy Sitting* (Fuel mesh URL, auto-cached),
    child = 0.6×, adult = 1.0×. Its default spot and heading are `person_spot` in scene.yaml. Re-running
    it moves the person; `--remove` removes it.
  - `llm_commander/webui.py` and `web/index.html`: the `--web` UI, stdlib `ThreadingHTTPServer`
    in a daemon thread inside the commander, plus one vanilla-JS page that polls `/api/state` every
    0.7 s and reloads `/api/camera.jpg` (JPEG encoded on request from the latest ROS frame).
    - Chat goes through `/api/command`, which puts a string on `node.commands`.
    - The buttons use `/api/action`. `reset` and `home` are queued as **callables** on the same
      queue, because the worker runs strings via `run_command` and callables directly, so they
      never overlap robot motion. The person buttons, `check` and `safety_on/off` run immediately.
    - Person and scene buttons shell out to `ros2 run llm_commander spawn_person|spawn_scene`, and
      their output is logged as `[scene]` lines.
    - `node.say()` also appends to a 500-line ring buffer `(seq, kind, text)`. `kind` comes from
      the message prefix and drives the colours.
    - `--web` always creates the `SafetyMonitor`, so the camera is shown even with safety off.
      `node.safety_enabled` gates commands and can be toggled at runtime.
  - SafetyCamera: a static model with a gz camera sensor, spawned by `spawn_scene` at (0.70, 1.02, 1.65),
    looking at (0.42, 0.45, 0.87), 800×600 at 5 Hz, hfov 1.4. The upstream world already loads the gz
    Sensors system, so no launch or world change was needed. Cameras outside the glass enclosure, or at
    z ≥ 1.9, see mostly glass or the roof block.
- **LLM output contract:** `{"reply": str, "plan": [...]}`. Steps are `{"action": "pick", "object"}`,
  `{"action": "place", "destination"}`, `{"action": "pick_and_place", "object", "destination"}` or
  `{"action": "home"}`. The prompt includes "currently holding: X". `plan()` steps through the plan
  tracking what's held, and rejects placing with nothing held or picking a second cube.
  The LLM never outputs coordinates. The prompt lists only the objects whose pose is currently being
  published. Temperature is 0.
  - Prompt rules that were needed for Qwen3-4B: never invent a destination (otherwise "pick up X" went
    to the Box), and don't turn an unresolvable "it" into "home".
- **Pick sequence:** open gripper → PTP above the cube (z 1.0) → LIN down to `cube_z - 0.008` →
  MoveG 55 → LIN up. The grasp is confirmed by checking that the cube's z rose by more than 5 cm.
  Then PTP above the destination centre → LIN down to the release height → open → LIN up. Finally the
  cube's pose must be inside the destination footprint.
  - The grasp quaternion is `(-sin(ψ/2), cos(ψ/2), 0, 0)`, where ψ is the cube's yaw folded into ±45°.
- **Measured geometry (world frame):** table top z = 0.8665 (flat; a cube resting on it has its centre
  at z = 0.8815). The ergobox spans x 0.476–0.576 and y 0.049–0.203, with its top at z 0.935. Trays
  are at BlueTray (0.68, 0.40) and GreenTray (0.35, 0.20). Cubes start at Red (0.60, 0.70),
  Green (0.48, 0.80) and Blue (0.36, 0.86). All of these were reachable.
- **GenieX:** `geniex serve` listens on `127.0.0.1:18181` (OpenAI-compatible; `/v1/models` lists the
  models). The default model is now **`ai-hub-models/Qwen3-VL-4B-Instruct`** (NPU; the alias
  `qualcomm/Qwen3-VL-4B-Instruct` also works). Planning takes 1–5 s, and an image yes/no answer about 0.7 s.
  - **Never append `:W4A16`**. `/v1/models` lists it, but the server answers HTTP 500 "quantization
    'W4A16' not found". That was also the cause of the earlier NPU-model 500s.
  - The previous default, `unsloth/Qwen3-4B-Instruct-2507-GGUF:Q4_0`, still works via `--model`.
  - A server started from an agent shell dies when that shell is interrupted; start it detached.
  - Port 11434 on this machine is a different service (probably Ollama) and is unrelated.

## Gotchas found in phase 2

- **Don't name Node methods or attributes `handle`**: it shadows rclpy's `Node.handle` and crashes.
  Also, a method and an attribute with the same name (e.g. `robmove`) silently clash.
- rclpy **segfaulted on exit** after action-client use. The fix is ordered shutdown plus `os._exit(0)`
  at the end of `commander.main()`.
- The small model sometimes emits plan steps as bare strings (`"home"`); the parser accepts that.
- **Cube-on-tray height is 7.5 mm higher than the geometry predicts** (centre z 0.8925 instead of
  0.8855), even when a cube is dropped straight in. This is a Gazebo contact artefact between the
  mesh and a primitive box, not a placement error. It's harmless because grasps use the live pose.
- A cube dropped into the Box can tumble onto another face. That's fine for a cube.
- `ros2 pkg executables` reporting "Package not found" right after a build happened only because it
  ran concurrently with the build.
- Gazebo CLI: `/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz` (not on PATH by default).

## Gotchas found in phase 3

- **Small-model prompts are brittle.** Changing only the `reply` wording ("...what you will do")
  broke 3 of the 15 planning cases. Always re-run `python3 src/llm_commander/tools/plan_matrix.py`
  (it should report 15/15) after touching `SYSTEM_PROMPT` or the model. The matrix
  covers 10 cases with nothing held and 5 holding RedCube: two "it" references, pick-only, grab,
  pick+place, a two-step command, "all cubes", yellow cube, weather, and home.
- **Safety question wording:** "Is there anything risky on the way of the robot? reply with yes or no
  only" (the user's wording) → `no` even with a person in the frame, with or without a system prompt.
  Adding "a person, or" → correct on both frames. A long JSON prompt listing the "safe" objects made the
  model echo the list and answer "safe" with a person in view.
- Qwen3-VL sometimes drops the final `}` of the JSON; `_close_brackets()` in llm.py repairs it.
- **A held cube slips out of the gripper** in Gazebo within about 15–60 s (MoveG 55; 65 and 75
  measured the same). The user accepted this; it's why mid-motion stops (which leave a cube held)
  were dropped along with the watchdog.
- The Rescue Randy mesh is in 0.1 mm units, with its origin off-centre and the legs along −y; `person_sdf()`
  re-centres it. The mesh URL contains spaces, and Gazebo resolves that fine for Fuel URLs.

## Gotchas found in phase 4

- The sim (Gazebo launch) does **not** include the trays, cubes or camera. After every sim restart, run
  `spawn_scene` or press **Reset scene** in the web UI.
- The web UI binds `127.0.0.1` by default. The user needs `--host 0.0.0.0` (no auth, so LAN only).
- The agent's browser tool runs on the user's machine, so it can't reach the PC's `localhost`. Test
  through `http://192.168.1.184:8080`.
- Verdict "age" is computed server-side, so it doesn't depend on the browser machine's clock.
- Don't run two commanders that both subscribe to `/llm_command`: a published command would execute
  twice. Commands from the web or a terminal prompt only go to their own process.

## Ideas for next steps (not started)

- **Several cubes into one destination** (the user deferred this). Right now every cube drops at the
  destination centre. Plan: give each destination a list of slots (e.g. 2×2 at ±0.035 m in a 0.14 m
  tray), pick the first slot with no other object within about 0.05 m, and put the cube back if the
  destination is full. A draft was written and then reverted untested at the user's request.
- Conversation memory ("now put it back"): pass recent turns and the last plan to the LLM.
- Safety: monitor during motion too. The upstream `/Robmove` and `/Move` servers accept cancel, which
  calls MoveIt `stop()`. A watchdog prototype (cancelling the active goal on `yes`) was written and then
  removed at the user's request. The held-cube slip would need fixing first.
- Add a one-shot launch that starts the sim, spawns the scene and starts the commander.
- Try `irb120cranfield_3` with object pose estimation (needs a venv with opencv and ultralytics;
  see the upstream `instructions/Installation.md`).
