"""ROS 2 node: natural-language commands -> local LLM plan -> robot pick-and-place.

Commands come from the interactive prompt (when run in a terminal) or the /llm_command topic
(std_msgs/String). Progress and results are printed and published on /llm_response.

Usage:  ros2 run llm_commander commander [--web] [--safety] [--config scene.yaml] [--model NAME] [--url URL] [--dry-run]

--web: also serve a browser UI (chat, live camera, scene buttons) on http://localhost:8080.

--safety: before the robot starts a command, the safety camera image is checked by the VLM; if it sees anything
risky, the command is refused and the robot does not move.
"""
import argparse
import collections
import os
import queue
import subprocess
import sys
import threading
import time

import rclpy
from objectpose_msgs.msg import ObjectPose
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from llm_commander.llm import LLMPlanner, PlanError
from llm_commander.robot import RobotSkills, SkillError
from llm_commander.safety import SafetyMonitor, start_bridge, stop_bridge
from llm_commander.scene import destinations, load_scene
from llm_commander.voice import Speaker, Transcriber
from llm_commander.webui import start_web_ui

POSE_TYPE = 'objectpose_msgs/msg/ObjectPose'
POSE_MAX_AGE_S = 2.0


class Commander(Node):
    def __init__(self, scene, planner, dry_run=False, safety=False, camera=False):
        super().__init__('llm_commander')
        self.scene, self.planner, self.dry_run = scene, planner, dry_run
        self.dests = destinations(scene)
        self.cb = ReentrantCallbackGroup()
        self.poses, self.pose_stamp, self._pose_subs = {}, {}, {}
        self.lock = threading.Lock()
        self.commands = queue.Queue()  # command strings, or callables for UI actions (run in order)
        self.busy = threading.Event()
        self.log_lock = threading.Lock()
        self.log_lines = collections.deque(maxlen=500)  # (seq, kind, text) for the web UI
        self.log_seq = 0
        self.last_verdict = None  # dict(risky, reason, seconds, at) of the latest safety check

        self.pub = self.create_publisher(String, '/llm_response', 10)
        self.create_subscription(String, '/llm_command', lambda m: self.commands.put(m.data), 10,
                                 callback_group=self.cb)
        self.create_timer(1.0, self._discover_objects, callback_group=self.cb)
        self.skills = RobotSkills(self, scene, self.dests, self.get_pose, self.all_poses, self.say)
        # The camera can be connected without gating commands (the web UI shows it and can toggle safety).
        self.monitor = (SafetyMonitor(self, scene, planner, callback_group=self.cb)
                        if (safety or camera) else None)
        self.safety_enabled = bool(safety)
        self.voice = None  # Transcriber, set by main() when --web and whisper.cpp is available
        self.speaker = None  # Speaker (Piper TTS), set by main() when --web and Piper is available

    # ---------- object pose tracking ----------
    def _discover_objects(self):
        for topic, types in self.get_topic_names_and_types():
            if POSE_TYPE in types and topic not in self._pose_subs:
                self._pose_subs[topic] = self.create_subscription(
                    ObjectPose, topic, self._on_pose, 10, callback_group=self.cb)

    def _on_pose(self, msg):
        with self.lock:
            self.poses[msg.objectname] = msg
            self.pose_stamp[msg.objectname] = time.monotonic()

    def get_pose(self, name):
        time.sleep(0.3)  # let a fresh pose arrive after the robot stops
        with self.lock:
            if time.monotonic() - self.pose_stamp.get(name, 0.0) > POSE_MAX_AGE_S:
                return None
            return self.poses.get(name)

    def live_objects(self):
        return sorted(self.all_poses())

    def all_poses(self):
        now = time.monotonic()
        with self.lock:
            return {n: p for n, p in self.poses.items()
                    if now - self.pose_stamp[n] < POSE_MAX_AGE_S and n not in self.dests}

    # ---------- command handling ----------
    @staticmethod
    def _kind(text):
        for prefix, kind in (('[voice]', 'scene'), ('[command]', 'user'), ('[llm ', 'llm'), ('[safety] RISK', 'risk'),
                             ('[safety] clear', 'clear'), ('[safety]', 'warn'), ('[error]', 'error'),
                             ('[failed]', 'error'), ('[done]', 'done'), ('[scene]', 'scene')):
            if text.startswith(prefix):
                return kind
        return 'info'

    def say(self, text):
        print(text, flush=True)
        self.pub.publish(String(data=text))
        with self.log_lock:
            self.log_seq += 1
            self.log_lines.append((self.log_seq, self._kind(text), text))

    def log_since(self, seq):
        with self.log_lock:
            return [line for line in self.log_lines if line[0] > seq], self.log_seq

    def check_safety(self):
        v = self.monitor.check()
        self.last_verdict = dict(risky=v.risky, reason=v.reason, seconds=round(v.seconds, 1), at=time.time())
        if v.risky:
            self.say(f'[safety] RISK ({v.reason}, {v.seconds:.1f}s).')
        else:
            self.say(f'[safety] clear ({v.seconds:.1f}s)')
        return v

    # ---------- UI actions ----------
    def run_tool(self, *args):
        """Run one of this package's CLI tools (spawn_scene, spawn_person) and log its result lines."""
        out = subprocess.run(['ros2', 'run', 'llm_commander', *args], capture_output=True, text=True, timeout=120)
        for line in (out.stdout + out.stderr).splitlines():
            line = line.split(']: ', 1)[-1].strip()
            if line and not line.startswith('Waiting for'):
                self.say(f'[scene] {line}')
        if out.returncode != 0:
            self.say(f"[error] {' '.join(args)} exited with code {out.returncode}")

    def reset_scene(self):
        """Open the gripper, send the robot home, and put all cubes (and missing scene items) back."""
        self.say('[scene] Resetting: gripper open, robot home, cubes back to start...')
        if not self.dry_run:
            try:
                self.skills.gripper(0.0)
                self.skills.home()
            except SkillError as e:
                self.say(f'[failed] {e}')
        self.run_tool('spawn_scene', '--reset')
        self.say('[done] Scene reset.')

    def run_step(self, step):
        a = step['action']
        if a == 'home':
            self.skills.home()
            return 'robot is at home.'
        if a == 'pick':
            return self.skills.pick(step['object'])
        if a == 'place':
            return self.skills.place(step['destination'])
        return self.skills.pick_and_place(step['object'], step['destination'])

    def run_command(self, command):
        objects = self.live_objects()
        self.say(f'[command] {command}')
        if self.safety_enabled:
            if self.check_safety().risky:
                self.say('[safety] Command refused; the robot will not move.')
                return
        holding = self.skills.holding()
        t0 = time.time()
        try:
            reply, plan, _ = self.planner.plan(command, objects, list(self.dests), holding)
        except PlanError as e:
            self.say(f'[error] {e}')
            return
        self.say(f'[llm {time.time() - t0:.1f}s] {reply}')
        if not plan:
            return
        for i, step in enumerate(plan, 1):
            a = step['action']
            desc = {'home': 'home',
                    'pick': f"pick {step.get('object')} (hold)",
                    'place': f"place held object -> {step.get('destination')}",
                    'pick_and_place': f"pick {step.get('object')} -> {step.get('destination')}"}[a]
            self.say(f'[step {i}/{len(plan)}] {desc}')
            if self.dry_run:
                continue
            try:
                self.say('  done: ' + self.run_step(step))
            except SkillError as e:
                self.say(f'[failed] {e} Stopping the remaining steps.')
                return
            except Exception as e:  # keep the commander alive on unexpected errors
                self.say(f'[failed] Unexpected error: {e!r}. Stopping the remaining steps.')
                return
        self.say('[done] All steps completed.')

    def worker(self):
        while rclpy.ok():
            try:
                cmd = self.commands.get(timeout=0.5)
            except queue.Empty:
                continue
            self.busy.set()
            try:
                if callable(cmd):
                    cmd()
                else:
                    self.run_command(cmd.strip())
            except Exception as e:  # keep the worker alive whatever happens
                self.say(f'[failed] Unexpected error: {e!r}')
            finally:
                self.busy.clear()
                self.commands.task_done()


def repl(node):
    time.sleep(2.0)  # allow object discovery before the first prompt
    print(f"\nObjects: {', '.join(node.live_objects()) or '(none yet - run spawn_scene)'}")
    print(f"Destinations: {', '.join(node.dests)}")
    print("Type a command (e.g. 'Pick up the red cube and place it in the blue tray.'), or 'exit'.\n")
    while rclpy.ok():
        try:
            line = input('> ').strip()
        except (EOFError, KeyboardInterrupt):
            break
        if line.lower() in ('exit', 'quit', '/exit'):
            break
        if line:
            node.commands.put(line)
            time.sleep(0.2)
            node.commands.join()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--config', default=None, help='scene.yaml (default: installed config)')
    parser.add_argument('--model', default=None, help='override llm.model from the config')
    parser.add_argument('--url', default=None, help='override llm.url from the config')
    parser.add_argument('--dry-run', action='store_true', help='plan only, do not move the robot')
    parser.add_argument('--safety', action='store_true',
                        help='check the safety camera with the VLM before running each command')
    parser.add_argument('--web', action='store_true', help='serve the browser UI')
    parser.add_argument('--port', type=int, default=8080, help='web UI port (default 8080)')
    parser.add_argument('--https', action='store_true',
                        help='serve the web UI over HTTPS (self-signed); browsers need this for the microphone')
    parser.add_argument('--host', default='127.0.0.1',
                        help='web UI address (default 127.0.0.1; use 0.0.0.0 to allow other machines)')
    parser.add_argument('command', nargs='*', help='run a single command and exit')
    args, _ = parser.parse_known_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    scene = load_scene(args.config)
    llm_cfg = scene['llm']
    planner = LLMPlanner(args.url or llm_cfg['url'], args.model or llm_cfg['model'], llm_cfg.get('timeout_s', 120))

    rclpy.init()
    node = Commander(scene, planner, dry_run=args.dry_run, safety=args.safety, camera=args.web)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    bridge = None
    if node.monitor:
        bridge = start_bridge(node, scene['camera']['topic'])
        if not node.monitor.wait_for_frame():
            node.say(f"[safety] WARNING: no image on {scene['camera']['topic']} yet. Run "
                     f"`ros2 run llm_commander spawn_scene` to add the SafetyCamera"
                     f"{'; until then commands are refused' if args.safety else ''}.")
    threading.Thread(target=node.worker, daemon=True).start()
    node.say(f'[llm_commander] model={planner.model}'
             f'{" | safety camera ON" if args.safety else ""}{" (dry run)" if args.dry_run else ""}')
    if args.web:
        if 'voice' in scene:
            t = Transcriber(scene['voice'])
            if t.available():
                node.voice = t
            else:
                node.say(f"[voice] Voice input off: whisper.cpp not found at {scene['voice']['whisper_bin']}")
            sp = Speaker(scene['voice'])
            if sp.available():
                node.speaker = sp
        try:
            url = start_web_ui(node, args.host, args.port, https=args.https)
        except OSError as e:
            node.say(f'[error] Cannot start the web UI on {args.host}:{args.port}: {e}')
            raise SystemExit(1)
        node.say(f'[llm_commander] Web UI: {url}')

    try:
        if args.command:
            time.sleep(2.0)
            node.commands.put(' '.join(args.command))
            time.sleep(0.2)
            node.commands.join()
        elif sys.stdin.isatty():
            repl(node)
        else:
            print(f"No terminal attached; listening on /llm_command{' and the web UI' if args.web else ''}. "
                  'Ctrl+C to stop.')
            while rclpy.ok():
                time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        stop_bridge(bridge)
        # Stop spinning before tearing down, otherwise rclpy can segfault on exit.
        executor.shutdown(timeout_sec=2.0)
        spin_thread.join(timeout=2.0)
        rclpy.try_shutdown()
        # rclpy may still segfault while garbage-collecting action clients; all work is done here.
        sys.stdout.flush()
        os._exit(0)


if __name__ == '__main__':
    main()
