"""ROS 2 node: natural-language commands -> local LLM plan -> robot pick-and-place.

Commands come from the interactive prompt (when run in a terminal) or the /llm_command topic
(std_msgs/String). Progress and results are printed and published on /llm_response.

Usage:  ros2 run llm_commander commander [--config scene.yaml] [--model NAME] [--url URL] [--dry-run]
"""
import argparse
import os
import queue
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
from llm_commander.scene import destinations, load_scene

POSE_TYPE = 'objectpose_msgs/msg/ObjectPose'
POSE_MAX_AGE_S = 2.0


class Commander(Node):
    def __init__(self, scene, planner, dry_run=False):
        super().__init__('llm_commander')
        self.scene, self.planner, self.dry_run = scene, planner, dry_run
        self.dests = destinations(scene)
        self.cb = ReentrantCallbackGroup()
        self.poses, self.pose_stamp, self._pose_subs = {}, {}, {}
        self.lock = threading.Lock()
        self.commands = queue.Queue()
        self.busy = threading.Event()

        self.pub = self.create_publisher(String, '/llm_response', 10)
        self.create_subscription(String, '/llm_command', lambda m: self.commands.put(m.data), 10,
                                 callback_group=self.cb)
        self.create_timer(1.0, self._discover_objects, callback_group=self.cb)
        self.skills = RobotSkills(self, scene, self.dests, self.get_pose, self.all_poses, self.say)

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
    def say(self, text):
        print(text, flush=True)
        self.pub.publish(String(data=text))

    def run_command(self, command):
        objects = self.live_objects()
        self.say(f'[command] {command}')
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
                if a == 'home':
                    self.skills.home()
                    result = 'robot is at home.'
                elif a == 'pick':
                    result = self.skills.pick(step['object'])
                elif a == 'place':
                    result = self.skills.place(step['destination'])
                else:
                    result = self.skills.pick_and_place(step['object'], step['destination'])
                self.say('  done: ' + result)
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
                self.run_command(cmd.strip())
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
    parser.add_argument('command', nargs='*', help='run a single command and exit')
    args, _ = parser.parse_known_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    scene = load_scene(args.config)
    llm_cfg = scene['llm']
    planner = LLMPlanner(args.url or llm_cfg['url'], args.model or llm_cfg['model'], llm_cfg.get('timeout_s', 120))

    rclpy.init()
    node = Commander(scene, planner, dry_run=args.dry_run)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    threading.Thread(target=node.worker, daemon=True).start()
    node.say(f'[llm_commander] model={planner.model}{" (dry run)" if args.dry_run else ""}')

    try:
        if args.command:
            time.sleep(2.0)
            node.commands.put(' '.join(args.command))
            time.sleep(0.2)
            node.commands.join()
        elif sys.stdin.isatty():
            repl(node)
        else:
            print('No terminal attached; listening on /llm_command. Ctrl+C to stop.')
            while rclpy.ok():
                time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop spinning before tearing down, otherwise rclpy can segfault on exit.
        executor.shutdown(timeout_sec=2.0)
        spin_thread.join(timeout=2.0)
        rclpy.try_shutdown()
        # rclpy may still segfault while garbage-collecting action clients; all work is done here.
        sys.stdout.flush()
        os._exit(0)


if __name__ == '__main__':
    main()
