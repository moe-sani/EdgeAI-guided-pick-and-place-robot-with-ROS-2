"""Camera + vision-language-model safety check for the robot cell.

The VLM is shown the latest frame from the in-cell safety camera and asked a yes/no question:
is there anything risky on the way of the robot? Anything other than a clear "no" (no frame, stale frame,
server error, unexpected answer) counts as *risky*.

Standalone use (for tuning / demo):
  ros2 run llm_commander safety_check [--repeat N]
"""
import argparse
import base64
import os
import re
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass

import cv2
import numpy as np
import rclpy
from sensor_msgs.msg import Image

from llm_commander.llm import LLMPlanner, PlanError
from llm_commander.scene import load_scene

# Qwen3-VL-4B answered "no" with a child in view when asked only about "anything risky";
# naming "a person" explicitly fixed that without false alarms on the normal scene.
SAFETY_PROMPT = "Is there a person, or anything else risky, on the way of the robot? reply with yes or no only"


@dataclass
class Verdict:
    risky: bool
    reason: str
    seconds: float


def to_bgr(msg):
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}.get(msg.encoding)
    if channels is None:
        raise ValueError(f'unsupported image encoding {msg.encoding!r}')
    img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * channels]
    img = img.reshape(msg.height, msg.width, channels) if channels > 1 else img.reshape(msg.height, msg.width)
    conv = {'rgb8': cv2.COLOR_RGB2BGR, 'rgba8': cv2.COLOR_RGBA2BGR, 'bgra8': cv2.COLOR_BGRA2BGR,
            'mono8': cv2.COLOR_GRAY2BGR}.get(msg.encoding)
    return cv2.cvtColor(img, conv) if conv is not None else img


def start_bridge(node, topic, wait_s=3.0):
    """Bridge the gz camera topic to ROS unless something already publishes it. Returns the Popen or None."""
    deadline = time.monotonic() + wait_s
    while time.monotonic() < deadline:
        if node.count_publishers(topic) > 0:
            return None
        time.sleep(0.2)
    proc = subprocess.Popen(
        ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', f'{topic}@sensor_msgs/msg/Image[gz.msgs.Image'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    node.get_logger().info(f'Started ros_gz_bridge for {topic} (pid {proc.pid})')
    return proc


def stop_bridge(proc):
    if proc is not None and proc.poll() is None:
        os.killpg(proc.pid, signal.SIGTERM)


class SafetyMonitor:
    def __init__(self, node, scene, client, callback_group=None):
        self.cfg, self.cam, self.client = scene['safety'], scene['camera'], client
        # The question asked with every frame. "yes" must mean risky; it can be changed at runtime.
        self.default_prompt = (self.cfg.get('prompt') or SAFETY_PROMPT).strip()
        self.prompt = self.default_prompt
        self._lock = threading.Lock()
        self._latest = None
        self._stamp = 0.0
        self.last_judged_jpeg = None  # JPEG bytes of the frame the VLM saw last
        node.create_subscription(Image, self.cam['topic'], self._on_image, 1, callback_group=callback_group)

    def _on_image(self, msg):
        with self._lock:
            self._latest, self._stamp = msg, time.monotonic()

    def latest_jpeg(self, width=None, quality=80):
        """(JPEG bytes, age in s) of the newest camera frame, or (None, None) if none has arrived yet."""
        with self._lock:
            msg, age = self._latest, time.monotonic() - self._stamp
        if msg is None:
            return None, None
        img = to_bgr(msg)
        if width and img.shape[1] > width:
            img = cv2.resize(img, (width, int(img.shape[0] * width / img.shape[1])), interpolation=cv2.INTER_AREA)
        ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, quality])
        return (jpg.tobytes() if ok else None), age

    def wait_for_frame(self, timeout_s=10.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            with self._lock:
                if self._latest is not None:
                    return True
            time.sleep(0.1)
        return False

    def _frame_b64(self):
        with self._lock:
            msg, age = self._latest, time.monotonic() - self._stamp
        if msg is None:
            raise RuntimeError(f"no image on {self.cam['topic']} (is the SafetyCamera spawned? run spawn_scene)")
        if age > self.cfg['max_image_age_s']:
            raise RuntimeError(f'camera image is stale ({age:.1f}s old)')
        img = to_bgr(msg)
        w = self.cfg['image_width']
        if img.shape[1] > w:
            img = cv2.resize(img, (w, int(img.shape[0] * w / img.shape[1])), interpolation=cv2.INTER_AREA)
        if self.cfg.get('save_frame'):
            cv2.imwrite(self.cfg['save_frame'], img)
        ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ok:
            raise RuntimeError('JPEG encoding failed')
        self.last_judged_jpeg = jpg.tobytes()
        return base64.b64encode(self.last_judged_jpeg).decode()

    def check(self):
        t0 = time.monotonic()
        try:
            b64 = self._frame_b64()
            raw = self.client.chat([{'role': 'user', 'content': [
                {'type': 'text', 'text': self.prompt},
                {'type': 'image_url', 'image_url': {'url': f'data:image/jpeg;base64,{b64}'}},
            ]}], max_tokens=5)
            answer = re.sub(r'<think>.*?</think>', '', raw, flags=re.S).strip().lower()
            word = re.match(r'[a-z]+', answer)
            word = word.group(0) if word else ''
            if word not in ('yes', 'no'):
                raise ValueError(f'expected yes/no, got {raw!r}')
            return Verdict(word == 'yes', f'VLM answered {word!r}', time.monotonic() - t0)
        except (PlanError, RuntimeError, ValueError) as e:
            return Verdict(True, f'safety check failed ({e}); treating as unsafe', time.monotonic() - t0)
        except Exception as e:  # e.g. JSON errors; never let a broken check mean "safe"
            return Verdict(True, f'safety check failed ({e!r}); treating as unsafe', time.monotonic() - t0)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--config', default=None)
    parser.add_argument('--model', default=None)
    parser.add_argument('--repeat', type=int, default=1)
    parser.add_argument('--prompt', default=None, help='try a different safety question (yes = risky)')
    args, _ = parser.parse_known_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    scene = load_scene(args.config)
    llm = scene['llm']
    client = LLMPlanner(llm['url'], args.model or llm['model'], llm.get('timeout_s', 120))
    rclpy.init()
    node = rclpy.create_node('llm_safety_check')
    monitor = SafetyMonitor(node, scene, client)
    if args.prompt:
        monitor.prompt = args.prompt
    print(f'Prompt: {monitor.prompt}')
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    bridge = start_bridge(node, scene['camera']['topic'])
    code = 0
    try:
        if not monitor.wait_for_frame():
            print(f"No image on {scene['camera']['topic']}. Is the sim running and the SafetyCamera spawned "
                  f"(ros2 run llm_commander spawn_scene --only camera)?")
            code = 1
        for i in range(args.repeat):
            v = monitor.check()
            print(f"[{i + 1}/{args.repeat}] {'RISKY' if v.risky else 'clear'} ({v.seconds:.1f}s): {v.reason}",
                  flush=True)
        if scene['safety'].get('save_frame'):
            print(f"Last judged frame: {scene['safety']['save_frame']}")
    finally:
        stop_bridge(bridge)
        executor.shutdown(timeout_sec=2.0)
        rclpy.try_shutdown()
    os._exit(code)


if __name__ == '__main__':
    main()
