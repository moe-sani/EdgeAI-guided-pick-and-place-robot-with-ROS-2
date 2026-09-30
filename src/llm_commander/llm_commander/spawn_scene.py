"""Spawn the trays and cubes defined in scene.yaml into the running Gazebo world.

Usage:  ros2 run llm_commander spawn_scene [--config path/to/scene.yaml] [--only trays|cubes] [--reset]

--reset moves cubes that already exist back to their start positions (and spawns anything missing).
"""
import argparse
import os
import shutil
import subprocess
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from ros_gz_interfaces.srv import SpawnEntity

from llm_commander.scene import cube_half_size, load_scene, tray_sdf


class Spawner(Node):
    def __init__(self, world):
        super().__init__('llm_scene_spawner')
        self.cli = self.create_client(SpawnEntity, f'/world/{world}/create')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for /world/{world}/create (is the simulation running?) ...')

    def spawn(self, name, sdf, x, y, z):
        req = SpawnEntity.Request()
        req.entity_factory.name = name
        req.entity_factory.allow_renaming = False
        req.entity_factory.relative_to = 'world'
        req.entity_factory.sdf = sdf
        req.entity_factory.pose.position.x = float(x)
        req.entity_factory.pose.position.y = float(y)
        req.entity_factory.pose.position.z = float(z)
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        res = fut.result()
        ok = res is not None and res.success
        self.get_logger().info(f"{'Spawned' if ok else 'FAILED to spawn'} {name} at ({x:.3f}, {y:.3f})")
        return ok


def cube_sdf(name):
    share = get_package_share_directory('irb120cranfield')
    with open(os.path.join(share, 'sdf', f'{name}.sdf')) as f:
        xml = f.read()
    mesh_uri = Path(os.path.join(share, 'meshes', 'objects', f'{name}.dae')).as_uri()
    return xml.replace('$(name)', name).replace('$(mesh_uri)', mesh_uri)


def gz_bin():
    return shutil.which('gz') or '/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz'


def gz_models():
    """Names of models currently in the simulation (Gazebo's create silently ignores duplicate names)."""
    out = subprocess.run([gz_bin(), 'model', '--list'], capture_output=True, text=True, timeout=15)
    return {line.strip()[2:] for line in out.stdout.splitlines() if line.strip().startswith('- ')}


def gz_set_pose(world, name, x, y, z):
    """Teleport a model via the Gazebo CLI (set_pose isn't bridged to ROS). Returns True on success."""
    req = f'name: "{name}" position {{x: {x} y: {y} z: {z}}} orientation {{w: 1}}'
    out = subprocess.run([gz_bin(), 'service', '-s', f'/world/{world}/set_pose', '--reqtype', 'gz.msgs.Pose',
                          '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', req],
                         capture_output=True, text=True)
    return 'data: true' in out.stdout


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=None)
    parser.add_argument('--only', choices=['trays', 'cubes'], default=None)
    parser.add_argument('--reset', action='store_true', help='move existing cubes back to their start poses')
    args, _ = parser.parse_known_args()

    scene = load_scene(args.config)
    rclpy.init()
    node = Spawner(scene['world'])
    log = node.get_logger().info
    existing = gz_models()
    try:
        if args.only in (None, 'trays'):
            for t in scene['trays']:
                if t['name'] in existing:
                    log(f"{t['name']} already present")
                    continue
                node.spawn(t['name'], tray_sdf(t['name'], t['rgb'], scene['tray_size']),
                           t['x'], t['y'], scene['table_z'])
        if args.only in (None, 'cubes'):
            for c in scene['cubes']:
                # Start 1 cm above the table so the cube settles flat.
                z = scene['table_z'] + cube_half_size() + 0.01
                if c['name'] in existing:
                    if args.reset and gz_set_pose(scene['world'], c['name'], c['x'], c['y'], z):
                        log(f"Reset {c['name']} to ({c['x']:.3f}, {c['y']:.3f})")
                    else:
                        log(f"{c['name']} already present" + ('' if args.reset else ' (use --reset to move it back)'))
                    continue
                node.spawn(c['name'], cube_sdf(c['name']), c['x'], c['y'], z)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
