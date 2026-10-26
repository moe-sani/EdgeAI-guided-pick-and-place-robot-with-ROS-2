"""Spawn the trays, cubes and safety camera defined in scene.yaml into the running Gazebo world (sim only).

Usage:  ros2 run llm_commander spawn_scene [--config path/to/scene.yaml] [--only trays|cubes|camera] [--reset]

--reset moves cubes that already exist back to their start positions (and spawns anything missing).
"""
import argparse
import shutil
import subprocess
import time

from llm_commander.scene import camera_sdf, cube_half_size, cube_sdf, load_scene, look_at_quaternion, tray_sdf


class Spawner:
    """Creates models through the world's gz-transport create service (no ROS bridge needed)."""

    def __init__(self, world):
        from gz.transport13 import Node as GzNode  # only in simulation installs
        self.gz, self.service = GzNode(), f'/world/{world}/create'
        self.log = print
        while self.service not in self.gz.service_list():
            self.log(f'Waiting for {self.service} (is the simulation running?) ...')
            time.sleep(1.0)

    def spawn(self, name, sdf, x, y, z, q=(0.0, 0.0, 0.0, 1.0)):
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.entity_factory_pb2 import EntityFactory
        req = EntityFactory(sdf=sdf, name=name, allow_renaming=False, relative_to='world')
        req.pose.position.x, req.pose.position.y, req.pose.position.z = float(x), float(y), float(z)
        o = req.pose.orientation
        o.x, o.y, o.z, o.w = (float(v) for v in q)
        ok, rep = self.gz.request(self.service, req, EntityFactory, Boolean, 10000)
        ok = ok and rep.data
        self.log(f"{'Spawned' if ok else 'FAILED to spawn'} {name} at ({x:.3f}, {y:.3f})")
        return ok


def gz_bin():
    return shutil.which('gz') or '/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz'


def gz_models():
    """Names of models currently in the simulation (Gazebo's create silently ignores duplicate names)."""
    out = subprocess.run([gz_bin(), 'model', '--list'], capture_output=True, text=True, timeout=15)
    return {line.strip()[2:] for line in out.stdout.splitlines() if line.strip().startswith('- ')}


def gz_remove(world, name):
    out = subprocess.run([gz_bin(), 'service', '-s', f'/world/{world}/remove', '--reqtype', 'gz.msgs.Entity',
                          '--reptype', 'gz.msgs.Boolean', '--timeout', '3000',
                          '--req', f'name: "{name}" type: MODEL'], capture_output=True, text=True)
    return 'data: true' in out.stdout


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
    parser.add_argument('--only', choices=['trays', 'cubes', 'camera'], default=None)
    parser.add_argument('--reset', action='store_true', help='move existing cubes back to their start poses')
    args, _ = parser.parse_known_args()

    scene = load_scene(args.config)
    node = Spawner(scene['world'])
    log = node.log
    existing = gz_models()
    if args.only in (None, 'trays'):
        for t in scene['trays']:
            if t['name'] in existing:
                log(f"{t['name']} already present")
                continue
            node.spawn(t['name'], tray_sdf(t['name'], t['rgb'], scene['tray_size']),
                       t['x'], t['y'], scene['table_z'])
    if args.only in (None, 'cubes'):
        for c in scene['cubes']:
            # Start 2 mm above the table so the cube settles flat.
            z = scene['table_z'] + cube_half_size() + 0.002
            if c['name'] in existing:
                if args.reset and gz_set_pose(scene['world'], c['name'], c['x'], c['y'], z):
                    log(f"Reset {c['name']} to ({c['x']:.3f}, {c['y']:.3f})")
                else:
                    log(f"{c['name']} already present" + ('' if args.reset else ' (use --reset to move it back)'))
                continue
            node.spawn(c['name'], cube_sdf(c['name'], c['rgb']), c['x'], c['y'], z)
    if args.only in (None, 'camera') and 'camera' in scene:
        cam = scene['camera']
        if cam['name'] in existing:
            log(f"{cam['name']} already present")
        else:
            node.spawn(cam['name'], camera_sdf(cam), *cam['position'],
                       q=look_at_quaternion(cam['position'], cam['look_at']))


if __name__ == '__main__':
    main()
