"""Put a simulated person (default: a child) into the robot cell, or remove it again.

Uses Gazebo Fuel's 'Rescue Randy Sitting', sitting on the work table in front of the robot.

Usage:
  ros2 run llm_commander spawn_person                      # child-size, at person_spot from scene.yaml
  ros2 run llm_commander spawn_person --kind adult --x 0.6 --y 0.3 --yaw 90
  ros2 run llm_commander spawn_person --remove

Re-running it moves the person (it is removed and spawned again).
The person is visual-only (no collision): it exists to test the camera safety check.
"""
import argparse
import math

import rclpy

from llm_commander.scene import load_scene, person_sdf, yaw_quaternion
from llm_commander.spawn_scene import Spawner, gz_models, gz_remove

SCALES = {'child': 0.6, 'adult': 1.0}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--config', default=None)
    parser.add_argument('--kind', choices=list(SCALES), default='child',
                        help='sitting Rescue Randy: child = 0.6x (~0.45 m tall, 0.6 m long), adult = full size')
    parser.add_argument('--name', default='Person')
    parser.add_argument('--x', type=float, default=None)
    parser.add_argument('--y', type=float, default=None)
    parser.add_argument('--yaw', type=float, default=None, help='heading in degrees (default: person_spot.yaw_deg)')
    parser.add_argument('--remove', action='store_true')
    args, _ = parser.parse_known_args()

    scene = load_scene(args.config)
    world = scene['world']
    existing = gz_models()
    if args.remove or args.name in existing:
        if args.name in existing:
            print(f"Removed {args.name}" if gz_remove(world, args.name) else f"FAILED to remove {args.name}")
        if args.remove:
            return

    spot = scene['person_spot']
    x = spot['x'] if args.x is None else args.x
    y = spot['y'] if args.y is None else args.y
    yaw = math.radians(spot['yaw_deg'] if args.yaw is None else args.yaw)

    rclpy.init()
    node = Spawner(world)
    try:
        node.spawn(args.name, person_sdf(args.name, SCALES[args.kind]), x, y, scene['table_z'],
                   q=yaw_quaternion(yaw))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
