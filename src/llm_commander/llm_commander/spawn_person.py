"""Put a simulated person into the robot cell, or remove it again.

Uses Gazebo Fuel's 'Rescue Randy Sitting' (child-sized), sitting on the work table in the robot's path.
The person has a physics body, so if the robot moves while they are there, it bumps into them and
pushes or knocks them over.

Usage:
  ros2 run llm_commander spawn_person                       # at person_spot from scene.yaml
  ros2 run llm_commander spawn_person --x 0.6 --y 0.3 --yaw 90
  ros2 run llm_commander spawn_person --remove

Re-running it puts the person back (it is removed and spawned again).
"""
import argparse
import math

import rclpy

from llm_commander.scene import load_scene, person_sdf, yaw_quaternion
from llm_commander.spawn_scene import Spawner, gz_models, gz_remove


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--config', default=None)
    parser.add_argument('--name', default='Person')
    parser.add_argument('--x', type=float, default=None, help='where the person sits (torso), world x [m]')
    parser.add_argument('--y', type=float, default=None, help='where the person sits (torso), world y [m]')
    parser.add_argument('--yaw', type=float, default=None, help='heading in degrees (default: person_spot.yaw_deg)')
    parser.add_argument('--remove', action='store_true')
    args, _ = parser.parse_known_args()

    scene = load_scene(args.config)
    world = scene['world']
    existing = gz_models()
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
        # 2 mm above the table so the body doesn't start in contact and get kicked.
        node.spawn(args.name, person_sdf(args.name), x, y, scene['table_z'] + 0.002, q=yaw_quaternion(yaw))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
