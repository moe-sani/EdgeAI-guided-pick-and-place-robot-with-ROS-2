"""Publish Gazebo model poses as /object_pose/<Name> (geometry_msgs/PoseStamped, frame "world").

Subscribes to the world's pose/info directly with gz-transport (the ROS bridge drops the model names), keeps
the top-level models that are scene objects (cubes, trays, people) and republishes each on its own topic.
pose/info lists links, visuals and lights too, by bare name. Gazebo's scene broadcaster puts the top-level
models first, in entity-id order, then their links (which start again at a lower id), so the models are the
entries before the id first drops. (The world's scene/info service would say the same but stops answering under load.)
This is the same interface the camera-based detector provides on the real robot.
"""
import argparse
import sys
import threading

import rclpy
from geometry_msgs.msg import PoseStamped
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from rclpy.node import Node

# Models that are part of the cell, not objects.
IGNORE = {'ground', 'table', 'so101'}
RATE_HZ = 20.0


class SimObjectPoses(Node):
    def __init__(self, world):
        super().__init__('sim_object_poses')
        self.pubs, self.latest, self.lock = {}, {}, threading.Lock()
        self.gz = GzNode()
        topic = f'/world/{world}/pose/info'
        if not self.gz.subscribe(Pose_V, topic, self.on_poses):
            raise RuntimeError(f'cannot subscribe to {topic}')
        self.create_timer(1.0 / RATE_HZ, self.publish)

    def on_poses(self, msg):
        models, last_id = {}, -1
        for p in msg.pose:
            if p.id < last_id:  # past the models, into their links
                break
            last_id = p.id
            if p.name not in IGNORE:
                models[p.name] = p
        with self.lock:
            self.latest = models

    def publish(self):
        with self.lock:
            poses = dict(self.latest)
        stamp = self.get_clock().now().to_msg()
        for name, p in poses.items():
            pub = self.pubs.get(name)
            if pub is None:
                pub = self.pubs[name] = self.create_publisher(PoseStamped, f'/object_pose/{name}', 10)
            m = PoseStamped()
            m.header.stamp, m.header.frame_id = stamp, 'world'
            m.pose.position.x, m.pose.position.y, m.pose.position.z = p.position.x, p.position.y, p.position.z
            o = p.orientation
            m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = o.x, o.y, o.z, o.w
            pub.publish(m)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--world', default='so101_cell')
    args, _ = parser.parse_known_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    rclpy.init()
    node = SimObjectPoses(args.world)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
