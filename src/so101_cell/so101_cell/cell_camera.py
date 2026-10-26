"""Overhead USB camera for the real cell: safety-camera image, table calibration and cube detection.

  ros2 run so101_cell cell_camera [--device 0] [--table-calibration ~/.config/so101_cell/table.yaml]

Publishes the raw frames on /safety_camera/image (what the VLM safety check looks at) and, once the camera
is calibrated, the cubes as /object_pose/<Name> (geometry_msgs/PoseStamped, frame "world"), the same
interface Gazebo provides in simulation.

Calibration (once, and again whenever the camera or the table moves): print four ArUco markers
(4x4_50, ids 0-3; `ros2 run so101_cell cell_camera --print-markers` writes a page), tape them flat at the spots listed in
`markers` in scene.yaml, and run `ros2 run so101_cell cell_camera --calibrate`. It finds them, computes
the image -> table homography and saves it. The markers can be removed afterwards.

Cubes are found by colour (HSV ranges in scene.yaml `detection`). The camera sees the cube's top face,
so its image position is corrected for the camera's height to get the cube centre on the table.
"""
import argparse
import math
import os
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image

DEFAULT_TABLE_CALIBRATION = '~/.config/so101_cell/table.yaml'


def aruco_detect(gray):
    """(corners, ids) of 4x4_50 markers, with either OpenCV aruco API (4.6 has no ArucoDetector)."""
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    if hasattr(aruco, 'ArucoDetector'):
        corners, ids, _ = aruco.ArucoDetector(dictionary, aruco.DetectorParameters()).detectMarkers(gray)
    else:
        corners, ids, _ = aruco.detectMarkers(gray, dictionary, parameters=aruco.DetectorParameters_create())
    return corners, (ids.flatten().tolist() if ids is not None else [])


def marker_image(marker_id, pixels):
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    if hasattr(aruco, 'generateImageMarker'):
        return aruco.generateImageMarker(dictionary, marker_id, pixels)
    return aruco.drawMarker(dictionary, marker_id, pixels)


def table_homography(frame, markers):
    """Homography image px -> table (x, y) in m from the marker centres; ``markers`` = {id: (x, y)}."""
    corners, ids = aruco_detect(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
    found = {i: c.reshape(4, 2).mean(axis=0) for i, c in zip(ids, corners) if i in markers}
    if len(found) < 4:
        raise RuntimeError(f'found markers {sorted(found)}; need {sorted(markers)} (all four in view, flat, lit)')
    src = np.array([found[i] for i in sorted(found)], np.float32)
    dst = np.array([markers[i] for i in sorted(found)], np.float32)
    h, _ = cv2.findHomography(src, dst)
    return h


class CubeDetector:
    def __init__(self, cfg, homography, camera_xyz):
        self.cfg, self.h = cfg, np.asarray(homography, float)
        self.camera_xyz = camera_xyz  # camera position (world) for the parallax correction, or None
        self.kernel = np.ones((5, 5), np.uint8)

    def detect(self, frame):
        """{name: (x, y, yaw)} of the cubes seen in a BGR ``frame``."""
        hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0), cv2.COLOR_BGR2HSV)
        half = self.cfg['cube_size'] / 2.0
        out = {}
        for name, ranges in self.cfg['colors'].items():
            mask = np.zeros(hsv.shape[:2], np.uint8)
            for lo, hi in ranges:
                mask |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
            # RETR_LIST also finds a cube inside a same-coloured tray (as a hole in the tray's blob).
            contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            blobs = [c for c in contours if cv2.contourArea(c) >= self.cfg['min_area_px'] and self._cube_sized(c)]
            if not blobs:
                continue
            blob = max(blobs, key=cv2.contourArea)
            (u, v), _, angle = cv2.minAreaRect(blob)
            x, y = self._to_table(u, v)
            # The top face is seen, not the base: move the point towards the camera's foot (similar triangles).
            if self.camera_xyz is not None:
                cx, cy, cz = self.camera_xyz
                s = (2 * half) / cz
                x, y = x + (cx - x) * s, y + (cy - y) * s
            # Edge direction in the image -> on the table; a cube looks the same every 90 degrees.
            a = math.radians(angle)
            x2, y2 = self._to_table(u + 20 * math.cos(a), v + 20 * math.sin(a))
            x1, y1 = self._to_table(u, v)
            yaw = (math.atan2(y2 - y1, x2 - x1) + math.pi / 4) % (math.pi / 2) - math.pi / 4
            out[name] = (x, y, yaw)
        return out

    def _cube_sized(self, contour):
        """Solid (a tray wall's outline is not), with a footprint on the table between half and 2.5x a cube's
        face (the side faces seen at an angle add to it)."""
        hull = cv2.contourArea(cv2.convexHull(contour))
        if hull <= 0 or cv2.contourArea(contour) / hull < 0.8:
            return False
        pts = [self._to_table(u, v) for u, v in contour.reshape(-1, 2)]
        area = abs(cv2.contourArea(np.float32(pts)))
        return 0.5 <= area / self.cfg['cube_size'] ** 2 <= 2.5

    def _to_table(self, u, v):
        p = self.h @ np.array([u, v, 1.0])
        return p[0] / p[2], p[1] / p[2]


class CellCamera(Node):
    def __init__(self, scene, device, table_calibration):
        super().__init__('cell_camera')
        cam_cfg = scene['camera']
        self.cap = cv2.VideoCapture(int(device) if str(device).isdigit() else device)
        if not self.cap.isOpened():
            raise RuntimeError(f'cannot open camera {device}')
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam_cfg['width'])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam_cfg['height'])
        self.table_z = scene['table_z']
        self.cube_half = scene['detection']['cube_size'] / 2.0
        self.img_pub = self.create_publisher(Image, cam_cfg['topic'], 1)
        self.pose_pubs = {}
        self.detector = None
        path = os.path.expanduser(table_calibration)
        if os.path.exists(path):
            with open(path) as f:
                cal = yaml.safe_load(f)
            self.detector = CubeDetector(scene['detection'], cal['homography'], cal.get('camera_xyz'))
            self.get_logger().info(f'table calibration loaded from {path}')
        else:
            self.get_logger().warn(f'no table calibration at {path}: images only, no cube poses. '
                                   'Run `ros2 run so101_cell cell_camera --calibrate`.')
        self.running = True
        self.thread = threading.Thread(target=self._loop, args=(1.0 / scene['camera']['rate'],), daemon=True)
        self.thread.start()

    def _loop(self, period):
        last_img = 0.0
        while self.running and rclpy.ok():
            ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.1)
                continue
            now = time.monotonic()
            stamp = self.get_clock().now().to_msg()
            if now - last_img >= period:
                last_img = now
                msg = Image(height=frame.shape[0], width=frame.shape[1], encoding='bgr8', is_bigendian=0,
                            step=frame.shape[1] * 3, data=frame.tobytes())
                msg.header.stamp, msg.header.frame_id = stamp, 'safety_camera'
                self.img_pub.publish(msg)
            if self.detector is None:
                continue
            for name, (x, y, yaw) in self.detector.detect(frame).items():
                pub = self.pose_pubs.get(name)
                if pub is None:
                    pub = self.pose_pubs[name] = self.create_publisher(PoseStamped, f'/object_pose/{name}', 10)
                m = PoseStamped()
                m.header.stamp, m.header.frame_id = stamp, 'world'
                m.pose.position.x, m.pose.position.y = float(x), float(y)
                m.pose.position.z = self.table_z + self.cube_half  # cubes only ever rest on the table here
                m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
                pub.publish(m)

    def shutdown(self):
        self.running = False
        self.thread.join(timeout=1.0)
        self.cap.release()


def calibrate(scene, device, path):
    cap = cv2.VideoCapture(int(device) if str(device).isdigit() else device)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, scene['camera']['width'])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, scene['camera']['height'])
    for _ in range(10):  # let auto exposure settle
        ok, frame = cap.read()
    cap.release()
    if not ok:
        sys.exit(f'cannot read from camera {device}')
    markers = {m['id']: (m['x'], m['y']) for m in scene['markers']}
    h = table_homography(frame, markers)
    cal = {'homography': h.tolist(), 'camera_xyz': scene['camera']['position']}
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as f:
        yaml.safe_dump(cal, f)
    # Check: where does each marker land?
    corners, ids = aruco_detect(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
    for i, c in zip(ids, corners):
        if i in markers:
            u, v = c.reshape(4, 2).mean(axis=0)
            p = h @ np.array([u, v, 1.0])
            print(f'  marker {i}: expected {markers[i]}, maps to ({p[0] / p[2]:.3f}, {p[1] / p[2]:.3f})')
    snap = os.path.splitext(path)[0] + '.jpg'
    cv2.aruco.drawDetectedMarkers(frame, corners)
    cv2.imwrite(snap, frame)
    print(f'Table calibration saved to {path} (camera view: {snap})')


def print_markers():
    """Write a printable page with the four table markers (5 cm each) to ./so101_markers.png."""
    dpi, size_cm = 150, 5.0
    px = int(size_cm / 2.54 * dpi)
    page = np.full((int(29.7 / 2.54 * dpi), int(21.0 / 2.54 * dpi)), 255, np.uint8)
    for k in range(4):
        r, c = divmod(k, 2)
        y0, x0 = 150 + r * (px + 250), 120 + c * (px + 200)
        page[y0:y0 + px, x0:x0 + px] = marker_image(k, px)
        cv2.putText(page, f'id {k}', (x0, y0 + px + 50), cv2.FONT_HERSHEY_SIMPLEX, 1.2, 0, 2)
    cv2.imwrite('so101_markers.png', page)
    print(f'Wrote so101_markers.png: print at 100% on A4 ({dpi} dpi), markers are {size_cm:.0f} cm wide.')


def _load_scene(path):
    if path is None:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('llm_commander'), 'config', 'scene.yaml')
    with open(path) as f:
        return yaml.safe_load(f)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--config', default=None, help='scene.yaml (default: llm_commander config)')
    parser.add_argument('--calibrate', action='store_true', help='find the ArUco markers and save the table calibration')
    parser.add_argument('--print-markers', action='store_true', help='write so101_markers.png to print')
    parser.add_argument('--device', default=None, help='camera index or path (default real_camera.device)')
    parser.add_argument('--table-calibration', default=DEFAULT_TABLE_CALIBRATION)
    args, _ = parser.parse_known_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    scene = _load_scene(args.config)
    device = args.device if args.device is not None else scene.get('real_camera', {}).get('device', 0)
    if args.print_markers:
        print_markers()
        return
    if args.calibrate:
        calibrate(scene, device, os.path.expanduser(args.table_calibration))
        return
    rclpy.init()
    node = CellCamera(scene, device, args.table_calibration)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
