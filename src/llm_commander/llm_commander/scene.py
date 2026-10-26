"""Scene configuration loading and SDF generation (cubes, trays, safety camera, test person)."""
import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory


def default_config_path():
    return os.path.join(get_package_share_directory('llm_commander'), 'config', 'scene.yaml')


def load_scene(path=None):
    with open(path or default_config_path()) as f:
        return yaml.safe_load(f)


def cube_half_size():
    return 0.015


def destinations(scene):
    """Return {name: dict(x, y, half_x, half_y, floor_z, release_z)}.

    ``release_z`` is the grasp-point height at which the gripper opens (the cube hangs centred on it).
    """
    size = scene['tray_size']
    clearance = scene['motion']['tray_release_clearance']
    dests = {}
    for t in scene['trays']:
        half = size['inner'] / 2.0
        floor_z = scene['table_z'] + size['floor_thickness']
        dests[t['name']] = dict(x=t['x'], y=t['y'], half_x=half, half_y=half, kind='tray', floor_z=floor_z,
                                release_z=floor_z + clearance + cube_half_size())
    for d in scene.get('fixed_destinations') or []:
        floor_z = d.get('z', scene['table_z'])
        dests[d['name']] = dict(x=d['x'], y=d['y'], half_x=d['half_x'], half_y=d['half_y'], kind='fixed',
                                floor_z=floor_z, release_z=floor_z + clearance + cube_half_size())
    return dests


def look_at_quaternion(pos, target):
    """(x, y, z, w) orienting a Gazebo camera (which looks along +X) from ``pos`` towards ``target``."""
    dx, dy, dz = (t - p for p, t in zip(pos, target))
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(-dz, math.hypot(dx, dy))  # positive pitch tilts +X downwards
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    return (-sp * sy, sp * cy, cp * sy, cp * cy)


def yaw_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def camera_sdf(cam):
    """Static camera model; the world already loads the gz Sensors system, so it renders once spawned."""
    return (f'<?xml version="1.0"?><sdf version="1.8"><model name="{cam["name"]}"><static>true</static>'
            f'<link name="link">'
            f'<visual name="body"><geometry><box><size>0.04 0.08 0.04</size></box></geometry>'
            f'<material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material></visual>'
            f'<sensor name="{cam["name"]}_sensor" type="camera"><always_on>1</always_on>'
            f'<update_rate>{cam["rate"]}</update_rate><topic>{cam["topic"]}</topic>'
            f'<camera><horizontal_fov>{cam["hfov"]}</horizontal_fov>'
            f'<image><width>{cam["width"]}</width><height>{cam["height"]}</height></image>'
            f'<clip><near>0.05</near><far>30</far></clip></camera></sensor>'
            f'</link></model></sdf>')


PERSON_MESH = 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/Rescue Randy Sitting/8/files/meshes/rescue_randy.dae'
# Mesh at scale 1 (m): x -0.473..0.274, y -0.714..0.304 (legs point to -y), z 0.618..1.348 (sitting).
# The model origin is put under the torso (where the person sits), so the collision body is centred on it.
PERSON_TORSO = (-0.025, 0.115)
PERSON_BOTTOM_Z = 0.618
PERSON_SCALE = 0.25  # ~0.18 m tall sitting: a figurine-sized person that fits in the SO-101's work area


def person_sdf(name, scale=PERSON_SCALE, mass=0.3, body=(0.06, 0.06, 0.17)):
    """Sitting person (Gazebo Fuel 'Rescue Randy Sitting') that the robot can bump and knock over.

    Physics uses one box around the torso and head (a full mesh collision on a moving body is slow and
    unstable); the legs are visual only, so the person can sit over cubes without shoving them.
    """
    ox, oy, oz = (-PERSON_TORSO[0] * scale, -PERSON_TORSO[1] * scale, -PERSON_BOTTOM_Z * scale)
    bx, by, bz = body
    ixx = mass / 12 * (by ** 2 + bz ** 2)
    iyy = mass / 12 * (bx ** 2 + bz ** 2)
    izz = mass / 12 * (bx ** 2 + by ** 2)
    return (f'<?xml version="1.0"?><sdf version="1.8"><model name="{name}"><static>false</static>'
            f'<link name="link">'
            f'<inertial><pose>0 0 {bz / 2:.3f} 0 0 0</pose><mass>{mass}</mass>'
            f'<inertia><ixx>{ixx:.5f}</ixx><iyy>{iyy:.5f}</iyy><izz>{izz:.5f}</izz>'
            f'<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>'
            f'<collision name="body"><pose>0 0 {bz / 2:.3f} 0 0 0</pose>'
            f'<geometry><box><size>{bx} {by} {bz}</size></box></geometry></collision>'
            f'<visual name="visual"><pose>{ox:.4f} {oy:.4f} {oz:.4f} 0 0 0</pose>'
            f'<geometry><mesh><uri>{PERSON_MESH}</uri><scale>{scale} {scale} {scale}</scale></mesh></geometry>'
            f'</visual></link></model></sdf>')


def tray_sdf(name, rgb, size):
    """Static, open-top tray made of a floor plate and four walls; origin at the floor's bottom centre.

    Only the walls are coloured: on a light floor a cube of the tray's own colour still stands out (for the
    overhead camera's colour detection, and the VLM).
    """
    inner, h = size['inner'], size['wall_height']
    t, ft = size['wall_thickness'], size['floor_thickness']
    outer = inner + 2 * t
    r, g, b = rgb

    def material(r, g, b):
        return (f'<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>'
                f'<specular>0.1 0.1 0.1 1</specular></material>')

    def box(part, x, y, z, sx, sy, sz, colour=(r, g, b)):
        geom = f'<geometry><box><size>{sx} {sy} {sz}</size></box></geometry>'
        return (f'<collision name="{part}_c"><pose>{x} {y} {z} 0 0 0</pose>{geom}</collision>'
                f'<visual name="{part}_v"><pose>{x} {y} {z} 0 0 0</pose>{geom}{material(*colour)}</visual>')

    wall_z = h / 2.0
    off = inner / 2.0 + t / 2.0
    parts = [
        box('floor', 0, 0, ft / 2.0, outer, outer, ft, colour=(0.75, 0.75, 0.72)),
        box('wall_px', off, 0, wall_z, t, outer, h),
        box('wall_nx', -off, 0, wall_z, t, outer, h),
        box('wall_py', 0, off, wall_z, inner, t, h),
        box('wall_ny', 0, -off, wall_z, inner, t, h),
    ]
    return (f'<?xml version="1.0"?><sdf version="1.8"><model name="{name}"><static>true</static>'
            f'<link name="link">{"".join(parts)}</link></model></sdf>')


def cube_sdf(name, rgb, size=None, mass=0.02):
    """Small dynamic cube; grippy enough for the SO-101's jaws to hold. Link is named "link" (see so101_cell)."""
    s = size or 2 * cube_half_size()
    r, g, b = rgb
    inertia = mass * s * s / 6.0
    geom = f'<geometry><box><size>{s} {s} {s}</size></box></geometry>'
    return (f'<?xml version="1.0"?><sdf version="1.8"><model name="{name}"><link name="link">'
            f'<inertial><mass>{mass}</mass><inertia><ixx>{inertia:.3e}</ixx><iyy>{inertia:.3e}</iyy>'
            f'<izz>{inertia:.3e}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>'
            f'<collision name="c">{geom}<surface><friction><ode><mu>1.5</mu><mu2>1.5</mu2></ode></friction>'
            f'</surface></collision>'
            f'<visual name="v">{geom}<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>'
            f'</material></visual></link></model></sdf>')
