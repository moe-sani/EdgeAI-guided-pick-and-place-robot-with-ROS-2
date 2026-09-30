"""Scene configuration loading and tray SDF generation."""
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
    """Return {name: dict(x, y, half_x, half_y, release_z_center)}.

    ``release_z_center`` is the height of the *cube centre* at which the gripper opens.
    """
    size = scene['tray_size']
    table_z = scene['table_z']
    dests = {}
    for t in scene['trays']:
        half = size['inner'] / 2.0
        floor_top = table_z + size['floor_thickness']
        dests[t['name']] = dict(
            x=t['x'], y=t['y'], half_x=half, half_y=half, kind='tray',
            release_z_center=floor_top + cube_half_size() + scene['motion']['tray_release_clearance'])
    for d in scene['fixed_destinations']:
        dests[d['name']] = dict(
            x=d['x'], y=d['y'], half_x=d['half_x'], half_y=d['half_y'], kind='fixed',
            release_z_center=d['place_z'] - scene['motion']['grasp_z_offset'])
    return dests


def tray_sdf(name, rgb, size):
    """Static, open-top tray made of a floor plate and four walls; origin at the floor's bottom centre."""
    inner, h = size['inner'], size['wall_height']
    t, ft = size['wall_thickness'], size['floor_thickness']
    outer = inner + 2 * t
    r, g, b = rgb
    material = (f'<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>'
                f'<specular>0.1 0.1 0.1 1</specular></material>')

    def box(part, x, y, z, sx, sy, sz):
        geom = f'<geometry><box><size>{sx} {sy} {sz}</size></box></geometry>'
        return (f'<collision name="{part}_c"><pose>{x} {y} {z} 0 0 0</pose>{geom}</collision>'
                f'<visual name="{part}_v"><pose>{x} {y} {z} 0 0 0</pose>{geom}{material}</visual>')

    wall_z = h / 2.0
    off = inner / 2.0 + t / 2.0
    parts = [
        box('floor', 0, 0, ft / 2.0, outer, outer, ft),
        box('wall_px', off, 0, wall_z, t, outer, h),
        box('wall_nx', -off, 0, wall_z, t, outer, h),
        box('wall_py', 0, off, wall_z, inner, t, h),
        box('wall_ny', 0, -off, wall_z, inner, t, h),
    ]
    return (f'<?xml version="1.0"?><sdf version="1.8"><model name="{name}"><static>true</static>'
            f'<link name="{name}_link">{"".join(parts)}</link></model></sdf>')
