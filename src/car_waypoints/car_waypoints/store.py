"""Named waypoints saved as YAML: {name: {x, y, yaw}} in the map frame."""
import math
import os
from pathlib import Path
from typing import NamedTuple

import yaml

# 放在 src 底下，重新 colcon build 不會蓋掉，也能直接 git commit
DEFAULT_FILE = str(Path(os.environ.get('TIRT_WORKSPACE', '~/mecanum_ros2')).expanduser() / 'src/car_waypoints/config/waypoints.yaml')


class Waypoint(NamedTuple):
    x: float
    y: float
    yaw: float


def yaw_to_quaternion(yaw):
    """Return (x, y, z, w) for a rotation about z."""
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


def quaternion_to_yaw(x, y, z, w):
    """Return the z rotation of a quaternion in radians."""
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def load_waypoints(path):
    """Read waypoints in file order; a missing file has none."""
    path = Path(path).expanduser()
    if not path.exists():
        return {}
    data = yaml.safe_load(path.read_text()) or {}
    waypoints = {}
    for name, entry in (data.get('waypoints') or {}).items():
        try:
            waypoints[name] = Waypoint(
                float(entry['x']), float(entry['y']), float(entry['yaw']))
        except (KeyError, TypeError, ValueError) as e:
            raise ValueError(f'{path}: waypoint {name} needs numeric x, y, yaw ({e})')
    return waypoints


def save_waypoints(path, waypoints):
    """Write waypoints with x/y to the millimetre and yaw to 1e-4 rad."""
    path = Path(path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    data = {'waypoints': {
        name: {'x': round(float(wp.x), 3), 'y': round(float(wp.y), 3),
               'yaw': round(float(wp.yaw), 4)}
        for name, wp in waypoints.items()}}
    path.write_text(yaml.safe_dump(data, sort_keys=False, default_flow_style=None))
