"""Mission settings from mission.yaml; every key is optional."""
import math
import os
from pathlib import Path
from typing import NamedTuple

import yaml

from car_waypoints.store import DEFAULT_FILE

# 放在 src 底下，改完不必重新 colcon build
DEFAULT_CONFIG = str(Path(os.environ.get('TIRT_WORKSPACE', '~/mecanum_ros2')).expanduser() / 'src/auto/config/mission.yaml')

TABLE_COUNT = 5
KEYS = ('waypoint_file', 'stop_seconds', 'points')
POINT_KEYS = ('start', 'prep', 'tables')


class Settings(NamedTuple):
    waypoint_file: str
    stop_seconds: float
    start: str
    prep: str
    tables: tuple


def load_settings(path):
    """Read mission.yaml; a missing file gives the defaults."""
    path = Path(path).expanduser()
    data = yaml.safe_load(path.read_text()) or {} if path.exists() else {}
    points = data.get('points') or {}
    for name, allowed in ((data, KEYS), (points, POINT_KEYS)):
        for key in name:
            if key not in allowed:
                raise ValueError(f'{path}: 不認得「{key}」，可用的有 {", ".join(allowed)}')
    stop_seconds = data.get('stop_seconds', 3.0)
    if (isinstance(stop_seconds, bool) or not isinstance(stop_seconds, (int, float))
            or not math.isfinite(stop_seconds) or stop_seconds < 0):
        raise ValueError(f'{path}: stop_seconds 要是 0 或正數（秒），收到 {stop_seconds!r}')
    tables = tuple(points.get('tables', [f't{i}' for i in range(1, TABLE_COUNT + 1)]))
    if len(tables) != TABLE_COUNT:
        raise ValueError(f'{path}: tables 要剛好 {TABLE_COUNT} 個名稱（1~5 號桌），收到 {len(tables)} 個')
    settings = Settings(
        waypoint_file=str(data.get('waypoint_file', DEFAULT_FILE)),
        stop_seconds=float(stop_seconds),
        start=str(points.get('start', 'start')),
        prep=str(points.get('prep', 'prep')),
        tables=tuple(str(t) for t in tables))
    names = point_names(settings)
    for name in names:
        if names.count(name) > 1:
            raise ValueError(f'{path}: 有兩個點都叫 {name}，每個點要用不同名稱')
    return settings


def point_names(settings):
    """Return the seven names in mission order: start, prep, tables 1-5."""
    return [settings.start, settings.prep, *settings.tables]


def missing_points(settings, waypoints):
    """Names the mission needs that are not in the waypoint file."""
    return [name for name in point_names(settings) if name not in waypoints]
