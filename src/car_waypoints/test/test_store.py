"""Waypoint file format and yaw conversion."""
import math

import pytest

from car_waypoints.store import (
    Waypoint, load_waypoints, quaternion_to_yaw, save_waypoints, yaw_to_quaternion)


def test_quarter_turn_quaternion():
    x, y, z, w = yaw_to_quaternion(math.pi / 2)
    assert (x, y) == (0.0, 0.0)
    assert z == pytest.approx(math.sqrt(0.5))
    assert w == pytest.approx(math.sqrt(0.5))


@pytest.mark.parametrize('yaw', [0.0, 1.0, -2.5, 3.1])
def test_yaw_round_trips_through_quaternion(yaw):
    assert quaternion_to_yaw(*yaw_to_quaternion(yaw)) == pytest.approx(yaw)


def test_missing_file_has_no_waypoints(tmp_path):
    assert load_waypoints(tmp_path / 'none.yaml') == {}


def test_save_then_load_keeps_order_and_rounds_to_mm(tmp_path):
    path = tmp_path / 'w.yaml'
    save_waypoints(path, {
        'start': Waypoint(0.0, 0.0, 0.0),
        't1': Waypoint(1.23456, -0.5, 1.5707963),
        'prep': Waypoint(4.2, -1.5, math.pi),
    })
    loaded = load_waypoints(path)
    assert list(loaded) == ['start', 't1', 'prep']
    assert loaded['t1'] == Waypoint(1.235, -0.5, 1.5708)


def test_save_creates_missing_directories(tmp_path):
    path = tmp_path / 'a' / 'b' / 'w.yaml'
    save_waypoints(path, {'t1': Waypoint(1.0, 2.0, 0.0)})
    assert load_waypoints(path) == {'t1': Waypoint(1.0, 2.0, 0.0)}


def test_entry_missing_yaw_names_the_waypoint(tmp_path):
    path = tmp_path / 'w.yaml'
    path.write_text('waypoints:\n  t2: {x: 1.0, y: 2.0}\n')
    with pytest.raises(ValueError, match='t2'):
        load_waypoints(path)
