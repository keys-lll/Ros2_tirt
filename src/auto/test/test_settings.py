"""mission.yaml: defaults, overrides and typo protection."""
import pytest

from auto.settings import load_settings, missing_points
from car_waypoints.store import DEFAULT_FILE, Waypoint


def write(tmp_path, text):
    path = tmp_path / 'mission.yaml'
    path.write_text(text)
    return path


def test_missing_file_uses_defaults(tmp_path):
    s = load_settings(tmp_path / 'none.yaml')
    assert s.waypoint_file == DEFAULT_FILE
    assert s.stop_seconds == 3.0
    assert (s.start, s.prep) == ('start', 'prep')
    assert s.tables == ('t1', 't2', 't3', 't4', 't5')


def test_file_overrides_only_what_it_names(tmp_path):
    s = load_settings(write(tmp_path, 'stop_seconds: 4.5\n'))
    assert s.stop_seconds == 4.5
    assert s.tables == ('t1', 't2', 't3', 't4', 't5')


def test_points_can_be_renamed(tmp_path):
    s = load_settings(write(tmp_path, (
        'points:\n  start: door\n  prep: kitchen\n  tables: [a, b, c, d, e]\n')))
    assert (s.start, s.prep, s.tables) == ('door', 'kitchen', ('a', 'b', 'c', 'd', 'e'))


def test_waypoint_file_can_be_set(tmp_path):
    s = load_settings(write(tmp_path, 'waypoint_file: /tmp/w.yaml\n'))
    assert s.waypoint_file == '/tmp/w.yaml'


def test_needs_exactly_five_tables(tmp_path):
    with pytest.raises(ValueError, match='tables'):
        load_settings(write(tmp_path, 'points:\n  tables: [t1, t2, t3]\n'))


@pytest.mark.parametrize('value', ['-1', 'abc', '.nan'])
def test_stop_seconds_must_be_a_non_negative_number(tmp_path, value):
    with pytest.raises(ValueError, match='stop_seconds'):
        load_settings(write(tmp_path, f'stop_seconds: {value}\n'))


def test_misspelled_key_is_reported_not_ignored(tmp_path):
    with pytest.raises(ValueError, match='stop_second'):
        load_settings(write(tmp_path, 'stop_second: 5\n'))


def test_two_points_cannot_share_a_name(tmp_path):
    with pytest.raises(ValueError, match='t2'):
        load_settings(write(tmp_path, 'points:\n  tables: [t1, t2, t2, t4, t5]\n'))


def test_missing_points_are_listed_in_mission_order(tmp_path):
    s = load_settings(tmp_path / 'none.yaml')
    have = {'t1': Waypoint(1.0, 0.0, 0.0), 'start': Waypoint(0.0, 0.0, 0.0),
            't4': Waypoint(4.0, 0.0, 0.0)}
    assert missing_points(s, have) == ['prep', 't2', 't3', 't5']


def test_no_missing_points_when_all_seven_exist(tmp_path):
    s = load_settings(tmp_path / 'none.yaml')
    have = {name: Waypoint(0.0, 0.0, 0.0) for name in
            ('start', 'prep', 't1', 't2', 't3', 't4', 't5')}
    assert missing_points(s, have) == []


def test_shipped_mission_yaml_loads_and_matches_the_defaults(tmp_path):
    from pathlib import Path
    shipped = Path(__file__).parents[1] / 'config' / 'mission.yaml'
    assert shipped.exists()
    assert load_settings(shipped) == load_settings(tmp_path / 'none.yaml')
