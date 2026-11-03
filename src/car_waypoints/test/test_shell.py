"""Terminal commands of wp_record and wp_go, without ROS."""
import pytest

from car_waypoints.shell import arrival_report, GoShell, RecordShell
from car_waypoints.store import Waypoint, load_waypoints, save_waypoints


@pytest.fixture
def path(tmp_path):
    return tmp_path / 'w.yaml'


def recorder(path, pose=Waypoint(1.0, 2.0, 0.5), changes=None):
    def get_pose():
        if isinstance(pose, Exception):
            raise pose
        return pose
    return RecordShell(path, get_pose, on_change=changes.append if changes is not None else None)


def test_bare_name_saves_current_pose(path):
    text, running = recorder(path).handle('t1')
    assert running
    assert 't1' in text
    assert load_waypoints(path) == {'t1': Waypoint(1.0, 2.0, 0.5)}


def test_saving_existing_name_overwrites_and_says_so(path):
    save_waypoints(path, {'t1': Waypoint(9.0, 9.0, 0.0)})
    text, _ = recorder(path).handle('t1')
    assert '覆寫' in text
    assert load_waypoints(path)['t1'] == Waypoint(1.0, 2.0, 0.5)


def test_del_removes_waypoint(path):
    save_waypoints(path, {'t1': Waypoint(1.0, 1.0, 0.0), 't2': Waypoint(2.0, 2.0, 0.0)})
    recorder(path).handle('del t1')
    assert list(load_waypoints(path)) == ['t2']


def test_del_unknown_name_changes_nothing(path):
    save_waypoints(path, {'t1': Waypoint(1.0, 1.0, 0.0)})
    text, running = recorder(path).handle('del t9')
    assert running
    assert 't9' in text
    assert list(load_waypoints(path)) == ['t1']


@pytest.mark.parametrize('line', ['1t', 't 1', 'go', 'del', 't-1'])
def test_invalid_or_reserved_names_are_not_saved(path, line):
    text, running = recorder(path).handle(line)
    assert running
    assert load_waypoints(path) == {}


def test_pose_lookup_failure_is_reported_and_nothing_saved(path):
    text, running = recorder(path, pose=LookupError('no map frame')).handle('t1')
    assert running
    assert 'no map frame' in text
    assert load_waypoints(path) == {}


def test_changes_are_announced_after_save_and_delete(path):
    changes = []
    shell = recorder(path, changes=changes)
    shell.handle('t1')
    shell.handle('del t1')
    assert changes == [{'t1': Waypoint(1.0, 2.0, 0.5)}, {}]


def test_list_shows_names_in_order(path):
    save_waypoints(path, {'start': Waypoint(0.0, 0.0, 0.0), 't1': Waypoint(1.0, 1.0, 0.0)})
    text, _ = recorder(path).handle('list')
    assert text.index('start') < text.index('t1')


@pytest.mark.parametrize('line', ['q', 'quit'])
def test_quit_stops_recorder(path, line):
    assert recorder(path).handle(line)[1] is False


def test_empty_line_does_nothing(path):
    assert recorder(path).handle('   ') == ('', True)


class Calls:
    def __init__(self):
        self.navigated = []
        self.initial = []

    def navigate(self, name, wp):
        self.navigated.append((name, wp))
        return f'arrived {name}'

    def set_initial_pose(self, wp):
        self.initial.append(wp)


def go_shell(path):
    calls = Calls()
    return GoShell(path, calls.navigate, calls.set_initial_pose), calls


def test_go_navigates_to_named_waypoint(path):
    save_waypoints(path, {'t1': Waypoint(1.0, 2.0, 0.5)})
    shell, calls = go_shell(path)
    text, running = shell.handle('go t1')
    assert (text, running) == ('arrived t1', True)
    assert calls.navigated == [('t1', Waypoint(1.0, 2.0, 0.5))]


def test_go_unknown_name_lists_known_ones(path):
    save_waypoints(path, {'t1': Waypoint(1.0, 2.0, 0.5), 'prep': Waypoint(3.0, 0.0, 0.0)})
    shell, calls = go_shell(path)
    text, _ = shell.handle('go t9')
    assert 't9' in text and 't1' in text and 'prep' in text
    assert calls.navigated == []


def test_bare_name_does_not_move_the_robot(path):
    save_waypoints(path, {'t1': Waypoint(1.0, 2.0, 0.5)})
    shell, calls = go_shell(path)
    text, running = shell.handle('t1')
    assert running
    assert 'go t1' in text
    assert calls.navigated == []


def test_waypoints_recorded_after_start_are_reachable(path):
    shell, calls = go_shell(path)
    save_waypoints(path, {'prep': Waypoint(3.0, 0.0, 0.0)})
    shell.handle('go prep')
    assert calls.navigated == [('prep', Waypoint(3.0, 0.0, 0.0))]


def test_init_sets_initial_pose_from_waypoint(path):
    save_waypoints(path, {'start': Waypoint(-1.0, 0.0, 0.0)})
    shell, calls = go_shell(path)
    text, _ = shell.handle('init start')
    assert 'start' in text
    assert calls.initial == [Waypoint(-1.0, 0.0, 0.0)]
    assert calls.navigated == []


def test_quit_stops_go_shell(path):
    shell, _ = go_shell(path)
    assert shell.handle('q')[1] is False


def test_malformed_file_is_reported_instead_of_crashing(path):
    path.write_text('waypoints:\n  t2: {x: 1.0}\n')
    shell, calls = go_shell(path)
    text, running = shell.handle('go t2')
    assert running
    assert 't2' in text
    assert calls.navigated == []


def test_arrival_report_gives_time():
    text = arrival_report('t3', 'SUCCEEDED', 42.04)
    assert 't3' in text and '42.0' in text
    assert '90' not in text


@pytest.mark.parametrize('seconds, limit', [(95.0, '90'), (130.0, '120')])
def test_arrival_report_warns_over_rulebook_leg_limits(seconds, limit):
    assert limit in arrival_report('t3', 'SUCCEEDED', seconds)


@pytest.mark.parametrize('outcome, word', [('CANCELED', '取消'), ('FAILED', '失敗')])
def test_arrival_report_for_unfinished_goal(outcome, word):
    assert word in arrival_report('t3', outcome, 10.0)
