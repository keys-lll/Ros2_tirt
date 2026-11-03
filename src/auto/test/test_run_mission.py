"""run_mission.py, the single-file version: flow, points and Nav2 glue."""
import importlib.util
import math
from pathlib import Path
import sys

import pytest


def load_module():
    path = Path(__file__).parents[1] / 'run_mission.py'
    spec = importlib.util.spec_from_file_location('run_mission', path)
    module = importlib.util.module_from_spec(spec)
    sys.modules['run_mission'] = module
    spec.loader.exec_module(module)
    return module


rm = load_module()


class Harness:
    """Scripted operator, fake navigator and a single ordered event log."""

    def __init__(self, answers=(), outcomes=()):
        self.answers = list(answers)
        self.outcomes = list(outcomes)
        self.events = []

    def ask(self, prompt):
        self.events.append(('ask', prompt))
        if not self.answers:
            raise EOFError
        return self.answers.pop(0)

    def say(self, text):
        self.events.append(('say', text))

    def sleep(self, seconds):
        self.events.append(('sleep', seconds))

    def navigate(self, name):
        self.events.append(('go', name))
        outcome = self.outcomes.pop(0) if self.outcomes else 'SUCCEEDED'
        return rm.NavResult(outcome, f'{outcome} {name}', 1.0)

    def mission(self, stop_seconds=3.0, ready=None):
        return rm.Mission(self.navigate, self.ask, self.say, self.sleep, stop_seconds, ready)

    def of_kind(self, kind):
        return [e[1] for e in self.events if e[0] == kind]

    @property
    def visited(self):
        return self.of_kind('go')

    @property
    def said(self):
        return self.of_kind('say')


def round_answers(order='', extra='可樂'):
    """Enter to depart, order text, Enter, extra item, Enter, the 'no more' card."""
    return ['', order, '', extra, '', '']


def after_first_prompt(choices):
    """Departure Enter, the failure answers, then the rest of the round."""
    return [''] + choices + round_answers()[1:]


# ---------- 7 個點 ----------

def test_all_points_are_start_prep_then_tables_one_to_five():
    points = rm.all_points()
    assert rm.NAMES == ('start', 'prep', 't1', 't2', 't3', 't4', 't5')
    assert list(points) == list(rm.NAMES)
    assert points['start'] == rm.START
    assert points['prep'] == rm.PREP
    assert points['t3'] == rm.TABLES[3]


def test_unset_points_are_named_in_mission_order():
    points = {'start': (0, 0, 0), 'prep': None, 't1': (1, 0, 0), 't2': None}
    assert rm.unset_points(points) == ['prep', 't2']


def test_no_unset_points_when_everything_is_filled():
    assert rm.unset_points({'start': (0, 0, 0), 't1': (1, 0, 0)}) == []


def test_shipped_points_are_unset_or_three_numbers():
    for name, point in rm.all_points().items():
        assert point is None or (
            len(point) == 3 and all(isinstance(v, (int, float)) for v in point)), name


def test_pose_message_has_frame_position_and_heading_in_degrees():
    pose = rm.pose_from_point((1.0, -2.0, 90.0), 'map', rm_stamp())
    assert pose.header.frame_id == 'map'
    assert (pose.pose.position.x, pose.pose.position.y) == (1.0, -2.0)
    assert pose.pose.orientation.z == pytest.approx(math.sqrt(0.5))
    assert pose.pose.orientation.w == pytest.approx(math.sqrt(0.5))


def rm_stamp():
    from builtin_interfaces.msg import Time
    return Time(sec=1, nanosec=2)


# ---------- 簡章單趟時間提醒 ----------

def test_arrival_report_gives_time_without_warning():
    text = rm.arrival_report('t3', 'SUCCEEDED', 42.04)
    assert 't3' in text and '42.0' in text and '⚠' not in text


@pytest.mark.parametrize('seconds, limit', [(95.0, '90'), (130.0, '120')])
def test_arrival_report_warns_over_the_leg_limits(seconds, limit):
    assert limit in rm.arrival_report('t3', 'SUCCEEDED', seconds)


@pytest.mark.parametrize('outcome, word', [('CANCELED', '取消'), ('FAILED', '失敗')])
def test_arrival_report_for_an_unfinished_goal(outcome, word):
    assert word in rm.arrival_report('t3', outcome, 10.0)


# ---------- 桌號與分區 ----------

@pytest.mark.parametrize('first, allowed', [
    (1, (2, 3, 5)), (4, (2, 3, 5)), (2, (1, 4)), (3, (1, 4)), (5, (1, 4))])
def test_second_round_must_use_the_other_zone(first, allowed):
    assert rm.allowed_second_tables(first) == allowed


def test_ask_table_accepts_a_number_from_one_to_five():
    assert Harness(['3']).mission().ask_table('桌號> ') == 3


def test_ask_table_asks_again_after_bad_input():
    h = Harness(['9', 'abc', '0', '2'])
    assert h.mission().ask_table('桌號> ') == 2
    assert sum('1~5' in t for t in h.said) == 3


def test_ask_table_blank_means_none_when_allowed():
    assert Harness(['']).mission().ask_table('桌號> ', allow_blank=True) is None


def test_ask_table_refuses_blank_when_not_allowed():
    assert Harness(['', '4']).mission().ask_table('桌號> ') == 4


def test_ask_table_enforces_zone_change():
    h = Harness(['2', '5', '4'])
    assert h.mission().ask_table('桌號> ', allowed=(1, 4)) == 4
    assert sum('換區' in t for t in h.said) == 2


def test_ask_table_does_not_crash_on_lookalike_digits():
    assert Harness(['²', '３', '3']).mission().ask_table('桌號> ') == 3


@pytest.mark.parametrize('answer', ['q', 'Q', 'quit'])
def test_q_aborts_the_table_question(answer):
    with pytest.raises(rm.MissionAborted):
        Harness([answer]).mission().ask_table('桌號> ')


def test_end_of_input_aborts():
    with pytest.raises(rm.MissionAborted):
        Harness([]).mission().ask_table('桌號> ')


# ---------- 一個回合的三趟 ----------

def test_round_goes_table_prep_table_prep_table_start():
    h = Harness(round_answers())
    h.mission().run_round(3)
    assert h.visited == ['t3', 'prep', 't3', 'prep', 't3', 'start']


def test_every_departure_waits_for_the_operator_and_every_table_stop_is_still():
    h = Harness(round_answers())
    h.mission().run_round(3)
    kinds = [e[0] for e in h.events if e[0] != 'say']
    assert kinds == ['ask', 'go', 'sleep', 'ask', 'go'] * 3


def test_table_stop_lasts_the_configured_seconds():
    h = Harness(round_answers())
    h.mission(stop_seconds=1.5).run_round(3)
    assert h.of_kind('sleep') == [1.5, 1.5, 1.5]


def test_stop_seconds_default_comes_from_the_settings_block():
    mission = rm.Mission(None, None, None, None)
    assert mission.stop_seconds == rm.STOP_SECONDS


def test_first_prompt_names_the_table():
    h = Harness(round_answers())
    h.mission().run_round(3)
    assert '3 號桌' in h.of_kind('ask')[0]


def test_order_and_extra_item_are_shown_when_loading_food():
    h = Harness(round_answers(order='牛肉麵', extra='可樂'))
    h.mission().run_round(3)
    prompts = h.of_kind('ask')
    assert '牛肉麵' in prompts[2]
    assert '可樂' in prompts[4]


def test_no_extra_returns_to_start_without_third_trip():
    h = Harness(round_answers(extra=''))
    h.mission().run_round(3)
    assert h.visited == ['t3', 'prep', 't3', 'start']


def test_round_ends_with_a_completion_message():
    h = Harness(round_answers())
    h.mission().run_round(3)
    assert '完成' in h.said[-1]


@pytest.mark.parametrize('prompt_index', range(6))
def test_q_at_any_prompt_aborts_without_further_moves(prompt_index):
    h = Harness(round_answers()[:prompt_index] + ['q'])
    with pytest.raises(rm.MissionAborted):
        h.mission().run_round(3)
    assert len(h.visited) == prompt_index


# ---------- 導航失敗：重試 / 跳過 / 結束 ----------

def test_failed_leg_can_be_retried():
    h = Harness(after_first_prompt(['r']), outcomes=['FAILED'])
    h.mission().run_round(3)
    assert h.visited == ['t3', 't3', 'prep', 't3', 'prep', 't3', 'start']


def test_failed_leg_can_be_skipped_and_the_stop_still_happens():
    h = Harness(after_first_prompt(['s']), outcomes=['FAILED'])
    h.mission().run_round(3)
    assert h.visited == ['t3', 'prep', 't3', 'prep', 't3', 'start']
    assert h.of_kind('sleep') == [3.0, 3.0, 3.0]


def test_quitting_after_a_failure_aborts_where_the_car_stands():
    h = Harness(['', 'q'], outcomes=['FAILED'])
    with pytest.raises(rm.MissionAborted):
        h.mission().run_round(3)
    assert h.visited == ['t3']
    assert '重試' in h.of_kind('ask')[-1]


def test_unknown_choice_after_a_failure_is_asked_again():
    h = Harness(after_first_prompt(['x', 'r']), outcomes=['FAILED'])
    h.mission().run_round(3)
    assert h.visited[:2] == ['t3', 't3']
    assert any('r、s 或 q' in t for t in h.said)


@pytest.mark.parametrize('outcome', ['FAILED', 'CANCELED', 'NOT_READY', 'UNKNOWN'])
def test_anything_but_success_asks_what_to_do(outcome):
    h = Harness(after_first_prompt(['s']), outcomes=[outcome])
    h.mission().run_round(3)
    kinds = [e[0] for e in h.events if e[0] != 'say']
    assert kinds[:3] == ['ask', 'go', 'ask']
    assert '重試' in h.of_kind('ask')[1] and '跳過' in h.of_kind('ask')[1]


# ---------- 兩個回合 ----------

def test_rounds_play_two_and_the_second_must_change_zone():
    h = Harness(['3', ''] + round_answers() + ['2', '4'] + round_answers())
    h.mission().run_rounds()
    assert h.visited == ['start',
                         't3', 'prep', 't3', 'prep', 't3', 'start',
                         't4', 'prep', 't4', 'prep', 't4', 'start']
    assert any('換區' in t for t in h.said)


def test_second_round_prompt_lists_the_allowed_tables():
    h = Harness(['3', ''] + round_answers() + [''])
    h.mission().run_rounds()
    assert '1、4' in h.of_kind('ask')[8]


def test_blank_answer_after_round_one_ends_the_session():
    h = Harness(['3', ''] + round_answers() + [''])
    h.mission().run_rounds()
    assert len(h.visited) == 1 + 6


def test_front_zone_first_then_back_zone():
    h = Harness(['1', ''] + round_answers() + ['5'] + round_answers())
    h.mission().run_rounds()
    assert h.visited[1] == 't1' and h.visited[7] == 't5'


def test_q_at_the_first_table_question_moves_nothing():
    h = Harness(['q'])
    with pytest.raises(rm.MissionAborted):
        h.mission().run_rounds()
    assert h.visited == []


# ---------- 只用已經設定的點 ----------

SOME = ['start', 'prep', 't1']


def test_ask_table_refuses_a_table_without_coordinates():
    h = Harness(['2', '3'])
    assert h.mission(ready=['start', 'prep', 't1', 't3']).ask_table('桌號> ') == 3
    assert any('t2 還沒設定座標' in t for t in h.said)
    assert not any('換區' in t for t in h.said)


def test_rounds_only_accept_tables_that_are_set_and_skip_a_second_round_with_no_table():
    h = Harness(['2', '1', ''] + round_answers())
    h.mission(ready=SOME).run_rounds()
    assert h.visited == ['start', 't1', 'prep', 't1', 'prep', 't1', 'start']
    assert any('t2 還沒設定座標' in t for t in h.said)
    assert len(h.of_kind('ask')) == 2 + 1 + 6
    assert any('第二回合' in t and '還沒設定座標' in t for t in h.said)


def test_rounds_say_so_when_no_table_is_set():
    h = Harness([])
    assert h.mission(ready=['start', 'prep']).run_rounds() is False
    assert any('1~5 號桌都還沒設定座標' in t for t in h.said)
    assert h.of_kind('ask') == []


@pytest.mark.parametrize('missing', ['start', 'prep'])
def test_rounds_need_start_and_prep(missing):
    h = Harness([])
    ready = [n for n in rm.NAMES if n != missing]
    assert h.mission(ready=ready).run_rounds() is False
    assert any(missing in t and '比賽流程' in t for t in h.said)
    assert h.visited == []


# ---------- 啟動後：自訂路線，或比賽流程 ----------

def test_startup_lists_which_points_are_set_and_which_are_not():
    h = Harness(['q'])
    with pytest.raises(rm.MissionAborted):
        h.mission(ready=SOME).run()
    assert 'start' in h.said[0] and 't1' in h.said[0]
    assert '還沒設定座標' in h.said[1] and 't2' in h.said[1] and 't5' in h.said[1]


def test_startup_has_no_missing_line_when_all_points_are_set():
    h = Harness(['q'])
    with pytest.raises(rm.MissionAborted):
        h.mission().run()
    assert not any('還沒設定座標' in t for t in h.said)


def test_route_visits_the_typed_points_waiting_for_enter_before_each():
    h = Harness(['start t1 prep', '', '', '', 'q'])
    with pytest.raises(rm.MissionAborted):
        h.mission().run()
    assert h.visited == ['start', 't1', 'prep']
    kinds = [e[0] for e in h.events if e[0] != 'say']
    assert kinds == ['ask', 'ask', 'go', 'ask', 'go', 'ask', 'go', 'ask']
    assert any('路線完成' in t for t in h.said)


def test_route_prompts_name_the_next_point_and_position():
    h = Harness(['t1 prep', '', '', 'q'])
    with pytest.raises(rm.MissionAborted):
        h.mission().run()
    prompts = h.of_kind('ask')
    assert 't1' in prompts[1] and '1/2' in prompts[1]
    assert 'prep' in prompts[2] and '2/2' in prompts[2]


def test_route_may_repeat_points_and_ignores_case():
    h = Harness(['T3 prep t3', '', '', '', 'q'])
    with pytest.raises(rm.MissionAborted):
        h.mission().run()
    assert h.visited == ['t3', 'prep', 't3']


def test_route_with_an_unknown_name_moves_nothing_and_asks_again():
    h = Harness(['start t9', 'start', '', 'q'])
    with pytest.raises(rm.MissionAborted):
        h.mission().run()
    assert h.visited == ['start']
    assert any('沒有「t9」' in t for t in h.said)


def test_route_with_a_point_that_has_no_coordinates_moves_nothing():
    h = Harness(['start t2', 'q'])
    with pytest.raises(rm.MissionAborted):
        h.mission(ready=SOME).run()
    assert h.visited == []
    assert any('t2 還沒設定座標' in t for t in h.said)


def test_q_at_a_route_departure_prompt_stops_where_the_car_is():
    h = Harness(['start t1 prep', '', 'q'])
    with pytest.raises(rm.MissionAborted):
        h.mission().run()
    assert h.visited == ['start']


def test_a_failed_route_leg_can_be_retried():
    h = Harness(['t1 prep', '', 'r', '', 'q'], outcomes=['FAILED'])
    with pytest.raises(rm.MissionAborted):
        h.mission().run()
    assert h.visited == ['t1', 't1', 'prep']


def test_blank_at_startup_runs_the_competition_rounds_and_then_ends():
    h = Harness([''] + ['3', ''] + round_answers() + [''])
    h.mission().run()
    assert h.visited == ['start', 't3', 'prep', 't3', 'prep', 't3', 'start']


def test_competition_that_cannot_start_returns_to_the_startup_question():
    h = Harness(['', 'start prep', '', '', 'q'])
    with pytest.raises(rm.MissionAborted):
        h.mission(ready=['start', 'prep']).run()
    assert h.visited == ['start', 'prep']
    assert any('1~5 號桌都還沒設定座標' in t for t in h.said)


# ---------- 比賽流程要先回到 start ----------

def test_rounds_go_back_to_start_before_the_first_trip():
    h = Harness(['3', ''] + round_answers() + [''])
    h.mission().run_rounds()
    kinds = [e[0] for e in h.events if e[0] != 'say']
    assert kinds[:4] == ['ask', 'ask', 'go', 'ask']
    assert h.visited[:2] == ['start', 't3']
    assert 'start' in h.of_kind('ask')[1]


def test_going_back_to_start_waits_for_enter_and_q_moves_nothing():
    h = Harness(['3', 'q'])
    with pytest.raises(rm.MissionAborted):
        h.mission().run_rounds()
    assert h.visited == []


def test_a_failed_return_to_start_can_be_retried():
    h = Harness(['3', '', 'r'] + round_answers() + [''], outcomes=['FAILED'])
    h.mission().run_rounds()
    assert h.visited[:3] == ['start', 'start', 't3']


def test_only_the_first_round_goes_back_to_start_first():
    h = Harness(['3', ''] + round_answers() + ['4'] + round_answers())
    h.mission().run_rounds()
    # 開頭回 start 一次，加上兩個回合各自最後一步回 start
    assert h.visited.count('start') == 3
    assert h.visited[7] == 't4'
