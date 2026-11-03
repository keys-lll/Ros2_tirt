"""The three-trip mission flow with scripted operator input and a fake navigator."""
import pytest

from auto.flow import allowed_second_tables, Mission, MissionAborted
from auto.settings import Settings
from car_waypoints.nav_result import NavResult

SETTINGS = Settings('w.yaml', 3.0, 'start', 'prep', ('t1', 't2', 't3', 't4', 't5'))


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
        return NavResult(outcome, f'{outcome} {name}', 1.0)

    def mission(self, settings=SETTINGS):
        return Mission(settings, self.navigate, self.ask, self.say, self.sleep)

    def of_kind(self, kind):
        return [e[1] for e in self.events if e[0] == kind]

    @property
    def visited(self):
        return self.of_kind('go')

    @property
    def said(self):
        return self.of_kind('say')


@pytest.mark.parametrize('first, allowed', [
    (1, (2, 3, 5)), (4, (2, 3, 5)), (2, (1, 4)), (3, (1, 4)), (5, (1, 4))])
def test_second_round_must_use_the_other_zone(first, allowed):
    assert allowed_second_tables(first) == allowed


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


@pytest.mark.parametrize('answer', ['q', 'Q', 'quit'])
def test_q_aborts_the_table_question(answer):
    with pytest.raises(MissionAborted):
        Harness([answer]).mission().ask_table('桌號> ')


def test_end_of_input_aborts():
    with pytest.raises(MissionAborted):
        Harness([]).mission().ask_table('桌號> ')


def round_answers(order='', extra=''):
    """Enter to depart, order text, Enter, extra item, Enter, the 'no more' card."""
    return ['', order, '', extra, '', '']


def test_round_goes_table_prep_table_prep_table_start():
    h = Harness(round_answers())
    h.mission().run_round(3)
    assert h.visited == ['t3', 'prep', 't3', 'prep', 't3', 'start']


def test_round_uses_the_configured_point_names():
    custom = Settings('w.yaml', 3.0, 'door', 'kitchen', ('a', 'b', 'c', 'd', 'e'))
    h = Harness(round_answers())
    h.mission(custom).run_round(2)
    assert h.visited == ['b', 'kitchen', 'b', 'kitchen', 'b', 'door']


def test_every_departure_waits_for_the_operator_and_every_table_stop_is_still():
    h = Harness(round_answers())
    h.mission().run_round(3)
    kinds = [e[0] for e in h.events if e[0] != 'say']
    assert kinds == ['ask', 'go', 'sleep', 'ask', 'go'] * 3


def test_table_stop_lasts_the_configured_seconds():
    custom = SETTINGS._replace(stop_seconds=1.5)
    h = Harness(round_answers())
    h.mission(custom).run_round(3)
    assert h.of_kind('sleep') == [1.5, 1.5, 1.5]


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


def test_blank_order_and_extra_item_are_allowed():
    h = Harness(round_answers())
    h.mission().run_round(3)
    assert len(h.visited) == 6


def test_round_ends_with_a_completion_message():
    h = Harness(round_answers())
    h.mission().run_round(3)
    assert '完成' in h.said[-1]


@pytest.mark.parametrize('prompt_index', range(6))
def test_q_at_any_prompt_aborts_without_further_moves(prompt_index):
    h = Harness(round_answers()[:prompt_index] + ['q'])
    with pytest.raises(MissionAborted):
        h.mission().run_round(3)
    assert len(h.visited) == prompt_index


def after_first_prompt(choices):
    """Departure Enter, the failure answers, then the rest of the round."""
    return [''] + choices + round_answers()[1:]


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
    with pytest.raises(MissionAborted):
        h.mission().run_round(3)
    assert h.visited == ['t3']


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


def test_ask_table_does_not_crash_on_lookalike_digits():
    assert Harness(['²', '３', '3']).mission().ask_table('桌號> ') == 3


def test_run_plays_two_rounds_and_the_second_must_change_zone():
    h = Harness(['3'] + round_answers() + ['2', '4'] + round_answers())
    h.mission().run()
    assert h.visited == ['t3', 'prep', 't3', 'prep', 't3', 'start',
                         't4', 'prep', 't4', 'prep', 't4', 'start']
    assert any('換區' in t for t in h.said)


def test_second_round_prompt_lists_the_allowed_tables():
    h = Harness(['3'] + round_answers() + [''])
    h.mission().run()
    assert '1、4' in h.of_kind('ask')[7]


def test_blank_answer_after_round_one_ends_the_session():
    h = Harness(['3'] + round_answers() + [''])
    h.mission().run()
    assert len(h.visited) == 6


def test_front_zone_first_then_back_zone():
    h = Harness(['1'] + round_answers() + ['5'] + round_answers())
    h.mission().run()
    assert h.visited[0] == 't1' and h.visited[6] == 't5'


def test_q_at_the_first_table_question_moves_nothing():
    h = Harness(['q'])
    with pytest.raises(MissionAborted):
        h.mission().run()
    assert h.visited == []
