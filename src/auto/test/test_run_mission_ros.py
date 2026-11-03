"""run_mission.py Navigator and run(), against a fake TF tree and NavigateToPose server."""
import os
import threading
import time

# 實車可能在同一個網路上跑，測試只在本機、用專屬 domain
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['ROS_DOMAIN_ID'] = '91'

import importlib.util  # noqa: E402
import math  # noqa: E402
from pathlib import Path  # noqa: E402
import sys  # noqa: E402

import pytest  # noqa: E402
import rclpy  # noqa: E402
from geometry_msgs.msg import TransformStamped  # noqa: E402
from nav2_msgs.action import NavigateToPose  # noqa: E402
from rclpy.action import ActionServer  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from tf2_ros import StaticTransformBroadcaster  # noqa: E402


def load_module():
    path = Path(__file__).parents[1] / 'run_mission.py'
    spec = importlib.util.spec_from_file_location('run_mission', path)
    module = importlib.util.module_from_spec(spec)
    sys.modules['run_mission'] = module
    spec.loader.exec_module(module)
    return module


rm = load_module()

NAMES = ('start', 'prep', 't1', 't2', 't3', 't4', 't5')
POINTS = {name: (float(i), 0.0, 0.0) for i, name in enumerate(NAMES)}
ROUND = ['', '', '', '', '', '']


class FakeRobot(Node):
    """Publishes map -> base_link and serves NavigateToPose."""

    def __init__(self):
        super().__init__('fake_robot_for_run_mission')
        self.goals = []
        self.outcome = 'succeed'
        self.broadcaster = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = 'map', 'base_link'
        tf.transform.rotation.w = 1.0
        self.broadcaster.sendTransform(tf)
        self.server = ActionServer(self, NavigateToPose, 'navigate_to_pose', self.execute)

    def execute(self, goal_handle):
        self.goals.append(goal_handle.request.pose)
        feedback = NavigateToPose.Feedback()
        feedback.distance_remaining = 1.0
        goal_handle.publish_feedback(feedback)
        time.sleep(0.2)
        if self.outcome == 'abort':
            goal_handle.abort()
        else:
            goal_handle.succeed()
        return NavigateToPose.Result()


@pytest.fixture(scope='module')
def robot():
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    node = FakeRobot()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    yield node
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


@pytest.fixture
def nav(robot):
    robot.outcome = 'succeed'
    robot.goals.clear()
    node = rm.Navigator()
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        if node.buffer.can_transform('map', 'base_link', rclpy.time.Time()):
            break
        time.sleep(0.05)
    yield node
    node.close()


class Operator:
    def __init__(self, answers):
        self.answers = list(answers)
        self.said = []
        self.prompts = []

    def ask(self, prompt):
        self.prompts.append(prompt)
        if not self.answers:
            raise EOFError
        return self.answers.pop(0)

    def say(self, text):
        self.said.append(text)


def test_navigate_reports_success_and_sends_the_goal(nav, robot):
    result = nav.navigate('t1', (2.0, 1.0, 90.0))
    assert result.outcome == 'SUCCEEDED' and result.ok
    assert result.seconds >= 0.2
    assert result.text.startswith('到達 t1')
    goal = robot.goals[0]
    assert goal.header.frame_id == 'map'
    assert (goal.pose.position.x, goal.pose.position.y) == (2.0, 1.0)
    assert goal.pose.orientation.z == pytest.approx(math.sqrt(0.5))


def test_navigate_reports_failure_when_nav2_aborts(nav, robot):
    robot.outcome = 'abort'
    result = nav.navigate('t1', (2.0, 1.0, 0.0))
    assert result.outcome == 'FAILED' and not result.ok


def test_navigate_is_not_ready_before_localisation(nav, monkeypatch):
    monkeypatch.setattr(rm, 'BASE_FRAME', 'no_such_frame')
    result = nav.navigate('t1', (2.0, 1.0, 0.0))
    assert result.outcome == 'NOT_READY'
    assert '定位' in result.text


def test_navigate_is_not_ready_when_nav2_is_absent(nav, robot, monkeypatch):
    monkeypatch.setattr(nav.nav_to_pose_client, 'wait_for_server', lambda timeout_sec=None: False)
    result = nav.navigate('t1', (2.0, 1.0, 0.0))
    assert result.outcome == 'NOT_READY'
    assert 'Nav2' in result.text
    assert robot.goals == []


def test_navigate_cancels_the_goal_on_ctrl_c(nav, monkeypatch):
    cancelled = []

    def interrupt():
        raise KeyboardInterrupt

    monkeypatch.setattr(nav, 'isTaskComplete', interrupt)
    monkeypatch.setattr(nav, 'cancelTask', lambda: cancelled.append(True))
    result = nav.navigate('t1', (2.0, 1.0, 0.0))
    assert result.outcome == 'CANCELED'
    assert cancelled == [True]


def test_close_stops_the_tf_listener_thread(robot):
    node = rm.Navigator()
    thread = node.listener.dedicated_listener_thread
    assert thread.is_alive()
    node.close()
    thread.join(timeout=3)
    assert not thread.is_alive()


def test_run_sends_the_points_in_trip_order(nav, robot):
    op = Operator([''] + ['3', ''] + ['', '', '', '可樂', '', ''] + [''])
    code = rm.run(nav, POINTS, op.ask, op.say, lambda s: None)
    assert code == 0
    # x 座標 = 點位在 NAMES 裡的順序：start 0, prep 1, t1 2, ... t3 4；開頭先回 start
    assert [g.pose.position.x for g in robot.goals] == [0.0, 4.0, 1.0, 4.0, 1.0, 4.0, 0.0]


def test_run_lets_you_visit_only_the_points_that_are_set(nav, robot):
    op = Operator(['t3 prep', '', '', 'q'])
    points = dict(POINTS, t2=None, t5=None)
    code = rm.run(nav, points, op.ask, op.say, lambda s: None)
    assert code == 0
    assert [g.pose.position.x for g in robot.goals] == [4.0, 1.0]
    assert any('t2' in t and 't5' in t and '還沒設定座標' in t for t in op.said)


def test_run_refuses_a_point_that_is_not_set_and_does_not_move(nav, robot):
    op = Operator(['t2', 'q'])
    points = dict(POINTS, t2=None)
    code = rm.run(nav, points, op.ask, op.say, lambda s: None)
    assert code == 0
    assert robot.goals == []
    assert any('t2 還沒設定座標' in t for t in op.said)


def test_run_refuses_to_start_when_no_point_is_set(nav, robot):
    op = Operator(['start'])
    code = rm.run(nav, {name: None for name in NAMES}, op.ask, op.say, lambda s: None)
    assert code == 1
    assert robot.goals == []
    assert op.prompts == []


def test_run_hands_a_nav2_failure_to_the_operator(nav, robot):
    robot.outcome = 'abort'
    op = Operator(['', '3', '', 'q'])
    code = rm.run(nav, POINTS, op.ask, op.say, lambda s: None)
    assert code == 0
    assert len(robot.goals) == 1
    # 最後一個問題必須是「這一步沒成功」，不是下一步的點餐卡提示
    assert '重試' in op.prompts[-1]
    assert any('任務結束' in t for t in op.said)


def test_no_extra_order_skips_third_trip(nav, robot):
    op = Operator([''] + ['3', ''] + ROUND + [''])
    assert rm.run(nav, POINTS, op.ask, op.say, lambda s: None) == 0
    assert [g.pose.position.x for g in robot.goals] == [0.0, 4.0, 1.0, 4.0, 0.0]
