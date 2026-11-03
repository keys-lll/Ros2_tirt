"""run_mission with the real Go node, a fake TF tree and a fake NavigateToPose server."""
import os
import threading
import time

# 實車可能在同一個網路上跑，測試只在本機、用專屬 domain
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['ROS_DOMAIN_ID'] = '89'

import pytest  # noqa: E402
import rclpy  # noqa: E402
from geometry_msgs.msg import TransformStamped  # noqa: E402
from nav2_msgs.action import NavigateToPose  # noqa: E402
from rclpy.action import ActionServer  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from tf2_ros import StaticTransformBroadcaster  # noqa: E402

from auto.mission import run_mission  # noqa: E402
from auto.settings import load_settings  # noqa: E402
from car_waypoints.go import Go  # noqa: E402
from car_waypoints.store import save_waypoints, Waypoint  # noqa: E402

NAMES = ('start', 'prep', 't1', 't2', 't3', 't4', 't5')


class FakeRobot(Node):
    """Publishes map -> base_link and serves NavigateToPose."""

    def __init__(self):
        super().__init__('fake_robot_for_mission')
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
def go(robot):
    robot.outcome = 'succeed'
    robot.goals.clear()
    node = Go()
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


def settings_for(tmp_path, names=NAMES):
    path = tmp_path / 'waypoints.yaml'
    save_waypoints(path, {n: Waypoint(float(i), 0.0, 0.0) for i, n in enumerate(names)})
    return load_settings(tmp_path / 'none.yaml')._replace(
        waypoint_file=str(path), stop_seconds=0.0)


ROUND = ['', '', '', '', '', '']


def test_round_sends_the_recorded_goals_in_trip_order(go, robot, tmp_path):
    op = Operator(['3'] + ROUND + [''])
    code = run_mission(go, settings_for(tmp_path), op.ask, op.say, lambda s: None)
    assert code == 0
    # x 座標 = 點位在 NAMES 裡的順序：start 0, prep 1, t1 2, ... t3 4
    assert [g.pose.position.x for g in robot.goals] == [4.0, 1.0, 4.0, 1.0, 4.0, 0.0]
    assert all(g.header.frame_id == 'map' for g in robot.goals)


def test_missing_points_are_named_and_nothing_moves(go, robot, tmp_path):
    op = Operator(['3'] + ROUND)
    code = run_mission(go, settings_for(tmp_path, NAMES[:3]), op.ask, op.say, lambda s: None)
    assert code == 1
    assert any('t2' in t and 't5' in t for t in op.said)
    assert robot.goals == []
    assert op.answers == ['3'] + ROUND


def test_malformed_waypoint_file_is_reported(go, robot, tmp_path):
    settings = settings_for(tmp_path)
    (tmp_path / 'waypoints.yaml').write_text('waypoints:\n  t1: {x: 1.0}\n')
    op = Operator(['3'] + ROUND)
    assert run_mission(go, settings, op.ask, op.say, lambda s: None) == 1
    assert any('t1' in t for t in op.said)
    assert robot.goals == []


def test_nav2_failure_reaches_the_operator_prompt(go, robot, tmp_path):
    robot.outcome = 'abort'
    op = Operator(['3', '', 'q'])
    code = run_mission(go, settings_for(tmp_path), op.ask, op.say, lambda s: None)
    assert code == 0
    assert len(robot.goals) == 1
    # 最後一個問題必須是「這一步沒成功」，不是下一步的點餐卡提示
    assert '重試' in op.prompts[-1]
    assert any('任務結束' in t for t in op.said)
