"""Recorder and Go against a fake TF tree and a fake NavigateToPose server."""
import os
import threading
import time

# 實車可能在同一個網路上跑，測試只在本機、用專屬 domain
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['ROS_DOMAIN_ID'] = '87'

import pytest  # noqa: E402
import rclpy  # noqa: E402
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped  # noqa: E402
from nav2_msgs.action import NavigateToPose  # noqa: E402
from rclpy.action import ActionServer  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from tf2_ros import StaticTransformBroadcaster  # noqa: E402
from visualization_msgs.msg import Marker, MarkerArray  # noqa: E402

from car_waypoints.go import Go  # noqa: E402
from car_waypoints.record import Recorder  # noqa: E402
from car_waypoints.store import Waypoint, yaw_to_quaternion  # noqa: E402


class Fake(Node):
    """Publishes map -> base_link and serves NavigateToPose."""

    def __init__(self):
        super().__init__('fake_robot')
        self.goals = []
        self.outcome = 'succeed'
        self.broadcaster = StaticTransformBroadcaster(self)
        self.server = ActionServer(self, NavigateToPose, 'navigate_to_pose', self.execute)

    def publish_robot_at(self, x, y, yaw):
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = 'map', 'base_link'
        tf.transform.translation.x, tf.transform.translation.y = x, y
        q = tf.transform.rotation
        q.x, q.y, q.z, q.w = yaw_to_quaternion(yaw)
        self.broadcaster.sendTransform(tf)

    def execute(self, goal_handle):
        self.goals.append(goal_handle.request.pose)
        feedback = NavigateToPose.Feedback()
        feedback.distance_remaining = 1.0
        goal_handle.publish_feedback(feedback)
        time.sleep(0.3)
        if self.outcome == 'abort':
            goal_handle.abort()
        else:
            goal_handle.succeed()
        return NavigateToPose.Result()


@pytest.fixture(scope='module')
def fake():
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    node = Fake()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    yield node
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


@pytest.fixture
def robot_at_origin(fake):
    fake.publish_robot_at(0.0, 0.0, 0.0)
    fake.outcome = 'succeed'
    fake.goals.clear()
    return fake


def wait_for(condition, seconds=5.0):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if condition():
            return True
        time.sleep(0.05)
    return False


def test_recorder_reads_pose_from_tf(fake, tmp_path):
    fake.publish_robot_at(1.5, -0.25, 0.5)
    node = Recorder()
    try:
        assert wait_for(lambda: _pose_or_none(node) is not None)
        wp = node.current_pose()
        assert (wp.x, wp.y) == (1.5, -0.25)
        assert wp.yaw == pytest.approx(0.5)
    finally:
        node.close()


def _pose_or_none(node):
    try:
        return node.current_pose()
    except LookupError:
        return None


def test_recorder_reports_lookup_error_when_frame_is_missing(fake):
    node = Recorder()
    node.set_parameters([rclpy.parameter.Parameter(
        'base_frame', rclpy.Parameter.Type.STRING, 'no_such_frame')])
    node.base_frame = 'no_such_frame'
    try:
        with pytest.raises(LookupError, match='no_such_frame'):
            node.current_pose()
    finally:
        node.close()


def test_recorder_markers_reach_a_late_subscriber(fake):
    node = Recorder()
    got = []
    listener = Node('marker_listener')
    try:
        node.publish_markers({'t1': Waypoint(1.0, 2.0, 0.0)})
        from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=QoSReliabilityPolicy.RELIABLE)
        listener.create_subscription(MarkerArray, 'waypoint_markers', got.append, qos)
        end = time.monotonic() + 5
        while not got and time.monotonic() < end:
            rclpy.spin_once(listener, timeout_sec=0.1)
        assert got, 'no marker array received'
        assert got[0].markers[0].action == Marker.DELETEALL
        assert [m.text for m in got[0].markers if m.text] == ['t1']
    finally:
        listener.destroy_node()
        node.close()


def test_go_reaches_waypoint_and_reports(robot_at_origin):
    node = Go()
    try:
        assert wait_for(lambda: _go_localised(node))
        text = node.navigate('t1', Waypoint(2.0, 1.0, 0.0))
        assert text.startswith('到達 t1')
        assert len(robot_at_origin.goals) == 1
        goal = robot_at_origin.goals[0]
        assert goal.header.frame_id == 'map'
        assert (goal.pose.position.x, goal.pose.position.y) == (2.0, 1.0)
    finally:
        node.close()


def _go_localised(node):
    try:
        node.navigate_ready = node.buffer.can_transform(
            'map', 'base_link', rclpy.time.Time())
    except Exception:
        return False
    return node.navigate_ready


def test_go_reports_failure_when_nav2_aborts(robot_at_origin):
    robot_at_origin.outcome = 'abort'
    node = Go()
    try:
        assert wait_for(lambda: _go_localised(node))
        assert '失敗' in node.navigate('t1', Waypoint(2.0, 1.0, 0.0))
    finally:
        node.close()


def test_go_refuses_to_start_when_not_localised(fake):
    node = Go()
    node.base_frame = 'no_such_frame'
    try:
        text = node.navigate('t1', Waypoint(2.0, 1.0, 0.0))
        assert 'init' in text
    finally:
        node.close()


def test_init_publishes_pose_with_covariance(robot_at_origin):
    node = Go()
    got = []
    listener = Node('init_listener')
    try:
        listener.create_subscription(PoseWithCovarianceStamped, 'initialpose', got.append, 10)
        end = time.monotonic() + 5
        while not got and time.monotonic() < end:
            node.set_initial_pose(Waypoint(-1.0, 0.5, 0.0))
            rclpy.spin_once(listener, timeout_sec=0.2)
        assert got, 'no initialpose received'
        assert got[0].pose.pose.position.x == -1.0
        assert got[0].pose.covariance[0] > 0
    finally:
        listener.destroy_node()
        node.close()


@pytest.mark.parametrize('factory', [Recorder, Go])
def test_close_stops_the_tf_listener_thread(fake, factory):
    node = factory()
    thread = node.listener.dedicated_listener_thread
    assert thread.is_alive()
    node.close()
    thread.join(timeout=3)
    assert not thread.is_alive()


def test_node_is_not_spun_by_the_tf_listener(fake):
    # BasicNavigator 會自己 spin 這個節點，TF 監聽的 executor 不能同時搶它
    node = Go()
    try:
        assert node.listener.node is not node
    finally:
        node.close()


def test_go_says_nav2_is_not_running_instead_of_waiting_forever(robot_at_origin, monkeypatch):
    node = Go()
    monkeypatch.setattr(node.nav_to_pose_client, 'wait_for_server', lambda timeout_sec=None: False)
    try:
        assert wait_for(lambda: _go_localised(node))
        text = node.navigate('t1', Waypoint(2.0, 1.0, 0.0))
        assert 'Nav2' in text
        assert robot_at_origin.goals == []
    finally:
        node.close()


def test_navigate_result_reports_success_with_seconds(robot_at_origin):
    node = Go()
    try:
        assert wait_for(lambda: _go_localised(node))
        result = node.navigate_result('t1', Waypoint(2.0, 1.0, 0.0))
        assert result.outcome == 'SUCCEEDED'
        assert result.ok
        assert result.seconds >= 0.3
        assert result.text.startswith('到達 t1')
    finally:
        node.close()


def test_navigate_result_reports_failure_when_nav2_aborts(robot_at_origin):
    robot_at_origin.outcome = 'abort'
    node = Go()
    try:
        assert wait_for(lambda: _go_localised(node))
        result = node.navigate_result('t1', Waypoint(2.0, 1.0, 0.0))
        assert result.outcome == 'FAILED'
        assert not result.ok
    finally:
        node.close()


def test_navigate_result_is_not_ready_before_localisation(fake):
    node = Go()
    node.base_frame = 'no_such_frame'
    try:
        result = node.navigate_result('t1', Waypoint(2.0, 1.0, 0.0))
        assert result.outcome == 'NOT_READY'
        assert not result.ok
        assert 'init' in result.text
    finally:
        node.close()


def test_navigate_result_is_not_ready_when_nav2_is_absent(robot_at_origin, monkeypatch):
    node = Go()
    monkeypatch.setattr(node.nav_to_pose_client, 'wait_for_server', lambda timeout_sec=None: False)
    try:
        assert wait_for(lambda: _go_localised(node))
        result = node.navigate_result('t1', Waypoint(2.0, 1.0, 0.0))
        assert result.outcome == 'NOT_READY'
        assert 'Nav2' in result.text
    finally:
        node.close()


def test_navigate_result_cancels_the_goal_on_ctrl_c(robot_at_origin, monkeypatch):
    node = Go()
    cancelled = []

    def interrupt():
        raise KeyboardInterrupt

    monkeypatch.setattr(node, 'isTaskComplete', interrupt)
    monkeypatch.setattr(node, 'cancelTask', lambda: cancelled.append(True))
    try:
        assert wait_for(lambda: _go_localised(node))
        result = node.navigate_result('t1', Waypoint(2.0, 1.0, 0.0))
        assert result.outcome == 'CANCELED'
        assert cancelled == [True]
    finally:
        node.close()


def test_navigate_still_returns_the_text_for_wp_go(robot_at_origin):
    node = Go()
    try:
        assert wait_for(lambda: _go_localised(node))
        text = node.navigate('t1', Waypoint(2.0, 1.0, 0.0))
        assert isinstance(text, str) and text.startswith('到達 t1')
    finally:
        node.close()
