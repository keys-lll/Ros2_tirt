"""Waypoints converted to Nav2 goals, AMCL initial poses and RViz markers."""
import math

from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
import pytest
from visualization_msgs.msg import Marker

from car_waypoints.ros_msgs import (
    build_markers, to_initial_pose, to_pose_stamped, waypoint_from_transform, zone_style)
from car_waypoints.store import Waypoint, yaw_to_quaternion

STAMP = Time(sec=12, nanosec=34)


def test_goal_pose_has_frame_position_and_heading():
    msg = to_pose_stamped(Waypoint(1.0, -2.0, math.pi / 2), 'map', STAMP)
    assert msg.header.frame_id == 'map'
    assert msg.header.stamp == STAMP
    assert (msg.pose.position.x, msg.pose.position.y) == (1.0, -2.0)
    assert msg.pose.orientation.z == pytest.approx(math.sqrt(0.5))
    assert msg.pose.orientation.w == pytest.approx(math.sqrt(0.5))


def test_initial_pose_has_position_and_nonzero_uncertainty():
    msg = to_initial_pose(Waypoint(1.0, -2.0, 0.0), 'map', STAMP)
    assert msg.header.frame_id == 'map'
    assert msg.pose.pose.position.x == 1.0
    # AMCL 用 0 共變異數初始化會讓所有粒子擠在同一點
    assert msg.pose.covariance[0] > 0
    assert msg.pose.covariance[7] > 0
    assert msg.pose.covariance[35] > 0


@pytest.mark.parametrize('name, size', [
    ('start', 0.90), ('prep', 0.90), ('t1', 0.75), ('t5', 0.75), ('door', 0.33)])
def test_zone_sizes_follow_competition_rules(name, size):
    assert zone_style(name)[0] == size


def test_markers_clear_old_ones_then_draw_each_waypoint():
    markers = build_markers(
        {'start': Waypoint(0.0, 0.0, 0.0), 't1': Waypoint(1.0, 1.0, 0.0)}, 'map').markers
    assert markers[0].action == Marker.DELETEALL
    drawn = markers[1:]
    assert all(m.header.frame_id == 'map' for m in drawn)
    assert len({(m.ns, m.id) for m in drawn}) == len(drawn)
    zones = [m for m in drawn if m.type == Marker.CUBE]
    assert [(m.pose.position.x, m.scale.x) for m in zones] == [(0.0, 0.90), (1.0, 0.75)]
    assert [m.text for m in drawn if m.type == Marker.TEXT_VIEW_FACING] == ['start', 't1']
    assert len([m for m in drawn if m.type == Marker.ARROW]) == 2


def test_no_waypoints_only_clears():
    assert [m.action for m in build_markers({}, 'map').markers] == [Marker.DELETEALL]


def test_transform_becomes_waypoint():
    tf = TransformStamped()
    tf.transform.translation.x, tf.transform.translation.y = 1.5, -0.25
    q = tf.transform.rotation
    q.x, q.y, q.z, q.w = yaw_to_quaternion(-2.0)
    wp = waypoint_from_transform(tf)
    assert (wp.x, wp.y) == (1.5, -0.25)
    assert wp.yaw == pytest.approx(-2.0)
