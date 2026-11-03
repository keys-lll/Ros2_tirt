"""Waypoint -> Nav2 goal, AMCL initial pose and RViz marker messages."""
import re

from geometry_msgs.msg import Point, Pose, PoseStamped, PoseWithCovarianceStamped, Quaternion
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from car_waypoints.store import quaternion_to_yaw, Waypoint, yaw_to_quaternion

# 車子是放進出發區框內才 init，誤差約 ±15 cm、±13°；RViz 預設 0.25/0.0685 太鬆
INITIAL_XY_VARIANCE = 0.02
INITIAL_YAW_VARIANCE = 0.05

# 簡章尺寸：出發區、備餐區 90 cm，紅色停等區 75 cm，其餘畫成車體 33 cm
ZONES = (
    (re.compile(r'start\Z'), 0.90, (0.1, 0.7, 0.2)),
    (re.compile(r'prep\Z'), 0.90, (0.1, 0.3, 0.9)),
    (re.compile(r't\d+\Z'), 0.75, (0.9, 0.1, 0.1)),
)
OTHER_ZONE = (0.33, (0.6, 0.6, 0.6))


def zone_style(name):
    """Return (square side in metres, rgb) used to draw a waypoint."""
    for pattern, size, rgb in ZONES:
        if pattern.match(name):
            return size, rgb
    return OTHER_ZONE


def waypoint_from_transform(tf):
    """Return the 2D pose of a map -> base TransformStamped."""
    t, q = tf.transform.translation, tf.transform.rotation
    return Waypoint(t.x, t.y, quaternion_to_yaw(q.x, q.y, q.z, q.w))


def to_pose(wp, z=0.0):
    qx, qy, qz, qw = yaw_to_quaternion(wp.yaw)
    return Pose(position=Point(x=wp.x, y=wp.y, z=z),
                orientation=Quaternion(x=qx, y=qy, z=qz, w=qw))


def to_pose_stamped(wp, frame_id, stamp):
    msg = PoseStamped(pose=to_pose(wp))
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    return msg


def to_initial_pose(wp, frame_id, stamp):
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.pose.pose = to_pose(wp)
    covariance = [0.0] * 36
    covariance[0] = covariance[7] = INITIAL_XY_VARIANCE
    covariance[35] = INITIAL_YAW_VARIANCE
    msg.pose.covariance = covariance
    return msg


def build_markers(waypoints, frame_id):
    """Clear previous markers, then draw a zone square, heading arrow and label per waypoint."""
    clear = Marker(action=Marker.DELETEALL)
    clear.header.frame_id = frame_id
    markers = [clear]
    for i, (name, wp) in enumerate(waypoints.items()):
        size, (r, g, b) = zone_style(name)
        zone = Marker(type=Marker.CUBE, pose=to_pose(wp, z=0.005),
                      color=ColorRGBA(r=r, g=g, b=b, a=0.35))
        zone.scale.x = zone.scale.y = size
        zone.scale.z = 0.01
        arrow = Marker(type=Marker.ARROW, pose=to_pose(wp, z=0.02),
                       color=ColorRGBA(r=r, g=g, b=b, a=1.0))
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.3, 0.05, 0.05
        label = Marker(type=Marker.TEXT_VIEW_FACING, pose=to_pose(wp, z=0.3), text=name,
                       color=ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0))
        label.scale.z = 0.2
        for k, marker in enumerate((zone, arrow, label)):
            marker.header.frame_id = frame_id
            marker.ns = 'waypoints'
            marker.id = 3 * i + k
            markers.append(marker)
    return MarkerArray(markers=markers)
