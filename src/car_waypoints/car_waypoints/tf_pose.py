"""Read the robot's pose in the map frame from TF."""
import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
import tf2_ros

from car_waypoints.ros_msgs import waypoint_from_transform


def make_listener():
    """
    Return (buffer, listener) with the listener on its own node and thread.

    The caller's node is deliberately not shared: BasicNavigator spins its own node,
    and two executors must not spin the same node.
    """
    buffer = tf2_ros.Buffer()
    return buffer, tf2_ros.TransformListener(buffer, None, spin_thread=True)


def stop_listener(listener):
    """Stop the listener thread, which is not a daemon and would block exit."""
    listener.executor.shutdown()
    listener.dedicated_listener_thread.join(timeout=3.0)
    listener.unregister()
    listener.node.destroy_node()


def lookup_pose(buffer, map_frame, base_frame, timeout=1.0):
    """Return the latest map -> base pose, or raise LookupError saying why."""
    try:
        tf = buffer.lookup_transform(
            map_frame, base_frame, Time(), timeout=Duration(seconds=timeout))
    except tf2_ros.TransformException as e:
        raise LookupError(f'{map_frame} -> {base_frame}：{e}')
    return waypoint_from_transform(tf)


def init_ros():
    """Start rclpy but leave Ctrl+C to Python so a goal can be cancelled."""
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
