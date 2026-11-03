"""wp_go: type 'go t1' to drive to a recorded waypoint, one at a time."""
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

from car_waypoints.nav_result import NavResult
from car_waypoints.ros_msgs import to_initial_pose, to_pose_stamped
from car_waypoints.shell import arrival_report, GoShell
from car_waypoints.store import DEFAULT_FILE
from car_waypoints.tf_pose import init_ros, lookup_pose, make_listener, stop_listener


class Go(BasicNavigator):
    def __init__(self):
        super().__init__('wp_go')
        self.waypoint_file = self.declare_parameter('file', DEFAULT_FILE).value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.buffer, self.listener = make_listener()
        self.initial_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)

    def close(self):
        stop_listener(self.listener)
        self.destroy_node()

    def stamp(self):
        return self.get_clock().now().to_msg()

    def set_initial_pose(self, wp):
        # 用自己的 initialpose：BasicNavigator.setInitialPose 的共變異數是 0
        self.initial_pub.publish(to_initial_pose(wp, self.map_frame, self.stamp()))

    def navigate(self, name, wp):
        return self.navigate_result(name, wp).text

    def navigate_result(self, name, wp):
        try:
            lookup_pose(self.buffer, self.map_frame, self.base_frame)
        except LookupError as e:
            return NavResult(
                'NOT_READY', f'車子還沒定位（{e}），先設定初始位置（RViz 或 wp_go 的 init <名稱>）')
        # goToPose 找不到 action server 會無限等待下去，所以先自己確認
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=2.0):
            return NavResult('NOT_READY', '找不到 Nav2（navigate_to_pose），請先啟動 navigation2.launch.py')
        if not self.goToPose(to_pose_stamped(wp, self.map_frame, self.stamp())):
            return NavResult('FAILED', f'Nav2 拒絕了前往 {name} 的目標')
        started = time.monotonic()
        last_print = 0.0
        try:
            while not self.isTaskComplete():
                feedback = self.getFeedback()
                if feedback and time.monotonic() - last_print >= 1.0:
                    last_print = time.monotonic()
                    print(f'\r前往 {name}：還有 {feedback.distance_remaining:.2f} m   ',
                          end='', flush=True)
        except KeyboardInterrupt:
            self.cancelTask()
            print()
            seconds = time.monotonic() - started
            return NavResult('CANCELED', arrival_report(name, 'CANCELED', seconds), seconds)
        print()
        seconds = time.monotonic() - started
        outcome = self.getResult().name
        return NavResult(outcome, arrival_report(name, outcome, seconds), seconds)


def main():
    init_ros()
    node = Go()
    shell = GoShell(node.waypoint_file, node.navigate, node.set_initial_pose)
    print(f'點位檔：{node.waypoint_file}')
    print('init <名稱> 設定車子位置，go <名稱> 出發（Ctrl+C 取消）；help 看說明，q 離開')
    try:
        running = True
        while running:
            try:
                text, running = shell.handle(input('wp_go> '))
            except EOFError:
                break
            if text:
                print(text)
    except KeyboardInterrupt:
        print()
    finally:
        node.close()
        rclpy.try_shutdown()
