"""mission: run the delivery rounds (three trips each) over recorded waypoints."""
import time

import rclpy

from auto.flow import Mission, MissionAborted
from auto.settings import DEFAULT_CONFIG, load_settings, missing_points
from car_waypoints.go import Go
from car_waypoints.store import load_waypoints
from car_waypoints.tf_pose import init_ros


def run_mission(node, settings, ask=input, say=print, sleep=time.sleep):
    """Check the waypoint file, then run the mission; returns the exit code."""
    try:
        waypoints = load_waypoints(settings.waypoint_file)
    except ValueError as e:
        say(f'點位檔有錯：{e}')
        return 1
    missing = missing_points(settings, waypoints)
    if missing:
        say(f'點位檔 {settings.waypoint_file} 缺少：{", ".join(missing)}')
        say('請先用 wp_record 把這些點記起來')
        return 1
    mission = Mission(
        settings, lambda name: node.navigate_result(name, waypoints[name]), ask, say, sleep)
    try:
        mission.run()
    except MissionAborted as e:
        say(f'任務結束：{e}')
    return 0


def main():
    init_ros()
    node = Go()
    try:
        config = node.declare_parameter('mission_config', DEFAULT_CONFIG).value
        try:
            settings = load_settings(config)
        except ValueError as e:
            print(f'設定檔有錯：{e}')
            return 1
        print(f'設定檔：{config}')
        print(f'點位檔：{settings.waypoint_file}')
        return run_mission(node, settings)
    except KeyboardInterrupt:
        print('\n已結束')
        return 0
    finally:
        node.close()
        rclpy.try_shutdown()
