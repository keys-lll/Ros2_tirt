"""ROS console entry points for the imported standalone mission interfaces."""
from pathlib import Path
import runpy
import sys
from ament_index_python.packages import get_package_share_directory


def _run(filename):
    directory = Path(get_package_share_directory('auto')) / 'scripts'
    sys.path.insert(0, str(directory))
    runpy.run_path(str(directory / filename), run_name='__main__')


def run_mission():
    return _run('run_mission.py')


def mission_gui():
    return _run('mission_gui.py')


def operator_gui():
    return _run('operator_gui.py')
