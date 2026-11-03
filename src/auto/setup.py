from setuptools import find_packages, setup
from glob import glob

package_name = 'auto'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/scripts', ['run_mission.py', 'mission_gui.py', 'operator_gui.py', 'terminal_io.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leo',
    maintainer_email='willie040600@gmail.com',
    description='Run the delivery competition mission over recorded waypoints.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mission = auto.mission:main',
            'run_mission = auto.gui_entry:run_mission',
            'mission_gui = auto.gui_entry:mission_gui',
            'operator_gui = auto.gui_entry:operator_gui',
        ],
    },
)
