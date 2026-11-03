from setuptools import find_packages, setup

package_name = 'stm32'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leo',
    maintainer_email='leo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "stm32_bridge = stm32.stm32_bridge:main",
            "stm32_feedback = stm32.stm32_feedback:main",
            "stm32_controller = stm32.stm32_controller:main"
        ],
    },
)
