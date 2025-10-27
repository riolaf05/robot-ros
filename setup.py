import os
from glob import glob
from setuptools import setup

package_name = 'robot_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
        (os.path.join('share', package_name, 'description'), glob('description/*.xacro')),
        (os.path.join('share', package_name, 'description'), glob('description/*.urdf')),
        (os.path.join('share', package_name, 'description', 'config'), glob('description/config/*')),
        #(os.path.join('share', package_name, 'data'), glob('data/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'bt'), glob('bt/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rosario',
    maintainer_email='lafacerosario@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav_behaviour_tree = robot_ros.nav_behaviour_tree:main',
            'talker = robot_ros.publisher_member_function:main',
            'cmdVel_to_pwm_node = robot_ros.cmd_to_pwm_driver:main',
            'motor_controller_custom_node = robot_ros.motor_controller_custom:main',
            'odom_publisher_node = robot_ros.odom_publisher:main'
        ],
    },
)
