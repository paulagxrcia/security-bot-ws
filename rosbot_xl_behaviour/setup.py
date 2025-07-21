from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rosbot_xl_behaviour'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='paula',
    maintainer_email='garciapaula297@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'shelf_inspection = rosbot_xl_behaviour.shelf_inspection_node:main', 
            'security_patrol = rosbot_xl_behaviour.security_patrol_node:main',
            'alarm_alert_node = rosbot_xl_behaviour.alarm_alert_node:main',
            'human_tracking = rosbot_xl_behaviour.human_tracking:main',
            'video_capture_node = rosbot_xl_behaviour.video_capture_node:main',
            'intruder_recorder_node = rosbot_xl_behaviour.intruder_recorder_node:main',
            'initial_pose_pub = rosbot_xl_behaviour.initial_pose_pub:main',
            'control_node = rosbot_xl_behaviour.control:main'


        ],
    },
)
