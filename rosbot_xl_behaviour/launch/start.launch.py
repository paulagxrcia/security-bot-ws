from launch import LaunchDescription
from launch.actions import  IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os 
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():

    gazebo_launch_path = os.path.join(
        get_package_share_directory('rosbot_xl_gazebo'),
        'launch',
        'warehouse_sim.launch.py'
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path)
    )

    rviz_nav2_launch_path = os.path.join(
        get_package_share_directory('rosbot_xl_navigation'),
        'launch',
        'nav2.launch.py'
    )

    rviz2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rviz_nav2_launch_path)
    )

    delayed_rviz2_launch = TimerAction(
        period=3.0,
        actions=[rviz2_launch]
    )



    return LaunchDescription([
        gazebo_launch,
        delayed_rviz2_launch,
    ])