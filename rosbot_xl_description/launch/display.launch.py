#!/usr/bin/env python3

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, SetParameter, SetRemap
from launch_ros.substitutions import FindPackageShare

def contains_cam_component(yaml_fil):
    with open(yaml_fil, "r") as file:
        data = yaml.safe_load(file)
        if "components" in data:
            return any(item["type"].startswith("CAM") for item in data["components"])
    return False

def launch_setup(context, *args, **kwargs):
    components_config = LaunchConfiguration("components_config").perform(context)
    mock_joints = LaunchConfiguration("mock_joints", default="True").perform(context)
    use_sim = LaunchConfiguration("use_sim", default="False").perform(context)
    lidar_model = LaunchConfiguration("lidar_model").perform(context)

    urdf_file = "rosbot_xl.urdf.xacro"
    include_camera_mount = str(contains_cam_component(components_config))
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("rosbot_xl_description"), "urdf", urdf_file]),
            " components_config:=",
            components_config,
            " include_camera_mount:=",
            include_camera_mount,
            " use_sim:=",
            use_sim,
            " lidar_model:=",
            lidar_model,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )

    return [
        SetParameter(name="use_sim_time", value=use_sim),
        SetRemap("/tf", "tf"),
        SetRemap("/tf_static", "tf_static"),
        robot_state_pub_node,
        joint_state_publisher_gui,
    ]

def generate_launch_description():
    declare_components_config_arg = DeclareLaunchArgument(
        "components_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("rosbot_xl_description"), "config", "components.yaml"]
        ),
        description=(
            "Specify file which contains components. These components will be included in URDF."
            "Available options can be found in manuals: https://husarion.com/manuals"
        ),
    )

    declare_lidar_model_arg = DeclareLaunchArgument(
        "lidar_model",
        default_value="slamtec_rplidar_s3",
        description="Add LiDAR model to the robot URDF",
        choices=[
            "None",
            "slamtec_rplidar_a2",
            "slamtec_rplidar_a3",
            "slamtec_rplidar_s1",
            "slamtec_rplidar_s2",
            "slamtec_rplidar_s3",
            "velodyne_puck",
        ],
    )

    publish_robot_description = OpaqueFunction(function=launch_setup)

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("rosbot_xl_description"), "rviz", "rosbot.rviz"]
            ),
        ],
    )

    return LaunchDescription(
        [
            declare_components_config_arg,
            declare_lidar_model_arg,
            publish_robot_description,
            rviz_node,
        ]
    )

