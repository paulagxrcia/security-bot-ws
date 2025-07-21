# Copyright 2024 Husarion sp. z o.o.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import SetParameter
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ParseMultiRobotPose

from ament_index_python.packages import get_package_share_directory
import os 

def launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration("world").perform(context)
    headless = LaunchConfiguration("headless").perform(context)


    gz_args = f"--headless-rendering -s -v 4 -r {world}" if eval(headless) else f"-r {world}"

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                ]
            )
        ),
        launch_arguments={
            "gz_args": gz_args,
            "on_exit_shutdown": "True",
        }.items(),
    )

    return[gz_sim]




def generate_launch_description():


    warehouse_world_dir = get_package_share_directory('warehouse_world')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(warehouse_world_dir, 'worlds', 'zm_robot_warehouse.sdf'),
        description='Full path to world model file to load'
    )

    model_path = os.path.join(warehouse_world_dir, 'models')

    declare_headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="False",
        description="Run Gazebo Ignition in the headless mode",
    )
    
    

    return LaunchDescription(
        [
            declare_world_cmd,
            declare_headless_arg,
            SetParameter(name="use_sim_time", value=True),
            OpaqueFunction(function=launch_setup),
            SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', model_path)
        ]
    )
