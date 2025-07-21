from launch import LaunchDescription
from launch.actions import  ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='rosbot_xl_behaviour',
            executable='security_patrol',
            name='security_patrol_node',
            remappings =[('/cmd_vel', '/cmd_vel_patrol')]
        ),

        Node(
            package='rosbot_xl_behaviour',
            executable ='shelf_inspection',
            name='shelf_inspection_node'
        ),

        Node(
            package='yolo',
            executable='yolo_person_detection_node',
            name='person_detector'
        ),

        Node(
            package='rosbot_xl_behaviour',
            executable = 'alarm_alert_node',
            name = 'alarm_alert_node',
            emulate_tty = True
        ),

        Node(
            package='rosbot_xl_behaviour',
            executable='human_tracking',
            name='intruder_tracking',
            remappings=[('/cmd_vel', '/cmd_vel_tracking')]
        ),

        Node(
            package = 'rosbot_xl_behaviour',
            executable = 'video_capture_node',
            name='video_capture_node'
        ),

        Node(
            package='rosbot_xl_behaviour',
            executable = 'intruder_recorder_node',
            name='intruder_recorder_node'
        ),

        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_keyboard',
            prefix='xterm -e',
            remappings=[('/cmd_vel', '/cmd_vel_teleop')]
        ),

        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            parameters=['/home/paula/security_bot_ws/src/rosbot_xl_behaviour/config/twist_mux.yaml'],
            remappings=[('/cmd_vel_out', '/cmd_vel')]
        ),

        # Node(
        #     package='rosbot_xl_behaviour',
        #     executable='control_node',
        #     name='control_node',
        #     prefix='xterm -e'
        # ),

        ExecuteProcess(
            cmd=[
                'xterm', '-e', 'bash', '-c',
                'source /home/paula/security_bot_ws/install/setup.bash && ros2 run rosbot_xl_behaviour control_node'
            ],
            name='control_node_terminal',
            output='screen'
        )

    ])

    

