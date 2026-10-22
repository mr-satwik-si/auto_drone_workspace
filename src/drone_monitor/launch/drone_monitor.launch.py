from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='drone_sensors',
            executable='temp_sensor_node',
            name='temp_sensor_node',
            output='screen',
            parameters=[{
                'simulate': True,
            }]
        ),

        Node(
            package='drone_sensors',
            executable='sound_sensor_node',
            name='sound_sensor_node',
            output='screen',
            parameters=[{
                'simulate': True,
            }]
        ),

        Node(
            package='drone_monitor',
            executable='env_monitor_node',
            name='env_monitor_node',
            output='screen',
            parameters=[{
                'temp_threshold': 35.0,
                'sound_threshold': 80.0,
                'alert_cooldown': 30.0,
            }]
        ),

        Node(
            package='drone_monitor',
            executable='alert_manager_node',
            name='alert_manager_node',
            output='screen',
            parameters=[{
                'smtp_user': 'thedailyscoop7777777@gmail.com',
                'smtp_pass': 'wixq wmip xvdu pocp',
                'alert_email': 'thedailyscoop7777777@gmail.com',
            }]
        ),

        Node(
            package='drone_navigation',
            executable='battery_manager_node',
            name='battery_manager_node',
            output='screen',
            parameters=[{
                'recharge_threshold': 20.0,
                'dock_x': 0.0,
                'dock_y': 0.0,
                'dock_z': 0.0,
            }]
        ),

        Node(
            package='drone_navigation',
            executable='patrol_planner_node',
            name='patrol_planner_node',
            output='screen',
            parameters=[{
                'waypoints': [
                    1.0, 0.0, 1.2,
                    1.0, 2.0, 1.2,
                    3.0, 2.0, 1.2,
                    3.0, 0.0, 1.2,
                ],
            }]
        ),

    ])
