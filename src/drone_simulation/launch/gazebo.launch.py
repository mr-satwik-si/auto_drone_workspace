import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    world_file = os.path.join(
        get_package_share_directory('drone_simulation'),
        'worlds', 'room.sdf'
    )

    return LaunchDescription([

        # Launch Gazebo with the room world
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_file],
            output='screen'
        ),

        # Bridge Gazebo topics to ROS 2
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_ros_bridge',
            output='screen',
            arguments=[
                '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/drone/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/model/drone/pose@geometry_msgs/msg/PoseStamped@gz.msgs.Pose',
            ]
        ),

        # Sensor nodes
        Node(
            package='drone_sensors',
            executable='temp_sensor_node',
            name='temp_sensor_node',
            output='screen',
            parameters=[{'simulate': True}]
        ),

        Node(
            package='drone_sensors',
            executable='sound_sensor_node',
            name='sound_sensor_node',
            output='screen',
            parameters=[{'simulate': True}]
        ),

        # Monitor nodes
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
                'smtp_user': '',
                'smtp_pass': '',
                'alert_email': '',
            }]
        ),

        # Navigation nodes
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
