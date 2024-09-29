# roaming_with_detection.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare the slam_params_file and use_sim_time launch arguments
    slam_params_file = LaunchConfiguration('slam_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Define the default path to the slam.yaml file
    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare('tutorial_pkg'), 'config', 'slam.yaml']
        ),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node'
    )

    # Declare the use_sim_time argument
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation/Gazebo clock'
    )

    return LaunchDescription([
        slam_params_file_arg,
        use_sim_time_arg,
        # Start SLAM Toolbox with odometry topic remapping
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam',
            output='screen',
            parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
            remappings=[
                ('/odom', '/rosbot_base_controller/odom'),
            ],
        ),
        # Start Robot Control Node
        Node(
            package='robot_control_pkg',
            executable='robot_control_node',
            name='robot_control_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        # Start Object Detection Node
        Node(
            package='object_detection_pkg',
            executable='object_detection_node',
            name='object_detection_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        # Start Text-to-Speech Node
        Node(
            package='text_to_speech_pkg',
            executable='text_to_speech_node',
            name='text_to_speech_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        # Start Vosk Node (if needed)
        Node(
            package='voskros',
            executable='vosk',
            name='vosk',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        # Start Command Processor Node (if needed)
        Node(
            package='command_processor_pkg',
            executable='command_processor_node',
            name='command_processor_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
