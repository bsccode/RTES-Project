# Filename: full_robot_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    
    # Declare the path to your package folders
    package_path = os.getenv('ROS_PACKAGE_PATH', '').split(':')[0]

    # Declare the launch arguments
    ld = LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation time if true'
        )
    ])

    # Launching ROSBOT simulation using rosbot_gazebo launch file
    rosbot_sim_launch = IncludeLaunchDescription(
    	PythonLaunchDescriptionSource(
        	'/home/ros/ros2_ws/install/rosbot_gazebo/share/rosbot_gazebo/launch/simulation.launch.py'
    	),
    	launch_arguments={'use_sim_time': 'True'}.items()
)


    # Other nodes and launches
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    roaming2 = Node(
        package='command',
        executable='roaming2',
        name='roaming2',
        output='screen'
    )

    ball_info2 = Node(
        package='command',
        executable='ball_info2',
        name='ball_info2',
        output='screen'
    )

    voice_parse = Node(
        package='command',
        executable='voice_parse',
        name='voice_parse',
        output='screen'
    )

    ball_location = Node(
        package='command',
        executable='ball_location',
        name='ball_location',
        output='screen'
    )

    ball_color_detector = Node(
        package='ball_detection_pkg',
        executable='ball_color_detector',
        name='ball_color_detector',
        output='screen'
    )

    text_to_speech = Node(
        package='text_to_speech_pkg',
        executable='text_to_speech_node',
        name='text_to_speech_node',
        output='screen'
    )

    result_to_tts = Node(
        package='text_to_speech_pkg',
        executable='result_to_tts',
        name='result_to_tts',
        output='screen'
    )

    vosk = Node(
        package='voskros',
        executable='vosk',
        name='vosk',
        output='screen'
    )

    # Including SLAM and navigation launch files
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            '/home/ros/ros2_ws/install/tutorial_pkg/share/tutorial_pkg/launch/slam.launch.py'
        ),
        launch_arguments={'use_sim_time': 'True'}.items()
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            '/home/ros/ros2_ws/install/tutorial_pkg/share/tutorial_pkg/launch/navigation.launch.py'
        ),
        launch_arguments={'use_sim_time': 'True'}.items()
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': 'map.yaml', 'use_sim_time': use_sim_time}]
    )

    robot_control = Node(
        package='robot_control_pkg',
        executable='robot_control_node',
        name='robot_control_node',
        output='screen'
    )

    # Add all actions to launch description
    ld.add_action(rosbot_sim_launch)
    ld.add_action(rviz)
    ld.add_action(roaming2)
    ld.add_action(ball_info2)
    ld.add_action(voice_parse)
    ld.add_action(ball_location)
    ld.add_action(ball_color_detector)
    ld.add_action(text_to_speech)
    ld.add_action(result_to_tts)
    ld.add_action(vosk)
    ld.add_action(slam_launch)
    ld.add_action(nav2_launch)
    ld.add_action(map_server)
    ld.add_action(robot_control)

    return ld

