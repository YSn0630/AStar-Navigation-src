import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('a_star_pkg')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_file = LaunchConfiguration('map', default='/home/ysn0630/maps/map.yaml')
    ap_params = PathJoinSubstitution([pkg_dir, 'config', 'inte-navi-params.yaml'])
    yolo_params = PathJoinSubstitution([pkg_dir, 'config', 'yolo_params.yaml'])

    localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml')
        }.items()
    )

    ap_node = Node(
        package='a_star_pkg',
        executable='navigation',
        name='integrated_navigation',
        parameters=[ap_params, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    image_node = Node(
        package='a_star_pkg',
        executable='yolo_detector',
        name='waffle_v4l2',
        parameters=[yolo_params, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    rviz_config = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    rviz_cmd = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false'))
    ld.add_action(DeclareLaunchArgument('map', default_value=map_file))

    ld.add_action(localization_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(ap_node)
    ld.add_action(image_node)

    return ld