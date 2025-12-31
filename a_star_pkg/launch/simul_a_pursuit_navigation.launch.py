import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # initialize directory
    pkg_dir = get_package_share_directory('a_star_pkg')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    
    # initialize parameter
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    ap_config = PathJoinSubstitution([pkg_dir, 'config', 'inte-navi-params.yaml'])

    # Gazebo Node
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'turtlebot3_custom_maze3.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Localization (AMCL)
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml')
        }.items()
    )

    # RViz Node
    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # integrated_navigation Node
    inte_navi_node = Node(
        package='a_star_pkg',
        executable='navigation',
        name='integrated_navigation',
        parameters=[ap_config, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value='/home/ysn0630/maps/maze3.yaml',
            description='Full path to map yaml file to load'),
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true', # set 'true' in simulation
            description='Use simulation (Gazebo) clock if true'),

        gazebo_launch,
        localization_launch,
        rviz_node,
        inte_navi_node
    ])