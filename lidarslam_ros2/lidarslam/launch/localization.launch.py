import os

import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument          
from launch.substitutions import LaunchConfiguration     
from launch_ros.actions import SetParameter   

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    main_param_dir = launch.substitutions.LaunchConfiguration(
        'main_param_dir',
        default=os.path.join(
            get_package_share_directory('lidarslam'),
            'param',
            'lidarslam.yaml'))
    
    rviz_param_dir = launch.substitutions.LaunchConfiguration(
        'rviz_param_dir',
        default=os.path.join(
            get_package_share_directory('lidarslam'),
            'rviz',
            'localization.rviz'))

    # use_sim_time_arg = DeclareLaunchArgument(
    #     'use_sim_time',
    #     default_value='false',
    #     description='Use /clock(simulation time)'
    # )

    # set_sim_time = SetParameter(
    #     name='use_sim_time',
    #     value=LaunchConfiguration('use_sim_time')
    # )

    mapping = launch_ros.actions.Node(
        package='scanmatcher',
        executable='scanmatcher_node',
        parameters=[main_param_dir],
        remappings=[('/input_cloud','/livox/lidar')],
        output='screen'
        )

    tf = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.27','0','0.07','0','0','0.7171','0.7171','base_link','livox_frame']
        )


    # graphbasedslam = launch_ros.actions.Node(
    #     package='graph_based_slam',
    #     executable='graph_based_slam_node',
    #     parameters=[main_param_dir],
    #     output='screen'
    #     )
    
    rviz = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_param_dir]
        )


    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'main_param_dir',
            default_value=main_param_dir,
            description='Full path to main parameter file to load'),
      #  use_sim_time_arg,
      #  set_sim_time,
        mapping,
        tf,
        # graphbasedslam,
        rviz,
            ])