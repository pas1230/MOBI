import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction, IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown
from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                LaunchConfiguration, LocalSubstitution,
                                PythonExpression)
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='sam_bot_description').find('sam_bot_description')
    default_model_path = os.path.join(pkg_share, 'src/description/sam_bot_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    world_path=os.path.join(pkg_share, 'world/my_world.sdf')
    
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    spawn_entity = launch_ros.actions.Node(
    	package='gazebo_ros', 
    	executable='spawn_entity.py',
        arguments=['-entity', 'sam_bot', '-topic', 'robot_description'],
        output='screen'
    )
    robot_localization_node = launch_ros.actions.Node(
         package='robot_localization',
         executable='ekf_node',
         name='ekf_filter_node',
         output='screen',
         parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),
        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_entity,
        robot_localization_node,
        rviz_node,
        launch.actions.RegisterEventHandler(
            OnProcessStart(
                target_action = spawn_entity,
                on_start=[
                    LogInfo(msg='gazebo started'),
                    TimerAction(
                        period=10.0,
                        actions=[
                            IncludeLaunchDescription(
                                PythonLaunchDescriptionSource(
                                    os.path.join(get_package_share_directory('slam_toolbox'),
                                                "launch",
                                                "online_async_launch.py")
                                )
                            )
                        ]       
                    )
                ]
            ) 
        ),
        launch.actions.RegisterEventHandler(
            OnProcessStart(
                target_action = spawn_entity,
                on_start=[
                    LogInfo(msg='gazebo started'),
                    TimerAction(
                        period=15.0,
                        actions=[
                            IncludeLaunchDescription(
                                PythonLaunchDescriptionSource(
                                    os.path.join(get_package_share_directory('nav2_bringup'),
                                                "launch",
                                                "navigation_launch.py")
                                )
                            ),
                        ]       
                    )
                ]
            ) 
        ),
    ])