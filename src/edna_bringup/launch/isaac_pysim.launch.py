import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

NAMESPACE = os.environ.get('ROS_NAMESPACE') if 'ROS_NAMESPACE' in os.environ else 'default'

def generate_launch_description():
    bringup_path = get_package_share_directory("edna_bringup")
    rviz_file = os.path.join(bringup_path, 'config', 'view.rviz')
    
    common = { 'use_sim_time': 'true', 'namespace': NAMESPACE }
    
    debug_launch_args = common | {
        'enable_rviz': 'false',
        'enable_foxglove': 'false',
        'rviz_file': rviz_file
    }
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        namespace=NAMESPACE,
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", ['/', NAMESPACE, "/controller_manager"]],
    )
    
    isaac_hadware_test = Node(
        package='isaac_hardware_test',
        namespace=NAMESPACE,
        executable='isaac_drive',
        parameters=[{}],
    )
    
    debug_layer = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    bringup_path,'launch','debugLayer.launch.py'
                )]), launch_arguments=debug_launch_args.items())
    delay_debug_layer =  TimerAction(period=3.0, actions=[debug_layer])

    # Launch!
    return LaunchDescription([
        # joint_state_broadcaster_spawner,
        isaac_hadware_test,
        delay_debug_layer,
    ])
