import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition

# Easy use of namespace since args are not strings
# NAMESPACE = os.environ.get('ROS_NAMESPACE') if 'ROS_NAMESPACE' in os.environ else 'default'

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    policy = Node(
        package='policy_runner',
        executable='runner',
        name='policy_runner_node',
        parameters=[{
            'use_sim_time': use_sim_time,
            'odom_topic': '/saranga/zed/odom',
            'target_topic': '/real/obj_det_pose',
        }]
    )
    
    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        policy
    ])