import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

NAMESPACE = os.environ.get('ROS_NAMESPACE') if 'ROS_NAMESPACE' in os.environ else 'default'

def generate_launch_description():
    bringup_path = get_package_share_directory("edna_bringup")
    joystick_file = os.path.join(bringup_path, 'config', 'xbox-sim.yaml')
    rviz_file = os.path.join(bringup_path, 'config', 'view.rviz')
    
    common = { 'use_sim_time': 'true', 'namespace': NAMESPACE }
    
    isaac_layer = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    bringup_path,'launch','isaac.launch.py'
                )]))
    
    rtab_layer = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                bringup_path,'launch','rtab.launch.py'
            )]))
    
    delay_rtab_layer = TimerAction(period=8.0, actions=[rtab_layer])
    
    pysim_layer = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                bringup_path,'launch','isaac_pysim.launch.py'
            )]))
    
    rviz = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                bringup_path,'launch','rviz.launch.py'
            )]))

    # Launch!
    return LaunchDescription([
        isaac_layer,
        pysim_layer,
        # rviz,
        delay_rtab_layer,
    ])
