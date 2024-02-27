import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    camera_config_dir = os.path.join(get_package_share_directory('rift_bringup'), 'config')
    usb_cam_1 = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name="camera_1",
        namespace="camera_1",
        parameters=[os.path.join(camera_config_dir, 'camera_1_param.yaml')],
    )
    usb_cam_2 = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name="camera_2",
        namespace="camera_2",
        parameters=[os.path.join(camera_config_dir, 'camera_2_param.yaml')],
    )
    usb_cam_3 = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name="camera_3",
        namespace="camera_3",
        parameters=[os.path.join(camera_config_dir, 'camera_3_param.yaml')],
    )


    return LaunchDescription([
        usb_cam_1,
        usb_cam_2,
        usb_cam_3
    ])