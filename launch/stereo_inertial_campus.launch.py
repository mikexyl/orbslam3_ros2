import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration

# fmt: off

def launch_setup(context, *args, **kwargs):
  orb_slam3_node = Node(
    package='orbslam3_ros2',
    executable='stereo-inertial',
    name='orbslam3_stereo_inertial_node',
    output='screen',
    arguments=[get_package_share_directory('orbslam3_ros2') + '/vocab/ORBvoc.txt',
               get_package_share_directory('orbslam3_ros2') + '/config/RealSense_D455.yaml',
               '0'],
    remappings=[
      ('camera/left', LaunchConfiguration("camera_name").perform(context) + '/infra1/image_rect_raw'),
      ('camera/right', LaunchConfiguration("camera_name").perform(context) + '/infra2/image_rect_raw'),
      ('imu', LaunchConfiguration("camera_name").perform(context) + '/imu'),
    ]
  )

  bag_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([get_package_share_directory('aria_data_utils'), '/launch/play_bag_campus.launch.py']),
    launch_arguments={
      "bag_file": LaunchConfiguration("bag_file"),
      "camera_name": LaunchConfiguration("camera_name"),
    }.items()
  )

  return [orb_slam3_node, bag_launch]


def generate_launch_description():
    bag_file_arg = launch.actions.DeclareLaunchArgument( "bag_file", default_value="/datasets/campus-x/outdoor/10_14_hathor", description="bag file",)
    camera_name_arg=launch.actions.DeclareLaunchArgument( "camera_name", default_value="/hathor/forward", description="Camera name",)

    return LaunchDescription(
        [
            bag_file_arg,
            camera_name_arg,
            launch.actions.OpaqueFunction(function=launch_setup),
        ]
    )
