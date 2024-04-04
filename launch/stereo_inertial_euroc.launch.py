from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration

# fmt: off

def launch_setup(context, *args, **kwargs):
  orb_slam3_node = Node(
    package='orbslam3_ros2',
    executable='stereo-inertial',
    name='orbslam3_stereo_inertial_node',
    output='screen',
    arguments=[get_package_share_directory('orbslam3_ros2') + '/vocab/ORBvoc.txt',
               get_package_share_directory('orbslam3_ros2') + '/config/EuRoC.yaml',
               '1'], # 1 to do recfity
    remappings=[
      ('camera/left', '/cam0/image_raw'),
      ('camera/right', '/cam1/image_raw'),
      ('imu', '/imu0'),
    ]
  )

  bag_player=ExecuteProcess(
    cmd=['ros2', 'bag', 'play', LaunchConfiguration("bag_file"), '--clock'],
    output='screen',
  )

  return [orb_slam3_node, bag_player]


def generate_launch_description():
    bag_file_arg = launch.actions.DeclareLaunchArgument( "bag_file", default_value="/datasets/euroc/MH_03_medium", description="bag file",)

    return LaunchDescription(
        [
            bag_file_arg,
            launch.actions.OpaqueFunction(function=launch_setup),
        ]
    )
