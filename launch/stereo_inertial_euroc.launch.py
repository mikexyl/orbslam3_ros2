from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

# fmt: off

def launch_setup(context, *args, **kwargs):
  orb_slam3_node = Node(
    package='orbslam3_ros2',
    executable='stereo-inertial',
    name='orbslam3_stereo_inertial_node',
    output=LaunchConfiguration("output"),
    arguments=[get_package_share_directory('orbslam3_ros2') + '/vocab/ORBvoc.txt',
               get_package_share_directory('orbslam3_ros2') + '/config/EuRoC.yaml',
               'true'], # 1 to do recfity
    remappings=[
      ('camera/left', '/cam0/image_raw'),
      ('camera/right', '/cam1/image_raw'),
      ('imu', '/imu0'),
    ],
    # prefix='xterm -e gdb -ex run --args',
  )

  bag_player=ExecuteProcess(
    cmd=['ros2', 'bag', 'play', LaunchConfiguration("bag_file"), '--clock', '-r', '1', '--loop', '--start-offset', LaunchConfiguration("start")],
    output='screen',
  )

  rviz=Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', get_package_share_directory('orbslam3_ros2') + '/rviz/stereo_inertial.rviz'],
    condition=IfCondition(LaunchConfiguration("rviz")),
  )
  
  return [orb_slam3_node, bag_player, rviz]


def generate_launch_description():
    bag_file_arg = DeclareLaunchArgument( "bag_file", default_value="/datasets/euroc/MH_03_medium", description="bag file",)
    output_arg = DeclareLaunchArgument( "output", default_value="screen", description="output directory",)
    run_rviz_arg = DeclareLaunchArgument( "rviz", default_value="true", description="run rviz",)
    start_arg=DeclareLaunchArgument( "start", default_value="15", description="start time",)

    return LaunchDescription(
        [
            bag_file_arg,
            output_arg,
            run_rviz_arg,
            start_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
