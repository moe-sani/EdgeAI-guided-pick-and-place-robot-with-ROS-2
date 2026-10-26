"""Real SO-101 cell: servo driver, robot model and the overhead camera.

  ros2 launch so101_cell real.launch.py [port:=/dev/ttyACM0] [calibration:=<LeRobot JSON>] [camera:=0]
  ros2 launch so101_cell real.launch.py port:=fake camera:=none    # no hardware, for trying things out

Provides the same ROS interface as sim.launch.py:
  /arm_controller/follow_joint_trajectory, /gripper_controller/follow_joint_trajectory, /joint_states,
  /object_pose/<Name> (from the camera), /safety_camera/image.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from so101_cell.cell_camera import DEFAULT_TABLE_CALIBRATION
from so101_cell.driver import DEFAULT_CALIBRATION


def generate_launch_description():
    share = get_package_share_directory('so101_cell')
    description = ParameterValue(
        Command(['xacro ', os.path.join(share, 'urdf', 'so101.urdf.xacro'), ' sim:=false']), value_type=str)
    camera = LaunchConfiguration('camera')
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0', description='servo board serial port, or "fake"'),
        DeclareLaunchArgument('calibration', default_value=DEFAULT_CALIBRATION, description='LeRobot calibration JSON'),
        DeclareLaunchArgument('camera', default_value='0', description='overhead camera index/path, or "none"'),
        DeclareLaunchArgument('table_calibration', default_value=DEFAULT_TABLE_CALIBRATION),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': description}]),
        Node(package='so101_cell', executable='so101_driver', output='screen',
             parameters=[{'port': LaunchConfiguration('port'), 'calibration': LaunchConfiguration('calibration')}]),
        Node(package='so101_cell', executable='cell_camera', output='screen',
             condition=UnlessCondition(PythonExpression(["'", camera, "' == 'none'"])),
             arguments=['--device', camera, '--table-calibration', LaunchConfiguration('table_calibration')]),
    ])
