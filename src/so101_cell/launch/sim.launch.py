"""SO-101 cell in Gazebo Harmonic: the arm with ros2_control trajectory controllers, plus object pose bridging.

  ros2 launch so101_cell sim.launch.py [gui:=true|false]

Provides the same ROS interface as real.launch.py:
  /arm_controller/follow_joint_trajectory, /gripper_controller/follow_joint_trajectory, /joint_states,
  /object_pose/<Name> (geometry_msgs/PoseStamped, frame "world").
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('so101_cell')
    world = os.path.join(share, 'worlds', 'so101_cell.sdf')
    gui = LaunchConfiguration('gui')
    description = ParameterValue(
        Command(['xacro ', os.path.join(share, 'urdf', 'so101.urdf.xacro'), ' sim:=true']), value_type=str)

    gz = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    gz_with_gui = IncludeLaunchDescription(PythonLaunchDescriptionSource(gz), condition=IfCondition(gui),
                                           launch_arguments={'gz_args': f'-r {world}', 'on_exit_shutdown': 'true'}.items())
    gz_headless = IncludeLaunchDescription(PythonLaunchDescriptionSource(gz), condition=UnlessCondition(gui),
                                           launch_arguments={'gz_args': f'-r -s --headless-rendering {world}',
                                                             'on_exit_shutdown': 'true'}.items())

    spawn_robot = Node(package='ros_gz_sim', executable='create', output='screen',
                       arguments=['-topic', 'robot_description', '-name', 'so101', '-z', '0.0'])

    def spawner(name, after):
        return RegisterEventHandler(OnProcessExit(target_action=after, on_exit=[Node(
            package='controller_manager', executable='spawner', output='screen',
            arguments=[name, '--controller-manager-timeout', '60'])]))

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='open the Gazebo window'),
        # Lets Gazebo find the meshes referenced as file:// URIs and anything under share/so101_cell.
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                               os.path.dirname(share) + os.pathsep + os.environ.get('GZ_SIM_RESOURCE_PATH', '')),
        gz_with_gui,
        gz_headless,
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': description, 'use_sim_time': True}]),
        spawn_robot,
        spawner('joint_state_broadcaster', spawn_robot),
        spawner('arm_controller', spawn_robot),
        spawner('gripper_controller', spawn_robot),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
             parameters=[{'use_sim_time': True}]),
        Node(package='so101_cell', executable='sim_object_poses', output='screen',
             parameters=[{'use_sim_time': True}]),
    ])
