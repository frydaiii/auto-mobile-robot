"""Launch a self-contained manual driving and mapping simulation."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml


def launch_setup(context):
    share = lambda name: Path(get_package_share_directory(name))
    bringup = share('amr_bringup')
    simulation = share('amr_simulation')
    hardware = yaml.safe_load(Path(
        LaunchConfiguration('hardware_config').perform(context)).read_text())
    description = xacro.process_file(
        str(share('amr_description') / 'urdf/amr.urdf.xacro'),
        mappings={**{key: str(value) for key, value in hardware['robot'].items()},
                  'bumper_plugin_path': str(Path(get_package_prefix(
                      'amr_simulation')) / 'lib/libamr_bumper_state.so')}).toxml()
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    args = '-r ' + ('' if gui else '-s --headless-rendering ') + str(
        simulation / 'worlds/indoor.sdf')
    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            share('ros_gz_sim') / 'launch/gz_sim.launch.py')),
            launch_arguments={'gz_args': args}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description, 'use_sim_time': True}]),
        Node(package='ros_gz_sim', executable='create', arguments=[
            '-world', 'indoor', '-name', 'amr', '-topic', 'robot_description',
            '-x', '-2.0', '-y', '-2.0', '-z', '0.01'],
            parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             parameters=[{'config_file': str(simulation / 'config/bridge.yaml'),
                          'use_sim_time': True}], output='screen'),
        Node(package='amr_bringup', executable='command_gate',
             parameters=[hardware['command_gate']['ros__parameters'],
                         {key: float(hardware['robot'][key]) for key in (
                             'wheel_radius', 'wheel_separation', 'max_wheel_rpm')},
                         {'use_sim_time': True}], output='screen'),
        Node(package='amr_bringup', executable='hardware_feedback',
             parameters=[hardware['hardware_feedback']['ros__parameters'],
                         {'use_sim_time': True}], output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            share('slam_toolbox') / 'launch/online_async_launch.py')),
            launch_arguments={'use_sim_time': 'true',
                              'slam_params_file': str(bringup / 'config/slam.yaml')}.items()),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(bringup / 'config/mapping.rviz')],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('hardware_config', default_value=str(Path(
            get_package_share_directory('amr_bringup')) / 'config/hardware.yaml')),
        OpaqueFunction(function=launch_setup),
    ])
