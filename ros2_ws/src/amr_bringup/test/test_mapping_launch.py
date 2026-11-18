"""Validate launch outputs against installed assets without starting Gazebo."""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
import yaml


@pytest.mark.parametrize('gui,rviz', [('true', 'true'), ('false', 'false')])
def test_mapping_launch_connects_installed_assets(gui, rviz):
    path = Path(__file__).parents[1] / 'launch/mapping.launch.py'
    spec = importlib.util.spec_from_file_location('mapping_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    args = [a for a in description.entities if isinstance(a, DeclareLaunchArgument)]
    assert {a.name for a in args} == {'gui', 'rviz', 'hardware_config'}
    context = LaunchContext()
    context.launch_configurations.update(gui=gui, rviz=rviz)
    context.launch_configurations['hardware_config'] = str(
        Path(__file__).parents[1] / 'config/hardware.yaml')
    actions = module.launch_setup(context)
    includes = [a for a in actions if isinstance(a, IncludeLaunchDescription)]
    gz_args = dict(includes[0].launch_arguments)['gz_args']
    assert ('--headless-rendering' in gz_args) == (gui == 'false')
    assert Path(gz_args.split()[-1]).is_file()
    slam_args = dict(includes[1].launch_arguments)
    config = yaml.safe_load(Path(slam_args['slam_params_file']).read_text())
    assert config['slam_toolbox']['ros__parameters']['base_frame'] == 'base_link'
    assert slam_args['use_sim_time'] == 'true'
    nodes = [a for a in actions if isinstance(a, Node)]
    rviz_node = nodes[-1]
    assert rviz_node.condition.evaluate(context) == (rviz == 'true')
    # Expand the same installed geometry that launch passes to state publishing.
    robot = ET.fromstring(module.xacro.process_file(str(Path(
        get_package_share_directory('amr_description')) / 'urdf/amr.urdf.xacro')).toxml())
    links = {link.attrib['name'] for link in robot.findall('link')}
    assert {'base_link', 'lidar_link', 'left_wheel_link', 'right_wheel_link'} <= links
