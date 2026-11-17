"""A rear caster requires the mass centre to remain behind the drive axle."""
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import xacro


def test_default_mass_centre_is_inside_support_triangle():
    robot = ET.fromstring(xacro.process_file(str(Path(
        get_package_share_directory('amr_description')) / 'urdf/amr.urdf.xacro')).toxml())
    joints = {j.find('child').attrib['link']: j for j in robot.findall('joint')}
    total_mass = 0.0
    moment_x = 0.0
    for link in robot.findall('link'):
        inertial = link.find('inertial')
        mass = float(inertial.find('mass').attrib['value'])
        origin = inertial.find('origin')
        local_x = float(origin.attrib['xyz'].split()[0]) if origin is not None else 0.0
        joint = joints.get(link.attrib['name'])
        joint_x = float(joint.find('origin').attrib['xyz'].split()[0]) if joint is not None else 0.0
        total_mass += mass
        moment_x += mass * (joint_x + local_x)
    com_x = moment_x / total_mass
    caster_x = float(joints['caster_link'].find('origin').attrib['xyz'].split()[0])
    # 15 mm static margin to either tipping edge at y=0, not merely on an edge.
    assert caster_x + 0.015 < com_x < -0.015, f'Unstable COM x={com_x:.4f}'
