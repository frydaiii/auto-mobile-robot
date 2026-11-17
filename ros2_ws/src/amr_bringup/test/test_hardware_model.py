"""Validate converted simulator assets, including URDF collision renaming."""
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import xacro


def test_converted_contact_sensors_reference_real_collisions(tmp_path):
    urdf = tmp_path / 'robot.urdf'
    urdf.write_text(xacro.process_file(str(Path(get_package_share_directory(
        'amr_description')) / 'urdf/amr.urdf.xacro')).toxml())
    result = subprocess.run(['gz', 'sdf', '-p', str(urdf)], capture_output=True,
                            text=True, check=True, timeout=15)
    robot = ET.fromstring(result.stdout)
    for side in ('left', 'center', 'right'):
        link = robot.find(f".//link[@name='bumper_{side}_link']")
        collision_names = {c.get('name') for c in link.findall('collision')}
        assert link.findtext('sensor/contact/collision') in collision_names
