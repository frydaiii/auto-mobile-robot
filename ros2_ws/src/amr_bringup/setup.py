from glob import glob
from setuptools import setup

setup(
    name='amr_bringup', version='0.1.0', packages=['amr_bringup'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/amr_bringup']),
        ('share/amr_bringup', ['package.xml']),
        ('share/amr_bringup/launch', glob('launch/*.launch.py')),
        ('share/amr_bringup/config', glob('config/*')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='AMR contributors', maintainer_email='maintainers@example.com',
    description='Manual driving and mapping simulation bringup', license='Apache-2.0',
    entry_points={'console_scripts': [
        'command_gate = amr_bringup.command_gate_node:main',
    ]},
)
