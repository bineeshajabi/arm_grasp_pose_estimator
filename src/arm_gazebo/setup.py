from setuptools import find_packages, setup
from glob import glob
import os

pkg_name = 'arm_gazebo'
pkg_description ='arm_description'

setup(
    name=pkg_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + pkg_name]),
        ('share/' + pkg_name, ['package.xml']),
        (os.path.join('share', pkg_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', pkg_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', pkg_name, 'world'), glob(os.path.join('world', '*.sdf')))
         
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bineesha',
    maintainer_email='bineeshajabi98@gmail.com',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest'
        ]
    },
    entry_points={
        'console_scripts': [
            'trajectory_publisher_server = arm_gazebo.trajectory_action_server:main'
        ]

    }
)
