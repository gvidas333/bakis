import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'room_exploration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml')) + glob(os.path.join('config', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gvidas',
    maintainer_email='gvidas.bac@gmail.com',
    description='ROS 2 package for TurtleBot3 room exploration algorithm comparison',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_node = room_exploration.main_node:main',
            'lds_driver_node = room_exploration.nodes.lds_driver_node:main',
        ],
    },
)
