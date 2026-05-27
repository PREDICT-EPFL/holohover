import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'holohover_dial'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'example_params.yaml', 'example_config.yaml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='till',
    maintainer_email='till.beyer@epfl.ch',
    description='Diffusion Inspired Annealing (DIAL) for Holohover control and Puck shooting',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # Path planner node: subscribes to state, publishes path plan
            'path_planner=holohover_dial.path_planner_node:main',
            
            # Control node: subscribes to state and path plan, publishes control
            'control=holohover_dial.control_node:main',
        ],
    },
)
