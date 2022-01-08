import os
from setuptools import setup, glob

package_name = 'holohover_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'gui'), glob.glob('gui/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rschwan',
    maintainer_email='roland.schwan@epfl.ch',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rviz_interface = holohover_utils.rviz_interface:main',
        ],
    },
)
