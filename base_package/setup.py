from setuptools import setup
from glob import glob
import os
package_name = 'base_package'
submodules = "base_package/helpers"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    package_data={
        '': ['*.csv'],
    },
    install_requires=[
        'setuptools',
        'numpy',
        'pandas',
        'opencv-python',
        'opencv-contrib-python',
    ],
    zip_safe=True,
    maintainer='karim',
    maintainer_email='karimsamaha98@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera = base_package.camera:main',
            'simulator = base_package.simulator:main',
            'estimator = base_package.estimator:main',
            'controller = base_package.controller:main'
        ],
    },
)
