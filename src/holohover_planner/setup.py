from setuptools import find_packages, setup

package_name = 'holohover_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alexis Bollack',
    maintainer_email='alexis.bollack@gmail.com',
    description='Planners of the holohover: the shot MPC planner, publishing HolohoverPlan for a controller',
    license='BSD-2-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'shot_planner=holohover_planner.shot_planner_node:main',
        ],
    },
)
