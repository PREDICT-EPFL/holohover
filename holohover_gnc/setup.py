from setuptools import setup, find_packages

package_name = 'holohover_gnc'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=package_name + '*'),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
        'qpsolvers',
    ],
    zip_safe=True,
    maintainer='karim',
    maintainer_email='karimsamaha98@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera = holohover_gnc.camera:main',
            'simulator = holohover_gnc.simulator:main',
            'estimator = holohover_gnc.estimator:main',
            'controller = holohover_gnc.controller:main'
        ],
    },
)
