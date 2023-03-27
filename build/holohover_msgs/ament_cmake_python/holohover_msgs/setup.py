from setuptools import find_packages
from setuptools import setup

setup(
    name='holohover_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('holohover_msgs', 'holohover_msgs.*')),
)
