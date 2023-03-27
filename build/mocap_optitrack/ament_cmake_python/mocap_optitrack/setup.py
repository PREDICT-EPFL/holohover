from setuptools import find_packages
from setuptools import setup

setup(
    name='mocap_optitrack',
    version='0.1.2',
    packages=find_packages(
        include=('mocap_optitrack', 'mocap_optitrack.*')),
)
