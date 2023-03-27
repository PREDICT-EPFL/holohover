from setuptools import find_packages
from setuptools import setup

setup(
    name='rokubimini_msgs',
    version='0.6.1',
    packages=find_packages(
        include=('rokubimini_msgs', 'rokubimini_msgs.*')),
)
