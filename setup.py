import os

from setuptools import setup

# read version from surgym/version.txt
with open(os.path.join(os.path.dirname(__file__), "surgym", "version.txt")) as f:
    version = f.read()

setup(
    name='surgym',
    version=version,
    description='Reinforcement Learning for Surgical Robotics in Isaac Sim',
    author='ksterx',
)
