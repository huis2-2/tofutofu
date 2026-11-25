from glob import glob

from setuptools import find_packages, setup

package_name = 'autorace_lane'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tofu',
    maintainer_email='tofu@todo.todo',
    description=(
        'Lane mask -> ground(BEV) -> center path, Pure Pursuit lane command (/mission/lane), A/B fork'),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_node = autorace_lane.lane_node:main',
            'controller_node = autorace_lane.controller_node:main',
        ],
    },
)
