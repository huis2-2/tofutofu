from glob import glob

from setuptools import setup

package_name = 'autorace_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
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
    description='Launch everything (sensors, perception, missions, mux, drive) + rosbag recording',
    license='Apache-2.0',
    tests_require=['pytest'],
)
