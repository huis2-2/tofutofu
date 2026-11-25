from glob import glob

from setuptools import find_packages, setup

package_name = 'autorace_signal'

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
    description='Traffic light: stop at the line on red/yellow, go on green. Keeps lane steering',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'signal_node = autorace_signal.node:main',
        ],
    },
)
