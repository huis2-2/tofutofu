from glob import glob

from setuptools import find_packages, setup

package_name = 'autorace_yolo'

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
    description='YOLO-seg TensorRT inference: lane class mask + sign/light detections',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_node = autorace_yolo.node:main',
        ],
    },
)
