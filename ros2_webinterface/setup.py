from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ros2_webinterface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.json')),
        (os.path.join('share', package_name, 'util'), glob('util/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Claire Schubert',
    maintainer_email='claire.schubert@sciota.io',
    description='Web UI for Ros2 iron',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'webinterface = ros2_webinterface.webinterface_node:main',
            'test_publischer = ros2_webinterface.test.test_publischer:main',
            'test_subscriber = ros2_webinterface.test.test_subscriber:main',
            'waypoint_manager = ros2_webinterface.waypoint_manager:main'
        ],
    },
)
