from setuptools import setup
import os
from glob import glob

package_name = 'drone_monitor'

setup(
    name=package_name,
    version='0.0.1',
    packages=[
        package_name,
        package_name + '.dashboard',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'dashboard', 'templates'),
            glob('drone_monitor/dashboard/templates/*')),
        (os.path.join('share', package_name, 'dashboard', 'static'),
            glob('drone_monitor/dashboard/static/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='stack',
    maintainer_email='stack@todo.todo',
    description='Drone environment monitor and alert nodes',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'env_monitor_node      = drone_monitor.env_monitor_node:main',
            'alert_manager_node    = drone_monitor.alert_manager_node:main',
            'fire_detection_node   = drone_monitor.fire_detection_node:main',
            'dashboard_node        = drone_monitor.dashboard_node:main',
            
        ],
    },
)