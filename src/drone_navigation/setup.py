from setuptools import setup

package_name = 'drone_navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='stack',
    maintainer_email='stack@todo.todo',
    description='Drone navigation and battery management nodes',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'battery_manager_node = drone_navigation.battery_manager_node:main',
            'patrol_planner_node  = drone_navigation.patrol_planner_node:main',
        ],
    },
)
