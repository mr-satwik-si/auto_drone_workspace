from setuptools import setup

package_name = 'drone_sensors'

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
    description='Drone sensor driver nodes',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sound_sensor_node   = drone_sensors.sound_sensor_node:main',
            'temp_sensor_node    = drone_sensors.temp_sensor_node:main',
            'virtual_camera_node = drone_sensors.virtual_camera_node:main',
        ],
    },
)