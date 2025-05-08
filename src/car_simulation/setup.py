import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'car_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/sim_log', []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ezo',
    maintainer_email='zoltaneke@icloud.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spawn_leader_vehicle = car_simulation.spawn_leader_vehicle:main',
            'leader_position_publisher = car_simulation.leader_position_publisher:main',
            'vehicle_model_node = car_simulation.vehicle_model_node:main',
            'simulation_logger = car_simulation.simulation_logger:main',
            'open_loop_cmd_publisher = car_simulation.open_loop_cmd_publisher:main',
        ],
    },
)
