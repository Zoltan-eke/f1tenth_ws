import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'data_logger'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
            # minden launch fájl telepítése
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        # a logged_file mappa üresen (majd itt jönnek a csv‑k)
        ('share/' + package_name + '/logged_file/raw', []),
        ('share/' + package_name + '/logged_file/filtered', []),
        ('share/' + package_name + '/logged_file/simulated', []),
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
            'save_odom = data_logger.save_odom:main',
            'save_imu = data_logger.save_imu:main',
            'save_joint_states = data_logger.save_joint_states:main',
            'save_ackermann_cmd = data_logger.save_ackermann_cmd:main',
            'save_tf = data_logger.save_tf:main',
        ],
    },
)
