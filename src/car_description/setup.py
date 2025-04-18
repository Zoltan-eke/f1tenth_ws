import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'car_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob(os.path.join('launch/*.py'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf/*.xacro'))),
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes/*.stl'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz/*.rviz'))),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ezo',
    maintainer_email='ezo@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)