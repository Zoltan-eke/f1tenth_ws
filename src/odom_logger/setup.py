from glob import glob
from setuptools import find_packages, setup

package_name = 'odom_logger'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
            # package.xml telepítése
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         # launch fájlok telepítése
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ezo',
    maintainer_email='zoltaneke@icloud.com',
    description='ODOM logger package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'save_odom = odom_logger.save_odom:main',
        ],
    },
)
