import os
import glob
from setuptools import setup

package_name = 'agl_odometry'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name,                ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', '*.py'))),
        # ('share/' + package_name + '/paths',  glob.glob(os.path.join('paths', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='david',
    maintainer_email='davidvalenciac01@gmail.com',
    description='AGL Odometry Package',
    license='Movvo Robotics 2023. All rights reserved',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odometry = agl_odometry.odometry:main'
        ],
    },
)