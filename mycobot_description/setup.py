from setuptools import setup

import os

from glob import glob

package_name = 'mycobot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf'+'/260_pi', glob("urdf/260_pi/*")),
        ('share/' + package_name + '/urdf'+'/280jn', glob("urdf/280jn/*")),
        ('share/' + package_name + '/urdf'+'/320_urdf', glob("urdf/320_urdf/*")),
        ('share/' + package_name + '/urdf'+'/600_urdf', glob("urdf/600_urdf/*")),
        ('share/' + package_name + '/urdf'+'/mycobot', glob("urdf/mycobot/*")),
        ('share/' + package_name + '/urdf'+'/mycobot_pi', glob("urdf/mycobot_pi/*")),
        ('share/' + package_name + '/urdf'+'/mecharm', glob("urdf/mecharm/*")),

        ('share/' + package_name + '/urdf'+'/mecharm_pi', glob("urdf/mecharm_pi/*")),

        ('share/' + package_name + '/urdf'+'/320_pi',glob("urdf/320_pi/*")),

        ('share/' + package_name + '/urdf'+'/mybuddy', glob("urdf/mybuddy/*")),
        
        ('share/' + package_name + '/urdf'+'/ultraArm_urdf', glob("urdf/ultraArm_urdf/*")),
        ('share/' + package_name + '/urdf'+'/mycobot_x3pi', glob("urdf/mycobot_x3pi/*")),
        ('share/' + package_name + '/urdf'+'/260_x3pi', glob("urdf/260_x3pi/*")),


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='u2',
    maintainer_email='u2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
