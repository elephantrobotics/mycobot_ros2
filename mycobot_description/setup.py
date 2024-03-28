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
        ('share/' + package_name + '/urdf'+'/mypalletizer_260_pi', glob("urdf/mypalletizer_260_pi/*")),
        ('share/' + package_name + '/urdf'+'/mycobot_280_jn', glob("urdf/mycobot_280_jn/*")),
        ('share/' + package_name + '/urdf'+'/mycobot_320_m5_2022', glob("urdf/mycobot_320_m5_2022/*")),
        ('share/' + package_name + '/urdf'+'/mycobot_pro_600', glob("urdf/mycobot_pro_600/*")),
        ('share/' + package_name + '/urdf'+'/mycobot_280_m5', glob("urdf/mycobot_280_m5/*")),
        ('share/' + package_name + '/urdf'+'/mycobot_280_pi', glob("urdf/mycobot_280_pi/*")),
        ('share/' + package_name + '/urdf'+'/mecharm_270_m5', glob("urdf/mecharm_270_m5/*")),

        ('share/' + package_name + '/urdf'+'/mecharm_270_pi', glob("urdf/mecharm_270_pi/*")),

        ('share/' + package_name + '/urdf'+'/mycobot_320_pi_2022',glob("urdf/mycobot_320_pi_2022/*")),

        ('share/' + package_name + '/urdf'+'/mybuddy', glob("urdf/mybuddy/*")),
        
        ('share/' + package_name + '/urdf'+'/ultraArm_p340', glob("urdf/ultraArm_p340/*")),
        ('share/' + package_name + '/urdf'+'/mycobot_280_x3pi', glob("urdf/mycobot_280_x3pi/*")),
        ('share/' + package_name + '/urdf'+'/myarm_300_pi', glob("urdf/myarm_300_pi/*")),
        ('share/' + package_name + '/urdf'+'/mycobot_pro_630', glob("urdf/mycobot_pro_630/*")),



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
