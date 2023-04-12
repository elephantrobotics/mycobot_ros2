from setuptools import setup

import os
from glob import glob
package_name = 'ultraarm'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         # launch 文件路径
        (os.path.join('share', package_name, "launch"), glob('launch/*.launch.py')),
        # python 文件
        # (os.path.join('lib',package_name),glob(package_name+'/*.py')),
        # 配置文件
        (os.path.join('share', package_name, "config"), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='u20',
    maintainer_email='842749351@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'listen_real_of_topic = ultraarm.listen_real_of_topic:main',
            'listen_real = ultraarm.listen_real:main',
            'simple_gui = ultraarm.simple_gui:main',
            'slider_control = ultraarm.slider_control:main',
            'teleop_keyboard = ultraarm.teleop_keyboard:main',
        ],
    },
)
