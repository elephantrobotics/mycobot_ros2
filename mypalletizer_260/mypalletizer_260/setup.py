from setuptools import setup

import os
from glob import glob

package_name = 'mypalletizer_260'

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
    maintainer_email='u20@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_marker = mypalletizer_260.detect_marker:main',
            'follow_display = mypalletizer_260.follow_display:main',
            'following_marker = mypalletizer_260.following_marker:main',
            'listen_real_of_topic = mypalletizer_260.listen_real_of_topic:main',
            'listen_real = mypalletizer_260.listen_real:main',
            'simple_gui = mypalletizer_260.simple_gui:main',
            'slider_control = mypalletizer_260.slider_control:main',
            'teleop_keyboard = mypalletizer_260.teleop_keyboard:main',
        ],
    },
)
