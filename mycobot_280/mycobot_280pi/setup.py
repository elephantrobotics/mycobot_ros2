from setuptools import setup

import os
from glob import glob

package_name = 'mycobot_280pi'

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
    maintainer='u2',
    maintainer_email='u2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_display = mycobot_280pi.camera_display:main',
            'detect_marker = mycobot_280pi.detect_marker:main',
            'follow_display = mycobot_280pi.follow_display:main',
            'following_marker = mycobot_280pi.following_marker:main',
            'listen_real_of_topic = mycobot_280pi.listen_real_of_topic:main',
            'listen_real = mycobot_280pi.listen_real:main',
            'opencv_camera = mycobot_280pi.opencv_camera:main',
            'simple_gui = mycobot_280pi.simple_gui:main',
            'slider_control = mycobot_280pi.slider_control:main',
            'teleop_keyboard = mycobot_280pi.teleop_keyboard:main',
        ],
    },
)
