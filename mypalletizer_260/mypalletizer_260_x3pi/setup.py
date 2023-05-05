from setuptools import setup

import os
from glob import glob

package_name = 'mypalletizer_260_x3pi'

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
            'detect_marker = mypalletizer_260_x3pi.detect_marker:main',
            'follow_display = mypalletizer_260_x3pi.follow_display:main',
            'following_marker = mypalletizer_260_x3pi.following_marker:main',
            'listen_real_of_topic = mypalletizer_260_x3pi.listen_real_of_topic:main',
            'listen_real = mypalletizer_260_x3pi.listen_real:main',
            'simple_gui = mypalletizer_260_x3pi.simple_gui:main',
            'slider_control = mypalletizer_260_x3pi.slider_control:main',
            'teleop_keyboard = mypalletizer_260_x3pi.teleop_keyboard:main',
        ],
    },
)
