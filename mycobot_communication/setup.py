from setuptools import setup
import os
from glob import glob

package_name = 'mycobot_communication'

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

        # python 文件, 无需再entry_points中手动添加，但每次需要在install下相应的目录中添加可执行权限
        # (os.path.join('lib',package_name),glob(package_name+'/*'))

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
            # 节点名=包名.文件名:文件中的函数
            'mycobot_topics_jsnn = mycobot_communication.mycobot_topics_jsnn:main',
            'mycobot_topics_seeed = mycobot_communication.mycobot_topics_seeed:main',
            'mycobot_services = mycobot_communication.mycobot_services:main',
            'mycobot_topics_pi = mycobot_communication.mycobot_topics_pi:main',
            'mycobot_topics = mycobot_communication.mycobot_topics:main',               
        ],
    },
)
