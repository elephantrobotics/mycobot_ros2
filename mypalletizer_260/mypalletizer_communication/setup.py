import os
from setuptools import setup, __version__ as setuptools_version
from packaging.version import Version
from glob import glob

package_name = 'mypalletizer_communication'

# 检查 setuptools 版本
use_dash_separated_options = Version(setuptools_version) < Version("58.0.0")


# 动态生成 setup.cfg 内容
setup_cfg_content = """
[develop]
{script_option}=$base/lib/{package_name}

[install]
{install_scripts_option}=$base/lib/{package_name}
""".format(
    package_name=package_name,
    script_option='script-dir' if use_dash_separated_options else 'script_dir',
    install_scripts_option='install-scripts' if use_dash_separated_options else 'install_scripts'
)

# 将内容写入 setup.cfg
with open("setup.cfg", "w") as f:
    f.write(setup_cfg_content)

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
    maintainer='u20',
    maintainer_email='u20@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             # 节点名=包名.文件名:文件中的函数
            'mypalletizer_topics_seeed = mypalletizer_communication.mypalletizer_topics_seeed:main',
            'mypalletizer_services = mypalletizer_communication.mypalletizer_services:main',
            'mypalletizer_topics = mypalletizer_communication.mypalletizer_topics:main',        

        ],
    },
)
