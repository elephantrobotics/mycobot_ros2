import os
from setuptools import setup, __version__ as setuptools_version
from packaging.version import Version
from glob import glob

package_name = 'mypalletizer_260_pi'

# Checking the setuptools version
use_dash_separated_options = Version(setuptools_version) < Version("58.0.0")


# Dynamically generate setup.cfg content
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

# Write the contents to setup.cfg
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
        # launch file path
        (os.path.join('share', package_name, "launch"), glob('launch/*.launch.py')),
        # python file
        # (os.path.join('lib',package_name),glob(package_name+'/*.py')),
        # Configuration File
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
            'detect_marker = mypalletizer_260_pi.detect_marker:main',
            'follow_display = mypalletizer_260_pi.follow_display:main',
            'following_marker = mypalletizer_260_pi.following_marker:main',
            'listen_real_of_topic = mypalletizer_260_pi.listen_real_of_topic:main',
            'listen_real = mypalletizer_260_pi.listen_real:main',
            'simple_gui = mypalletizer_260_pi.simple_gui:main',
            'slider_control = mypalletizer_260_pi.slider_control:main',
            'teleop_keyboard = mypalletizer_260_pi.teleop_keyboard:main',
            'listen_real_service = mypalletizer_260.listen_real_service:main',
            'slider_control_adaptive_gripper = mypalletizer_260_pi.slider_control_adaptive_gripper:main',
        ],
    },
)
