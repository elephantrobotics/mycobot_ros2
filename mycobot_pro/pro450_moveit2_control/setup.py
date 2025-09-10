import os
from setuptools import setup, __version__ as setuptools_version
from packaging.version import Version
from glob import glob

package_name = 'pro450_moveit2_control'

# Check the setuptools version
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

# Dynamically generate a file list
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, "launch"), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, "config"), glob('config/*')),
]

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wangweijian',
    maintainer_email='weijian.wang@elephantrobotics.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sync_plan = pro450_moveit2_control.sync_plan:main',
        ],
    },
)
