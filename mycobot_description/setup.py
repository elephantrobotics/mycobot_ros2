import os
from setuptools import setup, __version__ as setuptools_version
from packaging.version import Version
from glob import glob

package_name = 'mycobot_description'

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
        ('share/' + package_name + '/urdf'+'/mypalletizer_260_pi', glob("urdf/mypalletizer_260_pi/*")),
        ('share/' + package_name + '/urdf'+'/mypalletizer_260_m5', glob("urdf/mypalletizer_260_m5/*")),
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
        
        ('share/' + package_name + '/urdf'+'/myarm_c650', glob("urdf/myarm_c650/*")),
        ('share/' + package_name + '/urdf'+'/myarm_m750', glob("urdf/myarm_m750/*")),
        
        ('share/' + package_name + '/urdf'+'/mycobot_pro_630', glob("urdf/mycobot_pro_630/*")),
        
        ('share/' + package_name + '/urdf'+'/mycobot_280_riscv', glob("urdf/mycobot_280_riscv/*")),
        ('share/' + package_name + '/urdf'+'/mycobot_280_rdkx5', glob("urdf/mycobot_280_rdkx5/*")),
        
        ('share/' + package_name + '/urdf'+'/mycobot_320_riscv',glob("urdf/mycobot_320_riscv/*")),



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
