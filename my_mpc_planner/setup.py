from setuptools import find_packages, setup
import os               # <--- EKSİK OLAN SATIR BU, MUTLAKA EKLE
from glob import glob
package_name = 'my_mpc_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch ve Config dosyalarını sisteme tanıtıyoruz:
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='atak',
    maintainer_email='sametatak1234@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'teleop_node = my_mpc_planner.teleop:main',
        'python_mpc_node = my_mpc_planner.python_mpc_node:main',
        'python_nmpc_node = my_mpc_planner.nonlinear_mpc:main',
        'python_rl_node = my_mpc_planner.rl_controller:main',
        'python_rl_node2 = my_mpc_planner.rl_controller_single:main',
        ],
    },
)
