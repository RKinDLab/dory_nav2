from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dory_nav2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('config/*.yaml')),
        (os.path.join('share', package_name), glob('casadi/*.casadi'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='geaglin',
    maintainer_email='gge0866@louisiana.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'clock_node = dory_nav2.clock_node:main',
        'gen_cmd_vel = dory_nav2.generate_command_vel:main',
        'odom_publisher = dory_nav2.odom_publisher:main',
        'gen_force = dory_nav2.generate_principle_forces:main'
        ],
    },
)
