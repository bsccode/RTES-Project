from setuptools import setup
from glob import glob
import os

package_name = 'robot_control_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Package description',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_control_node = robot_control_pkg.robot_control_node:main',
        ],
    },
)
