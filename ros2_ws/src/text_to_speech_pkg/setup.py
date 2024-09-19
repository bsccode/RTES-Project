from setuptools import setup
import os
from glob import glob

package_name = 'text_to_speech_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # Install the package.xml
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files if any (optional)
        # (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='your_email@example.com',
    description='ROS 2 package for text-to-speech using Piper',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'text_to_speech_node = text_to_speech_pkg.tts_node:main',
            'tts_publisher = text_to_speech_pkg.tts_publisher:main',  # Added publisher entry
            'result_to_tts = text_to_speech_pkg.result_to_tts:main', # Added bridge node entry
        ],
    },
)
