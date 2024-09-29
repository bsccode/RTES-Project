from setuptools import setup

package_name = 'object_detection_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    # Add any required package dependencies here
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Object detection package for detecting red or blue balls.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detection_node = object_detection_pkg.object_detection_node:main',
        ],
    },
)
