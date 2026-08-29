import os
from glob import glob

from setuptools import setup

package_name = 'perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sam',
    maintainer_email='sameerverma@example.com',
    description='YOLO detector on the drone downward camera; geotags hits to a hazard map (Phase I).',
    license='MIT',
    entry_points={
        'console_scripts': [
            'detector_node = perception.detector_node:main',
        ],
    },
)
