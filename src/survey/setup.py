import os
from glob import glob

from setuptools import setup

package_name = 'survey'

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
    description='Boustrophedon area-survey offboard node for PX4 (Phase I swarm project).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'survey_node = survey.survey_node:main',
        ],
    },
)
