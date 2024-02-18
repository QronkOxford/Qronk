from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'qronk_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shinnigus',
    maintainer_email='shinben0327@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planned_gait = qronk_control.planned_gait:main',
            'gait_sequences = qronk_control.gait_sequences:main',
            'kinematics = qronk_control.kinematics:main',
        ],
    },
)
