#Setup file for compiling qronk_pybullet
from setuptools import find_packages, setup
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'qronk_pybullet'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        ('share/'+ package_name, ['qronk.urdf'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zakcw',
    maintainer_email='63052069+16cra40@users.noreply.github.com',
    description='qronk_pybullet package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'startSim = qronk_pybullet.qronk_pybullet:main'
        ],
    },
)
