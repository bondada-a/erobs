import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'drylab_calibration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'rviz_config'), glob(os.path.join('rviz_config', '*.scene'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aditya',
    maintainer_email='bondada.a@northeastern.edu',
    description='Package with saved ZED Camera calibration in drylab',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scene_loader = drylab_calibration.scene_loader:main',
        ],
    },
)
