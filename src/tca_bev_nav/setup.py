from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'tca_bev_nav'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO_your_name',
    maintainer_email='you@example.com',
    description='TCA-BEV: Time- and Calibration-Aware Conservative BEV Fusion '
                'for Safe Indoor UGV Navigation.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'time_align_node = tca_bev_nav.time_align_node:main',
            'calibration_uncertainty_node = '
            'tca_bev_nav.calibration_uncertainty_node:main',
            'tca_bev_fusion_node = tca_bev_nav.tca_bev_fusion_node:main',
            'safety_supervisor_node = tca_bev_nav.safety_supervisor_node:main',
            'wheel_observer_node = tca_bev_nav.wheel_observer_node:main',
        ],
    },
)
