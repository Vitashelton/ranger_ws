from setuptools import setup
from glob import glob
import os

package_name = 'ranger_nav_v5'

setup(
    name=package_name,
    version='0.5.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'docs'), glob('docs/*.md')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zbx',
    maintainer_email='zbx@example.com',
    description='RangerMini 2.0 real-sensor-ready semantic shared control layer.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'local_bev_risk_node = ranger_nav_v5.local_bev_risk_node:main',
            'semantic_detector_stub = ranger_nav_v5.semantic_detector_stub:main',
            'semantic_memory_node = ranger_nav_v5.semantic_memory_node:main',
            'shared_control_filter = ranger_nav_v5.shared_control_filter:main',
            'cmd_vel_guard = ranger_nav_v5.cmd_vel_guard:main',
            'realtime_csv_logger = ranger_nav_v5.realtime_csv_logger:main',
            'plot_latest_log = ranger_nav_v5.plot_latest_log:main',
        ],
    },
)
