from setuptools import find_packages, setup

package_name = 'semantic_door_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/semantic_nav.launch.py']),
        ('share/' + package_name + '/config', ['config/semantic_nav.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotics_user',
    maintainer_email='replace@example.com',
    description='Minimal semantic door navigation overlay.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'aruco_door_observer = semantic_door_nav.aruco_door_observer:main',
            'topology_manager = semantic_door_nav.topology_manager:main',
            'reactive_local_controller = semantic_door_nav.reactive_local_controller:main',
        ],
    },
)
