from setuptools import setup

package_name = 'ranger_sensor_fusion'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/sensor_fusion.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zbx',
    maintainer_email='zbx@example.com',
    description='MID360S + D435i multi-sensor obstacle fusion',
    license='MIT',
    entry_points={
        'console_scripts': [
            'obstacle_cluster_node = ranger_sensor_fusion.obstacle_cluster_node:main',
            'd435i_obstacle_node = ranger_sensor_fusion.d435i_obstacle_node:main',
            'sensor_fusion_node = ranger_sensor_fusion.sensor_fusion_node:main',
        ],
    },
)
