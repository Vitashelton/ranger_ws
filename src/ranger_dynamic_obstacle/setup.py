from setuptools import setup

package_name = 'ranger_dynamic_obstacle'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/dynamic_obstacle.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zbx',
    maintainer_email='zbx@example.com',
    description='Dynamic obstacle tracking and prediction',
    license='MIT',
    entry_points={
        'console_scripts': [
            'obstacle_tracker_node = ranger_dynamic_obstacle.obstacle_tracker_node:main',
            'obstacle_predictor_node = ranger_dynamic_obstacle.obstacle_predictor_node:main',
            'risk_evaluator_node = ranger_dynamic_obstacle.risk_evaluator_node:main',
        ],
    },
)
