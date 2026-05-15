from setuptools import setup

package_name = 'ranger_replan_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/replan_manager.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zbx',
    maintainer_email='zbx@example.com',
    description='Replanning trigger and recovery manager',
    license='MIT',
    entry_points={
        'console_scripts': [
            'replan_manager_node = ranger_replan_manager.replan_manager_node:main',
        ],
    },
)
