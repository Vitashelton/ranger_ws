from setuptools import setup

package_name = 'ranger_nav_metrics'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zbx',
    maintainer_email='zbx@example.com',
    description='Navigation metrics logging and analysis',
    license='MIT',
    entry_points={
        'console_scripts': [
            'metrics_logger = ranger_nav_metrics.metrics_logger:main',
            'bag_analyzer = ranger_nav_metrics.bag_analyzer:main',
        ],
    },
)
