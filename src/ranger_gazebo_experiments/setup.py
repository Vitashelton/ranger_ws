from setuptools import find_packages, setup

package_name = 'ranger_gazebo_experiments'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            [f'launch/{f}' for f in [
                'ranger_lab_world.launch.py',
                'ranger_lab_fusion_demo.launch.py',
                'ranger_people_avoidance_demo.launch.py',
                'ranger_ablation_experiment.launch.py',
            ]]),
        ('share/' + package_name + '/config',
            [f'config/{f}' for f in [
                'people_scenarios.yaml',
                'fusion_modes.yaml',
                'nav2_sim.yaml',
                'yolo_detector.yaml',
                'experiment_metrics.yaml',
            ]]),
        ('share/' + package_name + '/worlds',
            [f'worlds/{f}' for f in [
                'ranger_lab_corridor.world',
                'ranger_low_obstacle.world',
                'ranger_people_corridor.world',
                'ranger_multi_people.world',
            ]]),
        ('share/' + package_name + '/urdf',
            ['urdf/ranger_mini_sim.urdf.xacro']),
        ('share/' + package_name + '/rviz',
            [f'rviz/{f}' for f in [
                'ranger_lab_fusion.rviz',
                'ranger_people_avoidance.rviz',
                'ranger_ablation.rviz',
            ]]),
        ('share/' + package_name + '/docs',
            [f'docs/{f}' for f in [
                'gazebo_people_avoidance_experiment.md',
                'experiment_protocol.md',
            ]]),
        ('share/' + package_name + '/models',
            ['models/README.md']),
        ('share/' + package_name + '/scripts',
            [f'scripts/{f}' for f in [
                'run_all_ablation_trials.py',
                'summarize_experiment_csv.py',
            ]]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zbx',
    maintainer_email='zbx@todo.todo',
    description='Gazebo simulation experiments for Ranger Mini 2.0',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_person_detector_node = ranger_gazebo_experiments.yolo_person_detector_node:main',
            'simulated_people_ground_truth_node = ranger_gazebo_experiments.simulated_people_ground_truth_node:main',
            'actor_proxy_sync_node = ranger_gazebo_experiments.actor_proxy_sync_node:main',
            'person_detection_to_obstacle_node = ranger_gazebo_experiments.person_detection_to_obstacle_node:main',
            'person_avoidance_controller = ranger_gazebo_experiments.person_avoidance_controller:main',
            'experiment_metrics_node = ranger_gazebo_experiments.experiment_metrics_node:main',
            'ablation_runner_node = ranger_gazebo_experiments.ablation_runner_node:main',
            'simulated_lidar_obstacle_adapter = ranger_gazebo_experiments.simulated_lidar_obstacle_adapter:main',
            'simulated_depth_obstacle_adapter = ranger_gazebo_experiments.simulated_depth_obstacle_adapter:main',
            'odom_to_tf_bridge = ranger_gazebo_experiments.odom_to_tf_bridge:main',
        ],
    },
)
