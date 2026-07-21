from glob import glob
from setuptools import setup
import os

package_name = "rangermini_doorway_sim"

data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml", "README.md"]),
]

for folder in ["launch", "worlds", "description", "config", "rviz", "docs", "scripts", "web_ui"]:
    for path in glob(os.path.join(folder, "*")):
        if os.path.isfile(path):
            data_files.append((os.path.join("share", package_name, folder), [path]))

setup(
    name=package_name,
    version="0.13.1",
    packages=[package_name],
    data_files=data_files,
    scripts=[
        "scripts/corridor_trial_metrics",
        "scripts/corridor_human_command_generator",
        "scripts/kinematic_corridor_sim",
        "scripts/corridor_semantic_filter",
        "scripts/corridor_markers",
        "scripts/corridor_semantic_detector_stub",
        "scripts/corridor_semantic_memory_node",
        "scripts/cmd_vel_watchdog",
        "scripts/start_office_rpg_demo.sh",
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Harry",
    maintainer_email="harry_vancepoy@mail.com",
    description="Corridor semantic shared-control benchmark for RangerMini 2.0.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "human_command_generator = rangermini_doorway_sim.human_command_generator:main",
            "kinematic_rangermini_sim = rangermini_doorway_sim.kinematic_rangermini_sim:main",
            "risk_cmd_filter = rangermini_doorway_sim.risk_cmd_filter:main",
            "doorway_markers = rangermini_doorway_sim.doorway_markers:main",
            "csv_logger = rangermini_doorway_sim.csv_logger:main",
            "compute_trial_metrics = rangermini_doorway_sim.compute_trial_metrics:main",
            "plot_latest_log = rangermini_doorway_sim.plot_latest_log:main",
            "cmd_vel_watchdog = rangermini_doorway_sim.cmd_vel_watchdog:main",
            "task_landmark_planner = rangermini_doorway_sim.task_landmark_planner:main",
            "evaluate_task_landmarks = rangermini_doorway_sim.evaluate_task_landmarks:main",
            "gazebo_odom_adapter = rangermini_doorway_sim.gazebo_odom_adapter:main",
            "office_rpg_npc_schedule = rangermini_doorway_sim.npc_schedule_node:main",
            "office_rpg_sim_perception_adapter = rangermini_doorway_sim.sim_perception_adapter:main",
            "office_rpg_mission_manager = rangermini_doorway_sim.office_mission_manager:main",
            "office_rpg_search_executor = rangermini_doorway_sim.office_search_executor:main",
            "office_rpg_safety_metrics = rangermini_doorway_sim.office_safety_metrics:main",
        ],
    },
)
