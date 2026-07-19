from setuptools import setup, find_packages
package_name = "rangerwm_preprocess"
setup(
    name=package_name, version="0.1.0", packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"], zip_safe=True, maintainer="rangerwm",
    description="rosbag->dataset conversion and mode-aware command conversion node.",
    license="MIT",
    entry_points={"console_scripts": [
        "cmd_to_mode_aware = rangerwm_preprocess.cmd_to_mode_aware_node:main",
        "bev_from_scan = rangerwm_preprocess.bev_from_scan:main",
    ]},
)
