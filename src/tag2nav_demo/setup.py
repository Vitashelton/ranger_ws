from glob import glob
from setuptools import find_packages, setup

package_name = "tag2nav_demo"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    entry_points={"console_scripts": [
        "optimize_tags = tag2nav_demo.optimize_tags:main",
        "evaluate_layouts = tag2nav_demo.evaluate_layouts:main",
        "layout_publisher = tag2nav_demo.layout_publisher:main",
    ]},
)
