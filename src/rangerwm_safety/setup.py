from setuptools import setup, find_packages
package_name = "rangerwm_safety"
setup(
    name=package_name, version="0.1.0", packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"], zip_safe=True, maintainer="rangerwm",
    description="Safety supervisor; sole /cmd_vel publisher.", license="MIT",
    entry_points={"console_scripts": ["safety_node = rangerwm_safety.safety_node:main"]},
)
