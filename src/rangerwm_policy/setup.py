from setuptools import setup, find_packages
package_name = "rangerwm_policy"
setup(
    name=package_name, version="0.1.0", packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"], zip_safe=True, maintainer="rangerwm",
    description="Policy inference node.", license="MIT",
    entry_points={"console_scripts": ["policy_node = rangerwm_policy.policy_node:main"]},
)
