from setuptools import setup, find_packages
package_name = "rangerwm_planning"
setup(
    name=package_name, version="0.1.0", packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"], zip_safe=True, maintainer="rangerwm",
    description="Pure-python planning: MPC reranking, analytic WM, safety filter.", license="MIT",
    entry_points={"console_scripts": []},
)
