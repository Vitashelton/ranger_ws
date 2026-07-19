from setuptools import setup, find_packages
package_name = "rangerwm_kinematics"
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rangerwm",
    description="Ranger Mini 2.0 kinematics, Twist arbitration, mode-aware conversion (pure python lib).",
    license="MIT",
)
