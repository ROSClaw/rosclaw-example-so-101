from glob import glob

from setuptools import find_packages, setup

package_name = "rosclaw_so101_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="marcodotio",
    maintainer_email="marcodotio@example.com",
    description="Combined bringup for ROSClaw and the SO-101 follower control stack.",
    license="MIT",
    tests_require=["pytest"],
)
