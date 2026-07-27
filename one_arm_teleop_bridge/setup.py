from glob import glob
from setuptools import find_packages, setup


package_name = "one_arm_teleop_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="One-Arm-Teleoperation",
    maintainer_email="maintainer@example.com",
    description="Safety-gated UDP leader to ROS 2 bridge.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "udp_leader_bridge = one_arm_teleop_bridge.udp_leader_bridge:main",
        ],
    },
)
