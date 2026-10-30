from glob import glob

from setuptools import find_packages, setup

package_name = "pm3rrr_control"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Basheer Altawil",
    maintainer_email="basheeraltaweel@gmail.com",
    description="ROS 2 control nodes for the 3-RRR parallel manipulator",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "controller = pm3rrr_control.controller_node:main",
            "scenario = pm3rrr_control.scenario_node:main",
            "imu_teleop = pm3rrr_control.imu_teleop_node:main",
            "fake_hand_imu = pm3rrr_control.fake_hand_imu_node:main",
            "fk_monitor = pm3rrr_control.fk_monitor_node:main",
            "serial_bridge = pm3rrr_control.serial_bridge_node:main",
        ],
    },
)
