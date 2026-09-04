# Tiny ament_python package for the ROS pub/sub warm-up.
# Not part of the Docker stack by default — build in a sourced ROS 2 env.

from setuptools import setup

package_name = "ros_pubsub_exercise"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ISC Racing",
    maintainer_email="driverless@iscracingteam.com",
    description="Onboarding toy talker/listener",
    license="MIT",
    entry_points={
        "console_scripts": [
            "talker = ros_pubsub_exercise.talker:main",
            "listener = ros_pubsub_exercise.listener:main",
        ],
    },
)
