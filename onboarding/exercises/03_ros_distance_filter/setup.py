from setuptools import setup

package_name = "ros_distance_filter"

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
    maintainer="onboarding student",
    maintainer_email="you@example.com",
    description="Onboarding distance filter",
    license="MIT",
    entry_points={
        "console_scripts": [
            "filter = ros_distance_filter.filter_node:main",
        ],
    },
)
