from setuptools import find_packages, setup

package_name = "sim_supervisor"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Raul Moran",
    maintainer_email="raul@isc-fs.com",
    description=(
        "sim_supervisor_node — simulates the IFS-08 uDV (microROS "
        "endpoint over USB CDC on the real car) for the DV pipeline. "
        "Sim-only; not built on the real car."
    ),
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sim_supervisor_node = sim_supervisor.sim_supervisor_node:main",
            "supervisor_cli = sim_supervisor.supervisor_cli:main",
        ],
    },
)
