from setuptools import setup, find_packages

setup(
    name="ifssim",
    version="1.0.0",
    description="IFSSIM Formula Student Driverless Simulator Python Client",
    packages=find_packages(),
    install_requires=["numpy"],
    python_requires=">=3.8",
)
