from setuptools import setup
import glob

package_name = "alicat_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, glob.glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob.glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="maimon",
    maintainer_email="tlmohren@gmail.com",
    description="Node to interact with Alicat device, tested with MC-series",
    license="LGPLv3",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "alicat_device = alicat_driver.alicat_device:main",
            "example_alicat_client = alicat_driver.example_alicat_client:main",
            "example_flowrate_publisher = alicat_driver.example_flowrate_publisher:main",
            "list_ports = alicat_driver.list_ports:main",
        ],
    },
)
