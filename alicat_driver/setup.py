from setuptools import setup
import glob

package_name = "alicat_driver"

setup(
    name=package_name,
    version="0.0.0",
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
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "alicat_server = alicat_driver.alicat_server:main",
            "example_alicat_client = alicat_driver.example_alicat_client:main",
            "alicat_strokedata_subscriber = alicat_driver.alicat_strokedata_subscriber:main",
        ],
    },
)
