import os
from glob import glob

from setuptools import find_packages, setup


package_name = "line_tracking"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Cobiz",
    maintainer_email="dev@cobiz.local",
    description="YOLOP road/lane segmentation tracking for quadruped velocity commands",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "line_tracking_node = line_tracking.line_tracking_node:main",
        ],
    },
)
