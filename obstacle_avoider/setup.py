import os
from glob import glob
from setuptools import setup

package_name = "obstacle_avoider"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"),
            glob("launch/*.py")),
        (os.path.join("share", package_name, "urdf"),
            glob("urdf/*")),
        (os.path.join("share", package_name, "worlds"),
            glob("worlds/*")),
        (os.path.join("share", package_name, "rviz"),
            glob("rviz/*")),
        (os.path.join("share", package_name, "config"),
            glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="fatima",
    maintainer_email="fatima@todo.todo",
    description="Autonomous obstacle avoiding robot with VFH and SLAM",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "avoider_node = obstacle_avoider.avoider_node:main",
        ],
    },
)
