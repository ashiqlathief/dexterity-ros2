from glob import glob

from setuptools import setup

package_name = "dexterity_evaluation"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ashiq",
    maintainer_email="ashiq.lathief@gmail.com",
    description="Ground truth evaluation of dexterity in the Gazebo simulation",
    license="BSD",
    entry_points={"console_scripts": ["evaluation_node = dexterity_evaluation.evaluation_node:main"]},
)
