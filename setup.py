import os
from glob import glob
from setuptools import find_packages, setup

package_name = "maze_robot_sim"


def data_files_from_dir(src_dir, dest_prefix):
    files = []
    for root, _, filenames in os.walk(src_dir):
        rel_root = os.path.relpath(root, src_dir)
        dest = os.path.join(dest_prefix, src_dir) if rel_root == "." else os.path.join(dest_prefix, src_dir, rel_root)
        paths = [os.path.join(root, f) for f in filenames]
        if paths:
            files.append((dest, paths))
    return files


data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml"]),
]
data_files += data_files_from_dir("launch", "share/" + package_name)
data_files += data_files_from_dir("description", "share/" + package_name)
data_files += data_files_from_dir("worlds", "share/" + package_name)
data_files += data_files_from_dir("config", "share/" + package_name)

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Yassine",
    maintainer_email="you@example.com",
    description=(
        "Simulation Gazebo Harmonic + ROS2 Jazzy d'un robot différentiel "
        "naviguant dans un labyrinthe (A*), avec IMU, encodeurs, marqueur ArUco "
        "et drone Crazyflie optionnel (cartographie + suivi)."
    ),
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "maze_navigator_node = maze_robot_sim.maze_navigator_node:main",
            "maze_world_generator = maze_robot_sim.maze_world_generator:main",
            "drone_mapper_node = maze_robot_sim.drone_mapper_node:main",
        ],
    },
)
