from glob import glob
from setuptools import find_packages, setup

package_name = "front_camera_info"

# Install the YAML and URDF with the Python node so ROS launch can resolve
# them through the package share directory after a workspace build.
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/urdf", glob("urdf/*.urdf")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    entry_points={"console_scripts": ["camera_info_node = front_camera_info.camera_info_node:main"]},
    zip_safe=True,
    maintainer="Lampidis",
    maintainer_email="loukas.lampidis@gmail.com",
    description="Publish calibrated front camera information and the supplied sensor transforms.",
    license="Apache-2.0",
)
