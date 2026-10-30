from setuptools import find_packages, setup

package_name = 'human_detection_node'

# Register the person detector as a standalone ROS executable for Part 3.
setup(
    name=package_name, version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Lampidis', maintainer_email='loukas.lampidis@gmail.com',
    description='Pretrained YOLO person detection for front camera images', license='Apache-2.0',
    entry_points={'console_scripts': [
        'human_detector = human_detection_node.detector:main',
    ]},
)
