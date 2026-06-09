from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'PraRobSem6'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "urdf"), glob(os.path.join("urdf/*"))),
        (os.path.join("share", package_name, "controllers"), glob(os.path.join("controllers/*"))),
        (os.path.join("share", package_name, "launch"), glob(os.path.join("launch/*"))),
        (os.path.join("share", package_name, "meshes"), glob(os.path.join("meshes/*"))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kegla',
    maintainer_email='kegla@todo.todo',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "kinematics_node=PraRobSem6.kinematics_node:main",
            "rviz_visualization=PraRobSem6.rviz_visualization:main"
        ],
    },
)
