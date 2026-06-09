from setuptools import find_packages, setup
# aaaa
package_name = 'rrr_arm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lorena',
    maintainer_email='lorena.hrman@fer.hr',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'kinematika = rrr_arm.kinematika:main',
            'robot_gui = rrr_arm.robot_gui:main',
            'rviz_visualization = rrr_arm.rviz_visualization:main'
        ],
    },
)
