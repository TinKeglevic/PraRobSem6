from setuptools import find_packages, setup

package_name = 'prarob_calib'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/camera_calib.launch.py']),
        ('share/' + package_name + '/launch', ['launch/camera_world.launch.py']),
        ('share/' + package_name + '/config', ['config/camera_calibration_params.yaml']), 
        ('share/' + package_name + '/config', ['config/camera_params.yaml']),
        ('share/' + package_name + '/config', ['config/logitech_calib.yaml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fpetric',
    maintainer_email='f5r1c.1m0@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cam_to_world = prarob_calib.camera_to_world:main'
        ],
    },
)
