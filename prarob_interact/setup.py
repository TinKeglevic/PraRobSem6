from setuptools import find_packages, setup

package_name = 'prarob_interact'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'dotenv', 'langchain_openai', 'jpl-rosa'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='pero.drobac@fer.hr',
    description='ROSA-enabled package for the robotics practicum course at UNIZG-FER.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'text_interface = prarob_interact.text_interface:main',
        ],
    },
)
