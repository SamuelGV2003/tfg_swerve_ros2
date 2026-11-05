import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'swerve_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Samuel Gómez Villanueva',
    maintainer_email='sagovi03@gmail.com',
    description='Control de la plataforma swerve de dos módulos, en simulación y con el prototipo',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'swerve_kinematics_node = swerve_control.swerve_kinematics_node:main',
            'swerve_punto_objetivo_node = swerve_control.swerve_punto_objetivo_node:main',
            'gemelo_puente_node = swerve_control.gemelo_puente_node:main',
        ],
    },
)