import os
from glob import glob
from setuptools import setup

package_name = 'atsd_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'systemd'), glob('systemd/*')),
        (os.path.join('share', package_name, 'udev'), glob('udev/*')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ATSD',
    maintainer_email='dev@atsd.local',
    description='Запуск подсистемы',
    license='MIT',
    entry_points={'console_scripts': []},
)
