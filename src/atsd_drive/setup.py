from setuptools import setup

package_name = 'atsd_drive'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ATSD',
    maintainer_email='dev@atsd.local',
    description='Мост Raspberry Pi - VEX V5',
    license='MIT',
    entry_points={
        'console_scripts': [
            'v5_bridge_node = atsd_drive.v5_bridge_node:main',
        ],
    },
)
