from setuptools import setup

package_name = 'atsd_sensors'

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
    description='Драйверы сенсоров: лидар LD19, GNSS BN-880, гейдж X1202',
    license='MIT',
    entry_points={
        'console_scripts': [
            'lidar_node = atsd_sensors.lidar_node:main',
            'gnss_node = atsd_sensors.gnss_node:main',
            'battery_node = atsd_sensors.battery_node:main',
        ],
    },
)
