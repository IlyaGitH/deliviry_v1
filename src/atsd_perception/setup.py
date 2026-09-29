from setuptools import setup

package_name = 'atsd_perception'

setup(
    name=package_name,
    version='0.3.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ATSD',
    maintainer_email='dev@atsd.local',
    description='Препятствия по лидару, знаки и светофор по камере',
    license='MIT',
    entry_points={
        'console_scripts': [
            'obstacle_node = atsd_perception.obstacle_node:main',
            'vision_node = atsd_perception.vision_node:main',
        ],
    },
)
