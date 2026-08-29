from setuptools import setup

package_name = 'atsd_actuators'

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
    description='Свет, лента, замок',
    license='MIT',
    entry_points={
        'console_scripts': [
            'light_node = atsd_actuators.light_node:main',
            'lock_node = atsd_actuators.lock_node:main',
        ],
    },
)
