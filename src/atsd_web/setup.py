import os
from glob import glob
from setuptools import setup

package_name = 'atsd_web'

setup(
    name=package_name,
    version='0.3.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'static'), glob('atsd_web/static/*.*')),
        (os.path.join('share', package_name, 'static', 'maps'), glob('atsd_web/static/maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ATSD',
    maintainer_email='dev@atsd.local',
    description='Веб-интерфейс заказов',
    license='MIT',
    entry_points={
        'console_scripts': [
            'web_node = atsd_web.server:main',
        ],
    },
)
