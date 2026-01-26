import os
from glob import glob
from setuptools import setup

package_name = 'pipe_crawler_control'

setup(
    name=package_name,
    version='2.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/pipe_crawler.launch.py']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/scripts', glob('scripts/*.sh')),
        ('share/' + package_name + '/web', glob('web/*.html')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='craig',
    maintainer_email='craig@example.com',
    description='Simple controller for pipe crawler robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_controller = pipe_crawler_control.motor_controller:main',
            'simple_controller = pipe_crawler_control.simple_controller:main',
            'cpu_temp_publisher = pipe_crawler_control.cpu_temp_publisher:main',
            'update_manager = pipe_crawler_control.update_manager:main',
            'network_info_publisher = pipe_crawler_control.network_info_publisher:main',
            'wt901_imu = pipe_crawler_control.wt901_imu_node:main',
        ],
    },
    scripts=[],
)
