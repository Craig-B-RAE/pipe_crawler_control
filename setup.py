from setuptools import setup

package_name = 'pipe_crawler_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/pipe_crawler.launch.py']),
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
            'simple_controller = pipe_crawler_control.simple_controller:main',
            'cpu_temp_publisher = pipe_crawler_control.cpu_temp_publisher:main',
        ],
    },
    scripts=[],
)
