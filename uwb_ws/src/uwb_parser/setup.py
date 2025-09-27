from setuptools import find_packages, setup

package_name = 'uwb_parser'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/uwb_parser.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sy',
    maintainer_email='sy@todo.todo',
    description='UWB coordinate parser for ROS2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uwb_coordinate_parser = uwb_parser.uwb_coordinate_parser:main',
            'uwb_navigation_system = uwb_parser.uwb_navigation_system:main',
        ],
    },
)