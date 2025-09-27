import os
from glob import glob
from setuptools import setup

package_name = 'parking_exe'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch 디렉토리와 그 안의 모든 .py 파일을 설치하도록 지정
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sy',
    maintainer_email='sy@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    # [가장 중요한 부분] 실행 파일을 생성하는 설정
    entry_points={
        'console_scripts': [
            'parking_exe_node = parking_exe.parking_exe:main',
        ],
    },
)