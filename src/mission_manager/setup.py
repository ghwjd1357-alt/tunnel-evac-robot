import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mission_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # config 의 웨이포인트 yaml 을 share 로 복사 (노드가 런타임에 읽음)
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # launch 파일 복사 (없으면 ros2 launch 가 못 찾음)
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='minwoo',
    maintainer_email='ghwjd1357@gmail.com',
    description='대피 유도 로봇 임무 상태머신(두뇌)',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # 'ros2 run mission_manager mission_node' → mission_node.py 의 main() 실행
            'mission_node = mission_manager.mission_node:main',
        ],
    },
)
