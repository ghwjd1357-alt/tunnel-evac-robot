import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'tunnel_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ★ 함정: 아래 3줄을 빼먹으면 colcon build 는 성공하는데 런타임에
        #   "런치/파라미터 파일을 못 찾음" 으로 죽는다 (PITFALLS.md §3).
        #   install/share 로 복사돼야 get_package_share_directory 가 찾는다.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='minwoo',
    maintainer_email='ghwjd1357@gmail.com',
    description='실차(Jetson) 전용 bringup — 런치·파라미터·URDF',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # 조건 기동 게이트 — 앞 단계가 '실제로 살아났는지' 확인되면 0 으로 종료한다.
            'readiness_gate = tunnel_bringup.readiness_gate:main',
        ],
    },
)
