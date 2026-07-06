import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'tunnel_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ★ 2B 함정 복습: 만든 폴더는 install/share 로 복사해야 ros2 launch 가 찾음
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        # 2D: URDF 로봇 설계도도 install/share 로 복사 (robot.launch.py 가 찾음)
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        # 2E: SLAM/Nav2 파라미터(yaml) 도 install/share 로 복사
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='minwoo',
    maintainer_email='minwoo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # 가짜 추종자 (②단계): 로봇 뒤를 따라 걷는 원기둥. 시뮬 전용 시험 도구.
            'fake_follower = tunnel_sim.fake_follower:main',
        ],
    },
)
