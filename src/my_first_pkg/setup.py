import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'my_first_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch 폴더의 *.launch.py 들을 install/share 로 복사
        # → 이 줄이 없으면 'ros2 launch' 가 파일을 못 찾는다 (파일없음 에러)
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='minwoo',
    maintainer_email='ghwjd1357@gmail.com',
    description='ROS2 learning package: talker/listener demo',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # '실행이름 = 패키지.파일:함수' 형식
            # → 'ros2 run my_first_pkg talker' 하면 talker.py의 main()이 실행됨
            'talker = my_first_pkg.talker:main',
            'listener = my_first_pkg.listener:main',
        ],
    },
)
