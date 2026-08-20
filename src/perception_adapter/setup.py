from setuptools import setup

package_name = 'perception_adapter'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/adapter.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='minwoo',
    maintainer_email='kimwonjung1240@gmail.com',
    description='/detections → /alarm 어댑터 (역할 B 인지 → 역할 A 미션)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'adapter_node = perception_adapter.adapter_node:main',
            'fake_detections = perception_adapter.fake_detections:main',
        ],
    },
)
