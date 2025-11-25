from setuptools import find_packages, setup
from setuptools import setup
import os


package_name = 'robot_final_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #launch 파일을 설치하도록 설정
        (os.path.join('share', package_name, 'launch'), [
        os.path.join('launch', 'slam_nav2_rviz.launch.py'),
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='flynn',
    maintainer_email='youdongoh67@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
             # 나중에 nav2_goal_sender 같은 노드 executables 여기 추가
        ],
    },
)
