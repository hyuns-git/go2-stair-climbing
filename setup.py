from setuptools import find_packages, setup

package_name = 'go2_nav_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/slam_precise.yaml', 'config/nav2_go2.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='nvidia@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cmd_vel_bridge = go2_nav_bridge.cmd_vel_bridge:main',
            'scan_maker_os1 = go2_nav_bridge.scan_maker_os1:main',
            'scan_maker_l1 = go2_nav_bridge.scan_maker_l1:main',
            'stair_traverse_node = go2_nav_bridge.stair_traverse_node:main',
            'photo_shooter = go2_nav_bridge.photo_shooter:main',
            'mission_5f_to_rooftop = go2_nav_bridge.mission_5f_to_rooftop:main',
            'email_sender = go2_nav_bridge.email_sender:main',
            'speaker = go2_nav_bridge.speaker:main',
            'rooftop_panorama = go2_nav_bridge.rooftop_panorama:main',
        ],
    },
)
