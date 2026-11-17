from setuptools import setup

name = 'elaf_nav2_mission_jazzy'
setup(
    name=name, version='0.1.0', packages=[name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + name]),
        ('share/' + name, ['package.xml']),
        ('share/' + name + '/launch', ['launch/elaf_nav2.launch.py']),
        ('share/' + name + '/config', ['config/nav2_params.yaml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Elaf', maintainer_email='elaf@example.com',
    description='Minimal F1TENTH Nav2 lap mission', license='Apache-2.0',
    entry_points={'console_scripts': [
        'ackermann_bridge = elaf_nav2_mission_jazzy.ackermann_bridge:main',
        'mission = elaf_nav2_mission_jazzy.mission:main',
    ]},
)
