from setuptools import find_packages, setup

package_name = 'unity_udp_ros_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/udp_bridge_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='henry',
    maintainer_email='henrydalrymple53@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'udp_bridge_pub = unity_udp_ros_bridge.UDPBridgePub:main',
            'udp_bridge_sub = unity_udp_ros_bridge.UDPBridgeSub:main',
            'udp_bridge_srv = unity_udp_ros_bridge.UDPBridgeSrv:main',
            'udp_bridge_action = unity_udp_ros_bridge.UDPBridgeAction:main'
        ],
    },
)
