from setuptools import setup

package_name = 'lidar_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'lidar_bridge_node = '
            'lidar_bridge.lidar_bridge_node:main',
        ],
    },
)
