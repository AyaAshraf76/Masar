from setuptools import setup

package_name = 'lane_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'lane_bridge_node = '
            'lane_bridge.lane_bridge_node:main',
        ],
    },
)
