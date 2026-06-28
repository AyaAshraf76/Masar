from setuptools import setup

package_name = 'acc_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'acc_node = '
            'acc_controller.acc_node:main',
        ],
    },
)
