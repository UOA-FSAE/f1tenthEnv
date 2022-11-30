import os
from glob import glob

from setuptools import setup

package_name = 'gazebo_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Christopher Graham, everyone',
    maintainer_email='chrisgraham908@gmail.com',
    description='Simulates f1tenth stuff in gazebo',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reward_gate_node = gazebo_simulation.reward_gate_node:main',
            'termination_node = gazebo_simulation.termination_node:main',
        ],
    },
)
