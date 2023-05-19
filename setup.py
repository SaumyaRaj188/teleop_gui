import os
from glob import glob
from setuptools import setup

package_name = 'teleop_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ARES',
    maintainer_email='TODO',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points= {
        'console_scripts': [
            'controller = teleop_gui.main:main',
            'test_listener = teleop_gui.test_listener:main',
        ],
    },
)
