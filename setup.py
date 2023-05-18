from setuptools import setup

package_name = 'teleop_gui'
submodule = 'teleop_gui/gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodule],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='steins',
    maintainer_email='saumyaraj188@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points= {
        'console_scripts': [
            'teleop = teleop_gui.teleop_pub:main',
        ],
    },
)
