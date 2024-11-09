from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'odrive_ep_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ascroid',
    maintainer_email='pas123068@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'odrive_ep_motor_double = odrive_ep_pkg.odrive_ep_double:main',
        'odrive_ep_motor_single = odrive_ep_pkg.odrive_ep_single:main',
        'odrive_ep_joy = odrive_ep_pkg.odrive_joy:main',
        'odrive_ep_joy_no_service = odrive_ep_pkg.odrive_joy_no_service:main',
        'odrive_ep_joy_single = odrive_ep_pkg.odrive_joy_single:main',
        ],
    },
)
