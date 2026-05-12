from glob import glob
import os
from setuptools import setup

package_name = 'my_nav_launch'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kto-to',
    maintainer_email='',
    description='',
    license='',
    tests_require=[],
    entry_points={
        'console_scripts': [],
    },
)