from setuptools import find_packages, setup

package_name = 'beacon_localizer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kakoy-to-chuvak',
    maintainer_email='',
    description='',
    license='',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "beacon_localizer = beacon_localizer.beacon_localizer:main",
        ],
    },
)
