from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'a_star_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ysn0630',
    maintainer_email='rkrtkdgj@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'map_reader = a_star_pkg.map_reader:main',
            'navigation = a_star_pkg.navigation:main',
            'yolo_detector = a_star_pkg.yolo_detector:main',
        ],
    },
)
