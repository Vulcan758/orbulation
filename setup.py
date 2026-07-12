from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'orbulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yml]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mahir',
    maintainer_email='mahirgored@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'orbslam_ros = orbulation.orbslam_ros:main',
            'tum_data_pump = orbulation.tum_data_pump:main',
            'vi_tum_data_pump = orbulation.vi_tum_data_pump:main',
            'zed_stereo_ros = orbulation.zed_stereo_ros:main',
        ],
    },
)
