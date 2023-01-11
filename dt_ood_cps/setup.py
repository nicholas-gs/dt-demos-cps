import os

from setuptools import setup
from glob import glob


package_name = 'dt_ood_cps'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name, 'dt_ood_cps/cropper'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.xml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nicholas-gs',
    maintainer_email='nicholasganshyan@gmail.com',
    description='Package containing nodes and utility scripts relating to the Out-of-Distribution detector.',
    license='GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stream_to_dataset_node=dt_ood_cps.stream_to_dataset_node:main'
        ],
    },
)
