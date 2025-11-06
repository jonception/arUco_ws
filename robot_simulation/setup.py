import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'worlds'), 
         glob(os.path.join('worlds', '*.world'))),
       (os.path.join('share', package_name, 'textures'), 
         glob(os.path.join('textures', '*.*'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='john',
    maintainer_email='jonathangeorge2006@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
