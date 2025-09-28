from setuptools import find_packages, setup
from glob import glob

package_name = 'vehicle_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.*')),
        ('share/' + package_name + '/urdfs/urdf', glob('urdfs/urdf/*')),
        ('share/' + package_name + '/urdfs/worlds', glob('urdfs/worlds/*')),
        ('share/' + package_name + '/urdfs/models/brick_box_wall', glob('urdfs/models/brick_box_wall/*')),
        ('share/' + package_name + '/urdfs/models/brick_box_wall2', glob('urdfs/models/brick_box_wall2/*')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adithya',
    maintainer_email='adithya.ramakrishnan@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
