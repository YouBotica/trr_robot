from setuptools import setup
import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
  	(os.path.join('share', package_name), glob('urdf/*')),
  	(os.path.join('share', package_name), glob('config/*')),
  	(os.path.join('share', package_name), glob('meshes/*')),
  	(os.path.join('share', package_name), glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andres F Hoyos',
    maintainer_email='andresfelipe3@hotmail.es',
    description='TRR Robot Model',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'state_publisher = robot.state_publisher:main',
        	'move_joints_gazebo = robot.move_joints_gazebo:main',
        ],
    },
)
