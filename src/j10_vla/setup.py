from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'j10_vla'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JNH Systems',
    maintainer_email='info@jnhsystems.com',
    description='VLA inference node and backend plugins for the J10 indoor UAV.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vla_inference_node = j10_vla.inference_node:main',
        ],
    },
)
