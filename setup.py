from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'artpark_assesment'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='intel',
    maintainer_email='01fe21bee114@kletech.ac.in',
    description='TODO: Package description',
    license='TODO: License declaration',
    # tests_require=['pytest'],
    extras_require={'test':['pytest'],},
    entry_points={
        'console_scripts': [
            'publisher_node = artpark_assesment.publisher_node:main',
            'subscriber_node = artpark_assesment.subscriber_node:main',
            'robot_state = artpark_assesment.robot_state:main',
            'dynamic_node_handler = artpark_assesment.dynamic_node_handler:main'

        ],
    },
)
