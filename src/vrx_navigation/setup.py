# from setuptools import find_packages, setup

# package_name = 'vrx_navigation'

# setup(
#     name=package_name,
#     version='0.0.0',
#     packages=find_packages(exclude=['test']),
#     data_files=[
#         ('share/ament_index/resource_index/packages',
#             ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#     ],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     maintainer='guru-the-lord',
#     maintainer_email='sgurukesav@gmail.com',
#     description='TODO: Package description',
#     license='MIT',
#     extras_require={
#         'test': [
#             'pytest',
#         ],
#     },
#     entry_points={
#         'console_scripts': [
#         ],
#     },
# )
from setuptools import setup
import os
from glob import glob

package_name = 'vrx_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='guru-the-lord',
    maintainer_email='sgurukesav@gmail.com',
    description='VRX Navigation and Control Package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kinematics_filter = vrx_navigation.kinematics_filter_launch:main',
            'los_guidance = vrx_navigation.LOS_vrx:main',
            'pid_controller = vrx_navigation.PID_vrx:main',
            'map_viz = vrx_navigation.map:main',
            'path_planner = vrx_navigation.path_planner:main'
        ],
    },
)