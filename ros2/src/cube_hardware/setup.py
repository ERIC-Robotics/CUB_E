import os
from glob import glob
from setuptools import find_packages
from setuptools import setup

package_name = 'cube_hardware'
srcpath = 'src'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        srcpath + '.cmdmotor',
        srcpath + '.odometry', 
        srcpath + '.lidar_publish',
        srcpath + '.navigation_action',
        srcpath + '.navigation_waypoint_action',
        srcpath + '.control',
        srcpath + '.lidar_saftey',
        srcpath + '.lidar_min',
        srcpath + '.nav_feedback_sub',
        srcpath + '.lidar_msg2string',
        srcpath + '.navigation_test',
        srcpath + '.transform'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jatin',
    maintainer_email='jatin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmdmotor = src.cmdmotor:main',
            'odometry = src.odometry:main',
            'lidarpub = src.lidar_publish:main',
            'transform = src.transform:main',
            'control = src.control:main',
            'lidar_msg2str = src.lidar_msg2string:main',
            'nav_feedback_sub = src.nav_feedback_sub:main',
            'lidar_min = src.lidar_min:main',
            'lidar_saftey = src.lidar_saftey:main',
            'navigation = src.navigation_test:main',
            'navigation_action = src.navigation_action:main',
            'navigation_waypoint_action = src.navigation_waypoint_action:main',
        ],
    },
)
