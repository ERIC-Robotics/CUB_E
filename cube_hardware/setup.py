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
        srcpath + '.transform'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'transform = src.transform:main',
        ],
    },
)
