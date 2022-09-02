from setuptools import setup

package_name = 'stretch_ros_tutorials'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chintan Desai',
    maintainer_email='cdesai@hello-robot.com',
    description='This package contains a set of ROS tutorials for Stretch beginners',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stow_command = stretch_ros_tutorials.stow_command:main',
            'multipoint_command = stretch_ros_tutorials.multipoint_command:main',
            'marker = stretch_ros_tutorials.marker:main',
            'tf_broadcaster = stretch_ros_tutorials.tf_broadcaster:main',
            'tf_listener = stretch_ros_tutorials.tf_listener:main',
            'scan_filter = stretch_ros_tutorials.scan_filter:main',
            'avoider = stretch_ros_tutorials.avoider:main',
            'move = stretch_ros_tutorials.move:main',
        ],
    },
)
