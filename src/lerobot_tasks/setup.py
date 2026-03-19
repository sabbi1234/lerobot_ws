from setuptools import find_packages, setup

package_name = 'lerobot_tasks'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/pick_cube.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sabbi',
    maintainer_email='sabbi.chakri@gmail.com',
    description='Pick and place tasks for SO101 robotic arm using MoveIt 2',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pick_cube = lerobot_tasks.pick_cube:main',
        ],
    },
)
