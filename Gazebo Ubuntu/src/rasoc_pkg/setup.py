import os # Operating system library
from glob import glob # Handles file path names
from setuptools import setup # Facilitates the building of packages

package_name = 'rasoc_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Path to the launch file      
        (os.path.join('share', package_name,'launch'), glob('launch/*.launch.py')),

        # Path to the world file
        (os.path.join('share', package_name,'worlds/'), glob('./worlds/*')),

        # Path to the warehouse sdf file
        (os.path.join('share', package_name,'models/finca_world_real/'), glob('./models/finca_world_real/*')),

        # Path to the mobile robot sdf file
        (os.path.join('share', package_name,'models/rasoc_robot2/'), glob('./models/rasoc_robot2/*')),
        
        # Path to the world file (i.e. warehouse + global environment)
        (os.path.join('share', package_name,'models/'), glob('./worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maxx',
    maintainer_email='maxx@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'spawn_demo = warehouse_robot_spawner_pkg.spawn_demo:main'
        ],
    },
)
