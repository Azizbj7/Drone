from setuptools import find_packages, setup
from glob import glob
import os

package_name ='drone'

setup(
    name=package_name,
    version='0.0.0',
    #packages=['drone', 'SPSO'],
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #('share/' + package_name + '/worlds', ['worlds/my_world.sdf']),
        # Include URDF and meshes
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.STL')),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aziz',
    maintainer_email='aziz@todo.todo',
    description='Teleoperation avec le clavier',
    #description='Path planning with SPSO',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
              #'spso_node = SPSO.SPSO_Main:main',
              'keyboard_teleop = drone.keybord_teleop:main',
               ],
               },
      )

