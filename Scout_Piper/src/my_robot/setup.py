from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'my_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),

        (
            'share/' + package_name,
            ['package.xml']
        ),

        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')
        ),

        (
            os.path.join('share', package_name, 'models', 'bottle'),
            glob('models/bottle/*.sdf') +
            glob('models/bottle/*.config') +
            glob('models/bottle/*.png')
        ),

        (
            os.path.join('share', package_name, 'models', 'bottle', 'meshes'),
            glob('models/bottle/meshes/*')
        ),

        (
            os.path.join('share', package_name, 'models', 'bottle', 'materials', 'textures'),
            glob('models/bottle/materials/textures/*')
        ),

        (
            os.path.join('share', package_name, 'models', 'scout_piper'),
            glob('models/scout_piper/*')
        ),

        (
            os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.sdf')
        ),
 ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='kowshik0714',
    maintainer_email='kowshik0714@todo.todo',

    description='Scout Mini autonomous navigation and bottle collection',
    license='Apache-2.0',

    extras_require={
        'test': ['pytest'],
    },

    entry_points={
        'console_scripts': [
            'bottle_detector = my_robot.bottle_detector:main',
	    'depth_processor = my_robot.depth_processor:main',
	    'bottle_distance = my_robot.bottle_distance:main',
	    'bottle_target = my_robot.bottle_target:main',
        ],
    },
)
