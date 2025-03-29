from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'arm_hand_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'UR5/mjcf'), glob('mjcf/UR5+gripper/**.xml')),
        (os.path.join('share', package_name, 'UR5/mjcf/mesh/visual'), glob('mjcf/UR5+gripper/mesh/visual/**')),
        (os.path.join('share', package_name, 'UR5/mjcf/textures'), glob('mjcf/UR5+gripper/textures/**.png')),

        (os.path.join('share', package_name, 'iiwa/mjcf'), glob('mjcf/iiwa/**.xml')),
        (os.path.join('share', package_name, 'iiwa/mjcf/meshes'), glob('mjcf/iiwa/meshes/**')),

        (os.path.join('share', package_name, 'jaco2/mjcf'), glob('mjcf/jaco2/**.xml')),
        (os.path.join('share', package_name, 'jaco2/mjcf/meshes'), glob('mjcf/jaco2/meshes/**')),

        (os.path.join('share', package_name, 'kuka_iiwa_14/mjcf'), glob('mjcf/kuka_iiwa_14/**.xml')),
        (os.path.join('share', package_name, 'kuka_iiwa_14/mjcf/assets'), glob('mjcf/kuka_iiwa_14/assets/**')),

        (os.path.join('share', package_name, 'kuka_iiwa_14/urdf'), glob('urdf/kuka_iiwa14/**.sdf')),
        # (os.path.join('share', package_name, 'kuka_iiwa_14/urdf/visual'), glob('urdf/kuka_iiwa14/visual/**')),
        # (os.path.join('share', package_name, 'kuka_iiwa_14/urdf/collision'), glob('urdf/kuka_iiwa14/collision/**')),

        (os.path.join('share', package_name, 'UR5/urdf'), glob('urdf/UR5/**.**')),
        # (os.path.join('share', package_name, 'UR5/urdf/visual'), glob('urdf/UR5/visual/**')),
        # (os.path.join('share', package_name, 'UR5/urdf/collision'), glob('urdf/UR5/collision/**')),
        (os.path.join('share', package_name, 'roake/urdf'), glob('urdf/roake/**.**')),


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='meng',
    maintainer_email='hanmg@buaa.edu.cn',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
