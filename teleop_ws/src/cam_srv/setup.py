from setuptools import find_packages, setup

package_name = 'cam_srv'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hanmg',
    maintainer_email='hanmg@buaa.edu.cn',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wrist_cam_streaming = cam_srv.gopro:main',
            'wrist_cam_capture = cam_srv.wri_cap:main',
        ],
    },
)
