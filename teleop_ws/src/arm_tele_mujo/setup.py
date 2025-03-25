from setuptools import find_packages, setup
import os

package_name = 'arm_tele_mujo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'config'), ['config/models.yaml']),
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='meng',
    maintainer_email='hanmg@buaa.edu.cn',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "arm_tele = arm_tele_mujo.arm_tele:main",
            "drake_model = arm_tele_mujo.drake_model:main",
        ],
    },
)
