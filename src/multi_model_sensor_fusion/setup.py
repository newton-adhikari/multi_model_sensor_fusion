from setuptools import setup
import os
from glob import glob

package_name = 'multi_model_sensor_fusion'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name], 
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'xacro'), glob('xacro/*.xacro')),
        (os.path.join('share', package_name, 'world'), glob('world/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='test@user.com',
    description='Multi-modal sensor fusion',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_fusion_node = multi_model_sensor_fusion.sensor_subscribers:main',
        ],
    },
)
