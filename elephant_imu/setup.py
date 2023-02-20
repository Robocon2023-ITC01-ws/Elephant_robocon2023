from setuptools import setup
import os
from glob import glob
package_name = 'elephant_imu'
package_name_in = 'ros_imu_bno055'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name,package_name_in],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotic',
    maintainer_email='chetsokhpanha@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'imu = elephant_imu.imu:main',
        ],
    },
)
