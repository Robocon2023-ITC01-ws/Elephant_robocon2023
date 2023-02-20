from setuptools import setup

package_name = 'elephant_can'
package_name_in = 'kinematic_model'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, package_name_in],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
    'can=elephant_can.can_node:main',
    'teleop=elephant_can.can_teleop:main',
        ],
    },
)
