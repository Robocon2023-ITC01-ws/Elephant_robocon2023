from setuptools import setup

package_name = 'elephant_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zero',
    maintainer_email='zeroeverything001@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mpc_node = elephant_control.mpc_node:main',
            'test_controller = elephant_control.controller:main',
            'test_pub = elephant_control.mpc_pos_pub:main',
            'test_pub_back = elephant_control.mpc_pos_pub_back:main',
            'mpc_node_open_loop = elephant_control.mpc_node_open_loop:main',
        ],
    },
)
