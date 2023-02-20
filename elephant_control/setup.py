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
            'mpc_node = test_mpc.mpc_node:main',
            'test_pub = test_mpc.mpc_pos_pub:main',
            'test_pub_back = test_mpc.mpc_pos_pub_back:main',
            'mpc_node_open_loop = test_mpc.mpc_node_open_loop:main',
        ],
    },
)
