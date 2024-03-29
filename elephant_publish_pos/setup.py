from setuptools import setup

package_name = 'elephant_publish_pos'

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
            'test_pub = elephant_publish_pos.mpc_pos_pub:main',
            'test_pub_back = elephant_publish_pos.mpc_pos_pub_back:main',
            'test_pos_pub = elephant_publish_pos.position_pub:main',
        ],
    },
)
