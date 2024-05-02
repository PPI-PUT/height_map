from setuptools import find_packages, setup

package_name = 'height_map'

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
    maintainer='f1tenth',
    maintainer_email='gcz@onet.pl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'height_map_node = height_map.height_map_node:main',
            'pose_pub_node = height_map.pose_pub_node:main',
        ],
    },
)
