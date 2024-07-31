from setuptools import find_packages, setup

package_name = 'tactile_examples'

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
    maintainer='carson',
    maintainer_email='carson.kohlbrenner@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'contact_locations_client = tactile_examples.contact_locations_client:main',
            'rand_touch_pos_pub = tactile_examples.rand_touch_pos_pub:main',
            'rand_touch_data_pub = tactile_examples.rand_touch_data_pub:main',
            'rand_ee_teleop_pub = tactile_examples.rand_ee_teleop_pub:main',
            'set_spawn_dist_pub = tactile_examples.set_spawn_dist_pub:main',
        ],
    },
)
