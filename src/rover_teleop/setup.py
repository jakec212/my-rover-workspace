from setuptools import find_packages, setup

package_name = 'rover_teleop'

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
    maintainer='jakec21',
    maintainer_email='Jcardon12@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'joy_to_twist = rover_teleop.joy_to_twist:main',
            'old_science_mod_node = rover_teleop.old_science_mod_controller:main',
        ],
    },
)