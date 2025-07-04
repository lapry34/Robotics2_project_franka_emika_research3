from setuptools import find_packages, setup

package_name = 'my_fr3_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/my_fr3_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kristoj',
    maintainer_email='kristar301@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'velocity_controller_node = my_fr3_control.velocity_controller_node:main',
            'acceleration_controller_node = my_fr3_control.acceleration_controller_node:main',
            'worker_node = my_fr3_control.worker_node:main',
        ],
    },

)
