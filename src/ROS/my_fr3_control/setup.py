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
            'projected_gradient_pos = my_fr3_control.projected_gradient_pos:main',
            'projected_gradient_ori = my_fr3_control.projected_gradient_ori:main',
            'projected_gradient_acc_ori = my_fr3_control.projected_gradient_acc_ori:main',
            'jacobian_computation_node = my_fr3_control.jacobian_computation_node:main',
    
        ],
    },
    # scripts=['scripts/random_joint_publisher_fr3.py'],

)
