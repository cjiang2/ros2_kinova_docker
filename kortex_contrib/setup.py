from setuptools import find_packages, setup

package_name = 'kortex_contrib'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/gen3.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotvision',
    maintainer_email='robotvision@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'joint_state_publisher = kortex_contrib.joint_state_publisher:main',
            'high_level_movement = kortex_contrib.high_level_movement:main',
        ],
    },
)
