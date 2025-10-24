from setuptools import find_packages, setup

package_name = 'maze_exploration_control'

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
    maintainer='ai',
    maintainer_email='ai@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'car_control=maze_exploration_control.car_control:main',
            'data_upload_parsing=maze_exploration_control.data_upload_parsing:main',
            'tf_pub=maze_exploration_control.tf_pub:main'
        ],
    },
)
