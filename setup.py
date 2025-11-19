from setuptools import find_packages, setup

package_name = 'line_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/line_follower_launch.py']),
        ('share/' + package_name + '/config', ['config/line_follower_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='webot',
    maintainer_email='20080534@tafe.wa.edu.au',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'avoid_wall = line_follower.line_follower_node:main'
        ],
    },
)
