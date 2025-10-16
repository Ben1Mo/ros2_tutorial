from setuptools import find_packages, setup

package_name = 'py_pubsub_pkg'

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
    maintainer='mbe',
    maintainer_email='benchatm@gmail.com',
    description='Simple rclpy publisher example',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'parrot = py_pubsub_pkg.talker:main',
            'spy = py_pubsub_pkg.listener:main',
            'custom_parrot = py_pubsub_pkg.talker_custom:main',
            'custom_spy = py_pubsub_pkg.listener_custom:main',
        ],
    },
)