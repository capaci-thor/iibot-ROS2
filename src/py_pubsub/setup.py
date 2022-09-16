from setuptools import setup

package_name = 'py_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    #install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Eduardo Ortega',
    maintainer_email='rotjeot@gmail.com',
    description='Nodes for move the robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = py_pubsub.publisher_member_function:main',
            'listener = py_pubsub.subscriber_member_function:main',
            'move = py_pubsub.move_motors_subscriber:main',
            'dir = py_pubsub.direction_publisher:main',
            'gyro = py_pubsub.publisher_gyro:main',
            'acel = py_pubsub.publisher_acel:main',
        ],
    },
)
