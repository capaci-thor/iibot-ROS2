from setuptools import setup

package_name = 'py_pubsensors'

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
    description='Nodes scan sensors of robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gyro = py_pubsensors.publisher_gyro:main',
            'acel = py_pubsensors.publisher_acel:main',
            'l_count = py_pubsensors.publisher_left_encoder:main',
            'r_count = py_pubsensors.publisher_right_encoder:main',
            'encoders = py_pubsensors.publisher_encoders:main',
        ],
    },
)
