from setuptools import setup

package_name = 'my_pub_sub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/pub_sub_launch.xml',
            'launch/service_client_launch.xml'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Examples of ROS2 publishers, subscribers, services, and clients in Python',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher_counter = my_pub_sub.publisher_counter:main',
            'subscriber_counter = my_pub_sub.subscriber_counter:main',
            'square_turtle = my_pub_sub.square_turtle:main',
            'service = my_service_pkg.service:main',
            'client = my_service_pkg.client:main',
        ],
    },
)
