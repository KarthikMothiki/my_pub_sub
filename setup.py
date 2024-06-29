from setuptools import setup

package_name = 'my_pub_sub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Examples of minimal publisher/subscriber using rclpy',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher_node = my_pub_sub.publisher_node:main',
            'subscriber_node = my_pub_sub.subscriber_node:main',
            'publisher_counter = my_pub_sub.publisher_counter:main',
            'subscriber_counter = my_pub_sub.subscriber_counter:main',
        ],
    },
)
