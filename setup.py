from setuptools import setup

package_name = 'my_pub_sub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Karthik',
    maintainer_email='your_email@example.com',
    description='Examples of publisher/subscriber using rclpy',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_pub = my_pub_sub.my_pub:main',
            'my_sub = my_pub_sub.my_sub:main',
        ],
    },
)
