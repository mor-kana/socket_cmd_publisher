from setuptools import setup

package_name = 'socket_cmd_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nasc',
    maintainer_email='nasc@dummy.com',
    description='Receive socket data and publish UInt8 on cmd_code',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'socket_cmd_publisher = socket_cmd_publisher.socket_cmd_publisher:main',
        ],
    },
)