from setuptools import setup

package_name = 'carla_sensor_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Umut Kurt',
    maintainer_email='umutkrt98@gmail.com',
    description='Package for handling CARLA sensor messages',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'carla_sensor_publisher = carla_sensor_package.sensor_publisher:main',
            'carla_image_subscriber = carla_sensor_package.image_subscriber:main',
            ],
    },
)
