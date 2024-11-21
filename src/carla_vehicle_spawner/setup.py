from setuptools import setup

package_name = 'carla_vehicle_spawner'

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
    maintainer='carlauser',
    maintainer_email='carlauser@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spawn_vehicle_service = carla_vehicle_spawner.spawn_vehicle_service:main',
            'spawn_bicycle = carla_vehicle_spawner.spawn_bicycle:main',
            'spawn_pedestrian = carla_vehicle_spawner.spawn_pedestrian:main',
            'spawn_road_hog = carla_vehicle_spawner.spawn_road_hog:main'
,       ],
    },
)
