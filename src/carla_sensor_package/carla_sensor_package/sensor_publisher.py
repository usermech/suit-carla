import carla
import argparse
import random
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Float32
import cv2
from cv_bridge import CvBridge

class CustomTimer:
    def __init__(self):
        try:
            self.timer = time.perf_counter
        except AttributeError:
            self.timer = time.time

    def time(self):
        return self.timer()

class CarlaSensorPublisher(Node):
    def __init__(self):
        super().__init__('carla_sensor_publisher')
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, 'carla/image', 10)
        self.lidar_pub = self.create_publisher(PointCloud2, 'carla/lidar', 10)
        self.radar_pub = self.create_publisher(Float32, 'carla/radar', 10)
        
    def publish_image(self, image_data):
        # Convert CARLA image to OpenCV format
        array = np.frombuffer(image_data.raw_data, dtype=np.uint8)
        array = np.reshape(array, (image_data.height, image_data.width, 4))
        array = array[:, :, :3]  # Remove alpha channel
        cv_image = array[:, :, ::-1]  # Convert RGB to BGR for OpenCV
        
        ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.image_pub.publish(ros_image)

    def publish_lidar(self, lidar_data):
        # Process the lidar data into ROS PointCloud2 format
        points = np.frombuffer(lidar_data.raw_data, dtype=np.float32)
        points = np.reshape(points, (len(points) // 4, 4))
        
        # Here we create a custom message as PointCloud2 format
        ros_lidar = PointCloud2()
        # Fill out fields for PointCloud2 based on sensor readings

        # Publishing placeholder message, add actual PointCloud2 conversion for ROS if needed
        self.lidar_pub.publish(ros_lidar)
        
    def publish_radar(self, radar_data):
        # Radar data has range, azimuth, altitude, and velocity values
        # For simplicity, we can calculate and publish the average range of detected objects
        points = np.frombuffer(radar_data.raw_data, dtype=np.float32)
        points = np.reshape(points, (len(points) // 4, 4))
        avg_range = np.mean(np.sqrt(points[:, 0] ** 2 + points[:, 1] ** 2 + points[:, 2] ** 2))
        
        range_msg = Float32()
        range_msg.data = avg_range
        self.radar_pub.publish(range_msg)
        
    def publish_sensor_data(self, sensor_type, data):
        if sensor_type == 'image':
            self.publish_image(data)
        elif sensor_type == 'lidar':
            self.publish_lidar(data)
        elif sensor_type == 'radar':
            self.publish_radar(data)

class SensorManager:
    def __init__(self, world, publisher, sensor_type, transform, sensor_options):
        self.publisher = publisher
        self.sensor = self.init_sensor(world, sensor_type, transform, sensor_options)
        self.sensor.listen(self.process_sensor_data)

    def init_sensor(self, world, sensor_type, transform, sensor_options):
        bp_library = world.get_blueprint_library()
        if sensor_type == 'RGBCamera':
            camera_bp = bp_library.find('sensor.camera.rgb')
            camera_bp.set_attribute('image_size_x', str(sensor_options['image_size_x']))
            camera_bp.set_attribute('image_size_y', str(sensor_options['image_size_y']))
            return world.spawn_actor(camera_bp, transform)

        elif sensor_type == 'LiDAR':
            lidar_bp = bp_library.find('sensor.lidar.ray_cast')
            for key in sensor_options:
                lidar_bp.set_attribute(key,sensor_options[key])           
            return world.spawn_actor(lidar_bp, transform)

        elif sensor_type == 'Radar':
            radar_bp = bp_library.find('sensor.other.radar')
            radar_bp.set_attribute('horizontal_fov', sensor_options['horizontal_fov'])
            radar_bp.set_attribute('vertical_fov', sensor_options['vertical_fov'])
            return world.spawn_actor(radar_bp, transform)

    def process_sensor_data(self, data):
        if isinstance(data, carla.Image):
            self.publisher.publish_sensor_data('image', data)
        elif isinstance(data, carla.LidarMeasurement):
            self.publisher.publish_sensor_data('lidar', data)
        elif isinstance(data, carla.RadarMeasurement):
            self.publisher.publish_sensor_data('radar', data)

def run_simulation(args, client, publisher):
    world = client.get_world()
    sensor_manager = None
    try:
        # Set synchronous mode if specified
        if args.sync:
            traffic_manager = client.get_trafficmanager(8000)
            settings = world.get_settings()
            traffic_manager.set_synchronous_mode(True)
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            world.apply_settings(settings)

        # Spawn sensors
        camera_transform = carla.Transform(carla.Location(x=-47,y=15, z=4),carla.Rotation(pitch=-15.0,yaw=-90.0))
        camera_options = {'image_size_x': '800', 'image_size_y': '600'}
        sensor_manager = SensorManager(world, publisher, 'RGBCamera', camera_transform, camera_options)
        
        lidar_transform = carla.Transform(carla.Location(x=-47,y=15, z=4),carla.Rotation(pitch=-15.0,yaw=-90.0))
        lidar_options = {'channels': '125', 'range': '200', 'points_per_second': '750000', 'rotation_frequency': '20','horizontal_fov':'120','upper_fov':'12.5','lower_fov':'-12.5'}
        SensorManager(world, publisher, 'LiDAR', lidar_transform, lidar_options)
        '''
        radar_transform = carla.Transform(carla.Location(x=0, z=2.4))
        radar_options = {'horizontal_fov': '30', 'vertical_fov': '10'}
        SensorManager(world, publisher, 'Radar', radar_transform, radar_options)
        '''
        while rclpy.ok():
            if args.sync:
                world.tick()
            else:
                world.wait_for_tick()
            rclpy.spin_once(publisher)

    finally:
        if sensor_manager:
            sensor_manager.sensor.destroy()
        world.apply_settings(world.get_settings())

def main():
    rclpy.init()
    parser = argparse.ArgumentParser(description='CARLA Sensor ROS 2 Publisher')
    parser.add_argument('--host', default='localhost', help='Host IP of the CARLA server')
    parser.add_argument('--port', type=int, default=2000, help='Port of the CARLA server')
    parser.add_argument('--sync', action='store_true', help='Enable synchronous mode')
    args = parser.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)

    publisher = CarlaSensorPublisher()
    run_simulation(args, client, publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
