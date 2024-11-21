import rclpy
from rclpy.node import Node
from carla import Client
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import argparse

class CarlaTrafficLightPublisher(Node):
    def __init__(self,host):
        super().__init__('traffic_light_publisher')
        self.publisher = self.create_publisher(String, '/carla/traffic_lights', 10)
        self.client = Client(host, 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()

        self.timer = self.create_timer(1.0, self.timer_callback)  # Adjust as needed

    def timer_callback(self):
        self.world.wait_for_tick()  # Wait for the next tick
        traffic_lights = self.world.get_actors().filter('*traffic_light')

        traffic_light_info = []
        for light in traffic_lights:
            light_state = str(light.get_state())
            transform = light.get_transform()
            location = transform.location
            pose = Pose()
            pose.position.x = location.x
            pose.position.y = location.y
            pose.position.z = location.z
            
            pose.orientation.z = transform.rotation.yaw

            traffic_light_info.append(f"ID: {light.id}, State: {light_state}, Pose: {pose}")

        # Publish the aggregated traffic light information as a single string
        msg = String()
        msg.data = "\n".join(traffic_light_info)
        self.publisher.publish(msg)
        self.get_logger().info('Published:\n' + msg.data)

def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', type=str, default='localhost',help='CARLA server IP address.')
    args = parser.parse_args()
    node = CarlaTrafficLightPublisher(host=args.host)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

