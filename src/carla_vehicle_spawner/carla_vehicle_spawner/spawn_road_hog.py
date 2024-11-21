import rclpy
from rclpy.node import Node
import carla

class TrafficManagerNode(Node):
    def __init__(self):
        super().__init__('traffic_manager_node')

        # Initialize CARLA client and traffic manager
        self.TM_PORT = 4050
        self.declare_parameter('host','localhost')
        self.host = self.get_parameter('host').value
        self.client = carla.Client(self.host, 2000)
        self.declare_parameter('mode','ignore_lights')
        self.mode = self.get_parameter('mode').value
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.synchronous_master = False

        self.traffic_manager = self.client.get_trafficmanager(self.TM_PORT)
        self.setup_synchronous_mode()

        self.spawn_vehicle()

        # Timer for regular world ticks
        self.timer = self.create_timer(0.05, self.tick_world)

    def setup_synchronous_mode(self):
        settings = self.world.get_settings()
        self.traffic_manager.set_synchronous_mode(True)

        if not settings.synchronous_mode:
            self.synchronous_master = True
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            self.world.apply_settings(settings)

    def spawn_vehicle(self):
        vehicle_bp = self.world.get_blueprint_library().filter('*vehicle*')
        
        # Define spawn point for lane 2
        spawn_point = carla.Transform()
        spawn_point.location.x, spawn_point.location.y = -50, 45
        spawn_point.rotation.yaw = 180

        self.car = self.world.spawn_actor(vehicle_bp[0], spawn_point)
        
        if self.mode == 'wrong_lane':
            self.traffic_manager.vehicle_lane_offset(self.car,-15)
        else:
            # Configure traffic manager settings
            self.traffic_manager.ignore_lights_percentage(self.car, 100)
            self.traffic_manager.ignore_signs_percentage(self.car, 100)
            self.car.set_autopilot(True, self.TM_PORT)


    def tick_world(self):
        if self.synchronous_master:
            self.world.tick()
        else:
            self.world.wait_for_tick()

    def destroy_vehicle(self):
        if self.synchronous_master:
            settings = self.world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            self.world.apply_settings(settings)
        
        if self.car.is_alive:
            self.car.destroy()
        self.get_logger().info('Vehicle destroyed.')

def main(args=None):
    rclpy.init(args=args)
    traffic_manager_node = TrafficManagerNode()

    try:
        rclpy.spin(traffic_manager_node)
    except KeyboardInterrupt:
        pass
    finally:
        traffic_manager_node.destroy_vehicle()
        traffic_manager_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

