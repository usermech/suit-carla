import rclpy
from rclpy.node import Node
import carla
import argparse

class CarlaController(Node):
    def __init__(self,host):
        super().__init__('carla_controller')
        self.client = carla.Client(host, 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.vehicle_bp_lib = self.world.get_blueprint_library().filter('*vehicle*')
        self.bicycles = [x for x in self.vehicle_bp_lib if x.get_attribute('base_type').as_str() == 'bicycle']
        self.spawn_point = carla.Transform()
        self.spawn_point.location.x, self.spawn_point.location.y = -60, 5
        self.synchronous_master = False

        self.setup_synchronous_mode()
        self.spawn_bicycle()

        # Initialize the loop timer
        self.timer = self.create_timer(0.05, self.control_loop)
        self.counter = 0

    def setup_synchronous_mode(self):
        settings = self.world.get_settings()
        if not settings.synchronous_mode:
            self.synchronous_master = True
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            self.world.apply_settings(settings)

    def normalize_angle(self, angle):
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle

    def spawn_bicycle(self):
        try:
            self.my_bike = self.world.spawn_actor(self.bicycles[0], self.spawn_point)
        except Exception as e:
            self.get_logger().error(f"Failed to spawn bicycle: {e}")

    def turn_right(self, initial_yaw):
        target_angle = initial_yaw + 90
        while abs(self.normalize_angle(target_angle - self.my_bike.get_transform().rotation.yaw)) >= 14:
            self.my_bike.apply_ackermann_control(carla.VehicleAckermannControl(steer=30, speed=1))
            if self.synchronous_master:
                self.world.tick()
            else:
                self.world.wait_for_tick()

    def move_forward(self):
        self.my_bike.apply_ackermann_control(carla.VehicleAckermannControl(steer=0, speed=1))

    def control_loop(self):
        self.counter += 1
        if self.counter % 500 == 0:
            self.turn_right(self.my_bike.get_transform().rotation.yaw)
            self.counter = 1
        self.move_forward()
        if self.synchronous_master:
            self.world.tick()
        else:
            self.world.wait_for_tick()

    def destroy_bicycle(self):
        try:
            self.my_bike.destroy()
        except:
            pass

    def shutdown(self):
        self.destroy_bicycle()
        if self.synchronous_master:
            settings = self.world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            self.world.apply_settings(settings)

def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser()
    parser.add_argument('--host',type=str,default='localhost',help='CARLA server host address')
    args = parser.parse_args()
    controller = CarlaController(args.host)
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.shutdown()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

