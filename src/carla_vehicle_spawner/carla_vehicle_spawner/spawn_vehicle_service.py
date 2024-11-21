import rclpy
from rclpy.node import Node
from carla_interface.srv import SpawnVehicle  
import carla
import random
import argparse

TM_PORT = 4050

class CarlaVehicleSpawner(Node):
    def __init__(self, host):
        super().__init__('carla_vehicle_spawner')
        self.srv = self.create_service(SpawnVehicle, 'spawn_vehicle', self.spawn_vehicle_callback)
        self.client = carla.Client(host, 2000)  # Use the provided host address
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.traffic_manager = self.client.get_trafficmanager(TM_PORT)
        self.traffic_manager.set_synchronous_mode(True)

    def spawn_vehicle_callback(self, request, response):
        spawn_point = carla.Transform()
        spawn_point.location.x, spawn_point.location.y, spawn_point.location.z = 100, 17, 0.1
        spawn_point.rotation.yaw = 180

        vehicle_bp = None
        if request.vehicle_type == 'random':
            vehicle_bp = random.choice(self.world.get_blueprint_library().filter('vehicle.*'))
        elif request.vehicle_type == 'firetruck':
            vehicle_bp = self.world.get_blueprint_library().filter('*firetruck*')[0]
        elif request.vehicle_type == 'ambulance':
            vehicle_bp = self.world.get_blueprint_library().filter('*ambulance*')[0]
        elif request.vehicle_type == 'police':
            vehicle_bp = self.world.get_blueprint_library().filter('*police_2020*')[0]
        else:
            response.success = False
            response.message = 'Invalid vehicle type.'
            return response

        vehicle = self.world.spawn_actor(vehicle_bp, spawn_point)
        vehicle.set_autopilot(True, TM_PORT)
        response.success = True
        response.message = f'Spawned {request.vehicle_type} with ID {vehicle.id}.'
        return response

def main(args=None):
    parser = argparse.ArgumentParser(description='CARLA Vehicle Spawner Node')
    parser.add_argument('--host', type=str, default='localhost', help='CARLA server host address')
    parsed_args = parser.parse_args()

    rclpy.init(args=args)
    spawner = CarlaVehicleSpawner(parsed_args.host)
    rclpy.spin(spawner)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

