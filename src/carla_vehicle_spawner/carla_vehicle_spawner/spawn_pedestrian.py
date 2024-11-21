import rclpy
from rclpy.node import Node
import carla
import random
import logging

class PedestrianController(Node):
    def __init__(self):
        super().__init__('pedestrian_controller')
        self.declare_parameter('host','localhost')
        self.host = self.get_parameter('host').value
        self.client = carla.Client(self.host, 2000)       
        self.declare_parameter('number_of_walkers',10)
        self.number_of_walkers = self.get_parameter('number_of_walkers').value
        self.percentagePedestriansRunning = 0.0
        self.percentagePedestriansCrossing = 1.0
        self.vehicles_list = []
        self.walkers_list = []
        self.all_id = []
        self.spawn_points = []
        self.synchronous_master = False

        self.client.set_timeout(10.0)
        self.world = self.client.get_world()

        self.setup_synchronous_mode()
        self.spawn_pedestrians()

        # Timer to tick the world regularly
        self.timer = self.create_timer(0.05, self.tick_world)

    def setup_synchronous_mode(self):
        settings = self.world.get_settings()
        if not settings.synchronous_mode:
            self.synchronous_master = True
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            self.world.apply_settings(settings)

    def spawn_pedestrians(self):
        blueprintsWalkers = self.world.get_blueprint_library().filter("walker.pedestrian.*")

        # Generate spawn points
        while len(self.spawn_points) <= 60:
            spawn_point = carla.Transform()
            spawn_point.location = self.world.get_random_location_from_navigation()
            if spawn_point.location:
                x, y = spawn_point.location.x, spawn_point.location.y
                if -70 <= x <= -20 and -5 <= y <= 45:
                    self.spawn_points.append(spawn_point)

        # Batch spawn pedestrians
        SpawnActor = carla.command.SpawnActor
        batch = []
        walker_speed = []
        for spawn_point in self.spawn_points:
            walker_bp = random.choice(blueprintsWalkers)
            if walker_bp.has_attribute('is_invincible'):
                walker_bp.set_attribute('is_invincible', 'false')

            if walker_bp.has_attribute('speed'):
                if random.random() > self.percentagePedestriansRunning:
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
                else:
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[2])
            else:
                walker_speed.append(0.0)
            batch.append(SpawnActor(walker_bp, spawn_point))

        results = self.client.apply_batch_sync(batch, True)
        walker_speed2 = []
        spawn_count = 0
        for i, result in enumerate(results):
            if result.error:
                logging.error(result.error)
            else:
                spawn_count += 1
                self.walkers_list.append({"id": result.actor_id})
                walker_speed2.append(walker_speed[i])
            if spawn_count >= self.number_of_walkers:
                last_walker = i+1
                break

        self.client.apply_batch([carla.command.DestroyActor(x.actor_id) for x in results[last_walker:]])

        walker_speed = walker_speed2
        batch = []
        # Spawn walker controllers
        walker_controller_bp = self.world.get_blueprint_library().find('controller.ai.walker')
        batch = [SpawnActor(walker_controller_bp, carla.Transform(), walker["id"]) for walker in self.walkers_list]
        results = self.client.apply_batch_sync(batch, True)
        for i, result in enumerate(results):
            if result.error:
                logging.error(result.error)
            else:
                self.walkers_list[i]["con"] = result.actor_id

        self.all_id = [item for sublist in [(walker["con"], walker["id"]) for walker in self.walkers_list] for item in sublist]
        self.all_actors = self.world.get_actors(self.all_id)

        self.world.set_pedestrians_cross_factor(self.percentagePedestriansCrossing)
        
        for i in range(0, len(self.all_id), 2):
            self.all_actors[i].start()
            self.all_actors[i].go_to_location(self.world.get_random_location_from_navigation())
            self.all_actors[i].set_max_speed(float(walker_speed[int(i / 2)]))

        self.get_logger().info(f'{len(self.walkers_list)} pedestrians are spawned.')

    def tick_world(self):
        if self.synchronous_master:
            self.world.tick()
        else:
            self.world.wait_for_tick()

    def shutdown(self):
        if self.synchronous_master:
            settings = self.world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            self.world.apply_settings(settings)
        
        for i in range(0, len(self.all_id), 2):
            self.all_actors[i].stop()
        self.client.apply_batch([carla.command.DestroyActor(x) for x in self.all_id])
        self.get_logger().info(f'Destroyed {len(self.walkers_list)} pedestrians.')

def main(args=None):
    rclpy.init(args=args)
    pedestrian_controller = PedestrianController()
    try:
        rclpy.spin(pedestrian_controller)
    except KeyboardInterrupt:
        pass
    finally:
        pedestrian_controller.shutdown()
        pedestrian_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
