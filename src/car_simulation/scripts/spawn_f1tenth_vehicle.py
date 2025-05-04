#!/usr/bin/env python3
import carla, time

def main():
    params = {
        'mass': 3.74,
        'cog_z': 0.045,
        'wheel_radius': 0.0534,
        'wheel_width': 0.045,
        'tire_friction': 0.66,
        'drag_coefficient': 0.3,
    }

    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    bp = world.get_blueprint_library().filter('vehicle.audi.tt')[0]

    spawn_points = world.get_map().get_spawn_points()
    vehicle = None
    for transform in spawn_points:
        vehicle = world.try_spawn_actor(bp, transform)
        if vehicle:
            break
    if vehicle is None:
        print("Spawn failed: minden spawn-pont foglalt")
        return

    print(f"Spawned {vehicle.type_id} (id: {vehicle.id}) with custom F1TENTH params")

    phys = vehicle.get_physics_control()
    phys.mass = params['mass']
    phys.center_of_mass.z_offset = params['cog_z']
    phys.drag_coefficient = params['drag_coefficient']
    for wheel in phys.wheels:
        wheel.radius        = params['wheel_radius']
        wheel.width         = params['wheel_width']
        wheel.tire_friction = params['tire_friction']
    vehicle.apply_physics_control(phys)

    vehicle.apply_control(carla.VehicleControl(throttle=0.5, steer=0.0))
    time.sleep(5)
    vehicle.destroy()

if __name__ == '__main__':
    main()
