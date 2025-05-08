#!/usr/bin/env python3
import carla
import time

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    bp = world.get_blueprint_library().filter('vehicle.tesla.model3')[0]
    bp.set_attribute('role_name', 'leader')
    # véletlenszerű spawn
    spawn_pt = world.get_map().get_spawn_points()[0]
    vehicle = world.try_spawn_actor(bp, spawn_pt)
    if not vehicle:
        print("Leader spawn fail")
        return

    print(f"Spawnolt leader id={vehicle.id}")
    # Autopilot
    vehicle.set_autopilot(True)
    print("Autopilot enabled on leader")

    # Keep alive
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        vehicle.destroy()
        print("Leader destroyed")

if __name__ == '__main__':
    main()
