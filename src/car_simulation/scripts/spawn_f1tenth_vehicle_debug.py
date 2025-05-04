#!/usr/bin/env python3
import carla, time, sys, traceback

def main():
    print("[DEBUG] 1) Indul a spawn szkript", flush=True)

    print("[DEBUG] 2) Kapcsolódás a CARLA szerverhez...", flush=True)
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    print("[DEBUG]  -> Kapcsolódás sikeres", flush=True)

    world = client.get_world()
    print("[DEBUG] 3) Világ lekérve", flush=True)

    library = world.get_blueprint_library().filter('vehicle.*')
    print(f"[DEBUG] 4) Blueprint-ek száma: {len(library)}", flush=True)
    bp = library[0]
    print(f"[DEBUG]  -> Használt blueprint: {bp.id}", flush=True)

    params = {
        'mass': 3.74,
        'cog_z': 0.045,
        'wheel_radius': 0.0534,
        'wheel_width': 0.045,
        'tire_friction': 0.66,
        'drag_coefficient': 0.3,
    }

    spawn_points = world.get_map().get_spawn_points()
    print(f"[DEBUG] 5) Spawn-pontok száma: {len(spawn_points)}", flush=True)

    # 6) Keressünk egy szabad pontot
    print("[DEBUG] 6) Szabad spawn-pont keresése...", flush=True)
    vehicle = None
    for i, transform in enumerate(spawn_points):
        vehicle = world.try_spawn_actor(bp, transform)
        if vehicle is not None:
            print(f"[DEBUG]  -> Spawnolva a {i}. pontnál: {transform.location}", flush=True)
            break
    if vehicle is None:
        print("[ERROR] Nem sikerült spawnolni – ütközés minden pontnál", flush=True)
        return

    # 7) Fizikai beállítások
    print("[DEBUG] 7) VehiclePhysicsControl lekérése...", flush=True)
    phys = vehicle.get_physics_control()
    phys.mass = params['mass']
    phys.center_of_mass.z_offset = params['cog_z']
    phys.drag_coefficient = params['drag_coefficient']
    for wheel in phys.wheels:
        wheel.radius        = params['wheel_radius']
        wheel.width         = params['wheel_width']
        wheel.tire_friction = params['tire_friction']
    print("[DEBUG]  -> Fizikai beállítás alkalmazása...", flush=True)
    vehicle.apply_physics_control(phys)

    # 8) Teszt-felgyorsítás
    print("[DEBUG] 8) Teszt-felgyorsítás 5 másodpercig...", flush=True)
    vehicle.apply_control(carla.VehicleControl(throttle=0.5, steer=0.0))
    time.sleep(5)

    # 9) Takarítás
    print("[DEBUG] 9) Jármű törlése...", flush=True)
    vehicle.destroy()
    print("[DEBUG] Kész!", flush=True)

if __name__ == '__main__':
    try:
        main()
    except Exception:
        traceback.print_exc(file=sys.stdout)
        sys.exit(1)
