import vaeb.env.carla as carla
import numpy as np
from .carla import CarlaManager
from vaeb.config import IMAGE_W, IMAGE_H
import cv2
import os


class CarlaTestManager(CarlaManager):
    def __init__(self):
        super().__init__()
        self.obstacle = None
        self.collision_sensor = None
        self.collision_history = []

        self.video_writer = None
        self.is_recording = False
        self.weather_type = ""
        self.other_obstacles = []

    def reset(self):
        """Czyści stan testu, usuwa aktory testowe, ale zachowuje połączenie."""
        self.cleanup_test_actors()
        self.collision_history = []
        self.image_data = {"image": np.zeros((IMAGE_H, IMAGE_W, 3), dtype=np.uint8)}

    def spawn_static_obstacle(self, distance=40.0):
        """Tworzy przeszkodę (pojazd) w zadanej odległości przed maską."""
        if not self.vehicle:
            print("Błąd: Brakuje sterowanego pojazdu.")
            return

        ego_trans = self.vehicle.get_transform()
        ego_loc = ego_trans.location
        ego_vec = ego_trans.get_forward_vector()

        target_loc = carla.Location(
            x=ego_loc.x + ego_vec.x * distance,
            y=ego_loc.y + ego_vec.y * distance,
            z=ego_loc.z,
        )
        target_trans = carla.Transform(target_loc, ego_trans.rotation)

        obs_bp = self.world.get_blueprint_library().filter("vehicle.tesla.model3")[0]
        self.obstacle = self.world.try_spawn_actor(obs_bp, target_trans)

        if self.obstacle:
            control = carla.VehicleControl()
            control.hand_brake = True
            self.obstacle.apply_control(control)
            print(f"[TEST] Przeszkoda zespawnowana {distance}m przed pojazdem.")
        else:
            print("[TEST] Błąd spawnowania przeszkody.")

    def spawn_actor(self, blueprint_name, transform, role="obstacle", isOther=False):
        """Generyczna metoda do spawnowania aktora (auto, pieszy, rowerzysta)."""
        bp = self.world.get_blueprint_library().find(blueprint_name)
        if bp.has_attribute("is_invincible"):
            bp.set_attribute("is_invincible", "false")

        actor = None
        z_offsets = [0.0, 0.2, 0.5, 1.0, 2.0]
        original_z = transform.location.z

        for offset in z_offsets:
            transform.location.z = original_z + offset
            if "walker" in blueprint_name:
                transform.location.z += 1.0

            actor = self.world.try_spawn_actor(bp, transform)
            if actor is not None:
                if offset > 0:
                    print(
                        f"[SPAWN INFO] Przesunięto spawnowanie o {offset}m w górę (kolizja)."
                    )
                break

        if actor is not None:
            if isOther:
                self.other_obstacles.append(actor)
            if "vehicle" in blueprint_name:
                pass

            self.world.tick()
            loc = actor.get_location()
            print(
                f"[TEST] Zespawnowano {blueprint_name} jako {role} w: X={loc.x:.2f}, Y={loc.y:.2f}, Z={loc.z:.2f}"
            )
        else:
            print(
                f"[TEST ERROR] Nie udało się zespawnować {blueprint_name} w {transform.location} po {len(z_offsets)} próbach."
            )
        return actor

    def spawn_vehicle(self, spawn_transform=None):
        """
        Spawnuje pojazd.
        :param spawn_transform: (Opcjonalny) carla.Transform - konkretne miejsce startu.
                                Jeśli None, funkcja poszuka wolnego miejsca wg starej logiki.
        """
        vehicle_bp = self.world.get_blueprint_library().filter("grandtourer")[0]
        if spawn_transform is not None:
            self.vehicle = self.world.try_spawn_actor(vehicle_bp, spawn_transform)
            if self.vehicle is None:
                spawn_transform.location.z += 0.5
                spawn_transform.location.y += 0.1
                self.vehicle = self.world.try_spawn_actor(vehicle_bp, spawn_transform)
        else:
            point_a = carla.Location(x=-64.599998, y=24.500000, z=1.000000)
            offsets = [(0, 0), (1, 0), (-1, 0), (0, 1), (0, -1), (2, 0), (-2, 0)]

            for dx, dy in offsets:
                loc = carla.Location(x=point_a.x + dx, y=point_a.y + dy, z=point_a.z)
                transform = carla.Transform(loc, carla.Rotation(yaw=0))
                try:
                    self.vehicle = self.world.spawn_actor(vehicle_bp, transform)
                    print(f"pojazd zespawnowany w: {transform.location}")
                    break
                except RuntimeError:
                    continue

        if self.vehicle is None:
            raise RuntimeError(
                "Nie udało się zespawnować pojazdu (Spawn point zablokowany)."
            )

        self.world.tick()
        self.vehicle.set_autopilot(False, self.tm_port)
        self.tm.ignore_lights_percentage(self.vehicle, 100.0)
        self.tm.ignore_vehicles_percentage(self.vehicle, 100.0)
        self.tm.ignore_walkers_percentage(self.vehicle, 100.0)
        self.tm.distance_to_leading_vehicle(self.vehicle, 0.0)
        self.tm.update_vehicle_lights(self.vehicle, True)

    def set_vehicle_speed(self, speed_kmh):
        """
        Wymusza natychmiastową prędkość pojazdu (fizyka).
        :param speed_kmh: Prędkość w km/h
        """
        if not self.vehicle:
            return
        speed_ms = speed_kmh / 3.6

        transform = self.vehicle.get_transform()
        fwd_vector = transform.get_forward_vector()

        velocity = carla.Vector3D(
            x=fwd_vector.x * speed_ms,
            y=fwd_vector.y * speed_ms,
            z=fwd_vector.z * speed_ms,
        )
        self.vehicle.set_target_velocity(velocity)
        print(f"[TEST] Ustawiono prędkość początkową: {speed_kmh} km/h")

    def attach_collision_sensor(self):
        """Podłącza czujnik kolizji tylko na potrzeby testu."""
        if not self.vehicle:
            return

        col_bp = self.world.get_blueprint_library().find("sensor.other.collision")
        self.collision_sensor = self.world.spawn_actor(
            col_bp, carla.Transform(), attach_to=self.vehicle
        )
        self.collision_sensor.listen(lambda event: self._on_collision(event))

    def _on_collision(self, event):
        """Callback kolizji."""
        if not event.other_actor.type_id.startswith("static"):
            if len(self.collision_history) <= 0:
                print(f"!!! KOLIZJA z {event.other_actor.type_id} !!!")
            self.collision_history.append(event)

    def get_distance_to_obstacle(self):
        """Zwraca rzeczywisty dystans (w metrach) między środkami pojazdów."""
        if self.vehicle and self.obstacle:
            loc_ego = self.vehicle.get_location()
            loc_obs = self.obstacle.get_location()
            return loc_ego.distance(loc_obs)
        return None

    def cleanup_test_actors(self):
        """Usuwa tylko obiekty specyficzne dla testu oraz pojazd ego."""
        if self.collision_sensor:
            self.collision_sensor.stop()
            self.collision_sensor.destroy()
            self.collision_sensor = None

        if self.camera:
            self.camera.stop()
            self.camera.destroy()
            self.camera = None

        if self.vehicle:
            self.vehicle.destroy()
            self.vehicle = None

        if self.obstacle:
            self.obstacle.destroy()
            self.obstacle = None

        if self.other_obstacles:
            for o in self.other_obstacles:
                if o is not None:
                    try:
                        if o.is_alive:
                            o.destroy()
                    except RuntimeError:
                        pass
            self.other_obstacles.clear()
        self.world.tick()
        self.world.tick()

    def destroy_all_spawned_actors(self):
        actors = self.world.get_actors()
        vehicles = actors.filter("vehicle.*")
        walkers = actors.filter("walker.*")
        sensors = actors.filter("sensor.*")
        controllers = actors.filter("controller.*")

        to_destroy = list(vehicles) + list(walkers) + list(sensors) + list(controllers)

        if not to_destroy:
            return

        print(f"[CLEANUP] Usuwanie {len(to_destroy)} obiektów z mapy...")

        batch = [carla.command.DestroyActor(x) for x in to_destroy]
        self.client.apply_batch(batch)

        try:
            self.world.tick()
        except Exception:
            pass

    def start_recording(self, filename):
        """Rozpoczyna nagrywanie do pliku."""
        if not os.path.exists("test_recordings"):
            os.makedirs("test_recordings")

        filepath = os.path.join("test_recordings", self.weather_type + filename)

        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        fps = 20.0

        self.video_writer = cv2.VideoWriter(filepath, fourcc, fps, (IMAGE_W, IMAGE_H))
        if not self.video_writer.isOpened():
            print("[REC ERROR] Kodek mp4v zawiódł.")
            self.video_writer = None
            self.is_recording = False
        else:
            self.is_recording = True

    def write_frame(self, frame_rgb):
        """
        Zapisuje klatkę do wideo.
        """
        if self.is_recording and self.video_writer is not None:
            frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
            self.video_writer.write(frame_bgr)

    def stop_recording(self):
        """Kończy nagrywanie i zwalnia plik."""
        if self.is_recording:
            if self.video_writer:
                self.video_writer.release()
            self.video_writer = None
            self.is_recording = False

    def change_weather(self):
        self.world.set_weather(carla.WeatherParameters.CloudyNoon)
        self.weather_type = "Day"
        yield
        weather = carla.WeatherParameters(
            sun_altitude_angle=-30.0, sun_azimuth_angle=90.0
        )
        self.world.set_weather(weather)
        self.weather_type = "Night"
        yield
        self.world.set_weather(carla.WeatherParameters.MidRainyNoon)
        self.weather_type = "Rain"
        yield
