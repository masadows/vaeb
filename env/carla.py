import vaeb.env.carla as carla
import numpy as np
import math
from vaeb.config.consts import *


class CarlaManager:
    def __init__(self):
        self.client = carla.Client(CARLA_HOST, CARLA_PORT)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.tm = self.client.get_trafficmanager(TM_PORT_OFFSET)
        self.tm_port = self.tm.get_port()

        self.vehicle = None
        self.camera = None
        self.image_data = {"image": np.zeros((IMAGE_H, IMAGE_W, 3), dtype=np.uint8)}

    def spawn_vehicle(self):
        vehicle_bp = self.world.get_blueprint_library().filter("grandtourer")[0]

        # Punkt startowy z notebooka
        point_a = carla.Location(x=-64.599998, y=24.500000, z=1.000000)

        # Logika szukania wolnego miejsca
        offsets = [(0, 0), (1, 0), (-1, 0), (0, 1), (0, -1), (2, 0), (-2, 0)]
        for dx, dy in offsets:
            loc = carla.Location(x=point_a.x + dx, y=point_a.y + dy, z=point_a.z)
            transform = carla.Transform(loc, carla.Rotation(yaw=0))
            try:
                self.vehicle = self.world.spawn_actor(vehicle_bp, transform)
                print(f"Vehicle spawned at: {transform.location}")
                break
            except RuntimeError:
                continue

        if self.vehicle is None:
            raise RuntimeError("Nie udało się zespawnować pojazdu.")

        # Konfiguracja Traffic Managera dla pojazdu
        self.vehicle.set_autopilot(True, self.tm_port)
        self.tm.ignore_lights_percentage(self.vehicle, 100.0)
        self.tm.ignore_vehicles_percentage(self.vehicle, 100.0)
        self.tm.ignore_walkers_percentage(self.vehicle, 100.0)
        self.tm.distance_to_leading_vehicle(self.vehicle, 0.0)
        self.tm.update_vehicle_lights(self.vehicle, True)

    def spawn_camera(self):
        camera_bp = self.world.get_blueprint_library().find("sensor.camera.rgb")
        camera_bp.set_attribute("image_size_x", str(IMAGE_W))
        camera_bp.set_attribute("image_size_y", str(IMAGE_H))

        tr = carla.Transform(carla.Location(z=CAMERA_POS_Z, x=CAMERA_POS_X))
        self.camera = self.world.spawn_actor(camera_bp, tr, attach_to=self.vehicle)
        self.camera.listen(self._camera_callback)

    def _camera_callback(self, image):
        self.image_data["image"] = np.reshape(
            np.copy(image.raw_data), (image.height, image.width, 4)
        )[:, :, :3]

    def get_image(self):
        return self.image_data["image"]

    def get_vehicle_speed(self):
        if self.vehicle:
            v = self.vehicle.get_velocity()
            return 3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)  # km/h
        return 0.0

    def cleanup(self):
        if self.camera:
            self.camera.stop()
            self.camera.destroy()
        if self.vehicle:
            self.vehicle.destroy()
