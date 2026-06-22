import carla
from .base_scenario import BaseScenario
import time
import math
import numpy as np


class StationaryVehicleTest(BaseScenario):
    def __init__(self, distance, target_speed=30.0):
        super().__init__(f"Stationary Car @ {distance}m", target_speed=target_speed)
        self.target_distance = distance

    def setup(self, manager):
        ego_trans = manager.vehicle.get_transform()
        fwd = ego_trans.get_forward_vector()
        loc = ego_trans.location + carla.Location(
            fwd.x * self.target_distance, fwd.y * self.target_distance, 0
        )

        transform = carla.Transform(loc, ego_trans.rotation)
        obs = manager.spawn_actor("vehicle.tesla.model3", transform)

        if obs:
            c = carla.VehicleControl()
            c.hand_brake = True
            obs.apply_control(c)
            manager.obstacle = obs


class PedestrianCrossingTest(BaseScenario):
    def __init__(
        self,
        trigger_distance=20.0,
        walker_speed=3.5,
        spawn_distance=30.0,
        max_duration=10,
    ):
        super().__init__(
            f"Pedestrian Crossing (Trigger {trigger_distance}m)",
            max_duration=max_duration,
        )
        self.trigger_distance = trigger_distance
        self.walker = None
        self.walker_controller = None
        self.triggered = False
        self.start_loc = None
        self.end_loc = None
        self.walker_speed = walker_speed
        self.spawn_distance = spawn_distance

    def setup(self, manager):
        ego_trans = manager.vehicle.get_transform()
        fwd = ego_trans.get_forward_vector()
        right = ego_trans.get_right_vector()

        dist_forward = self.spawn_distance
        offset_right = 4.0
        offset_left = 4.0
        self.triggered = False

        self.spawn_loc = (
            ego_trans.location
            + carla.Location(fwd.x * dist_forward, fwd.y * dist_forward, 0.5)
            + carla.Location(right.x * offset_right, right.y * offset_right, 0)
        )

        target_loc = (
            ego_trans.location
            + carla.Location(fwd.x * dist_forward, fwd.y * dist_forward, 0.5)
            - carla.Location(right.x * offset_left, right.y * offset_left, 0)
        )

        dx = target_loc.x - self.spawn_loc.x
        dy = target_loc.y - self.spawn_loc.y
        length = math.sqrt(dx**2 + dy**2)

        self.direction_vector = carla.Vector3D(dx / length, dy / length, 0)

        spawn_tf = carla.Transform(self.spawn_loc, carla.Rotation())

        self.walker = manager.spawn_actor(
            "walker.pedestrian.0001", spawn_tf, role="pedestrian"
        )

        manager.obstacle = self.walker
        control = carla.WalkerControl()
        control.direction = carla.Vector3D(0, 0, 0)
        control.speed = 0.0
        self.walker.apply_control(control)

        self.walker_controller = None

    def update_behavior(self, manager, ego_vehicle):
        if self.triggered or not self.walker:
            return

        dist = manager.get_distance_to_obstacle()

        if dist and dist < self.trigger_distance:
            print(f"[SCENARIO] Pieszy wbiega na jezdnię! (Dyst: {dist:.1f}m)")
            if self.walker_controller:
                self.walker_controller.stop()
                self.walker_controller.destroy()
                self.walker_controller = None

            control = carla.WalkerControl()
            control.speed = self.walker_speed
            control.direction = self.direction_vector
            control.jump = False

            self.walker.apply_control(control)
            self.triggered = True


class FalsePositiveTest(BaseScenario):
    def __init__(self, max_duration=10):
        super().__init__("False Positive - Empty Road", max_duration=max_duration)

    def setup(self, manager):
        manager.obstacle = None
        print("[SCENARIO] Pusta droga - test niepotrzebnego hamowania.")


class LeadVehicleBrakingTest(BaseScenario):
    """
    Scenariusz: Jedziemy za innym autem. Ono nagle hamuje.
    Testuje reakcję na zmniejszający się dystans w ruchu.
    """

    def __init__(self, start_distance=20.0, target_speed=30.0, time_to_stop=5.0):
        super().__init__(
            f"Lead Vehicle Braking (Start Dist {start_distance}m)",
            target_speed=target_speed,
        )
        self.start_distance = start_distance
        self.leader = None
        self.state = "FOLLOWING"
        self.brake_start_time = None
        self.time = time_to_stop

    def setup(self, manager):
        ego_trans = manager.vehicle.get_transform()
        fwd = ego_trans.get_forward_vector()
        loc = ego_trans.location + carla.Location(
            fwd.x * self.start_distance, fwd.y * self.start_distance, 0
        )

        self.leader = manager.spawn_actor(
            "vehicle.tesla.model3",
            carla.Transform(loc, ego_trans.rotation),
            role="leader",
        )
        manager.obstacle = self.leader

        if self.leader:
            target_speed_ms = self.target_speed / 3.6
            velocity = carla.Vector3D(
                fwd.x * target_speed_ms,
                fwd.y * target_speed_ms,
                fwd.z * target_speed_ms
            )
            self.leader.set_target_velocity(velocity)
            control = carla.VehicleControl()
            control.throttle = 0.5
            control.steer = 0.0
            control.brake = 0.0
            control.hand_brake = False
            self.leader.apply_control(control)

    def update_behavior(self, manager, ego_vehicle):
        if self.state == "FOLLOWING" and time.time() > (
            self.start_time_internal + self.time
        ):
            print("[SCENARIO] Lider gwałtownie hamuje!")
            self.state = "BRAKING"
            self.leader.set_autopilot(False, manager.tm_port)
            control = carla.VehicleControl()
            control.throttle = 0.0
            control.brake = 1.0
            self.leader.apply_control(control)

    def run(self, manager, models):
        self.start_time_internal = time.time() + 0.2
        return super().run(manager, models)


class SuicidalPedestrianTest(BaseScenario):
    """
    Scenariusz: Pieszy 'Homing Missile'.
    Stoi w miejscu, a po aktywacji biegnie prosto na samochód (śledzi jego pozycję).
    Zatrzymuje się dopiero, gdy jest bardzo blisko (parametr stop_distance).
    """

    def __init__(
        self,
        trigger_distance=20.0,
        stop_distance=0.5,
        speed=30.0,
        spawn_distance=30.0,
        walker_speed=4.0,
        max_duration=10,
    ):
        super().__init__(
            f"Suicidal Homing Pedestrian (Trigger {trigger_distance}m, Stop @ {stop_distance}m)",
            target_speed=speed,
            max_duration=max_duration,
        )

        self.trigger_distance = trigger_distance
        self.stop_distance = stop_distance

        self.walker = None
        self.walker_controller = None
        self.triggered = False
        self.spawn_distance = spawn_distance
        self.walker_speed = walker_speed

    def setup(self, manager):
        ego_trans = manager.vehicle.get_transform()
        fwd = ego_trans.get_forward_vector()
        right = ego_trans.get_right_vector()

        long_dist = self.spawn_distance
        lat_dist = 2.0
        self.triggered = False

        self.spawn_loc = (
            ego_trans.location
            + carla.Location(fwd.x * long_dist, fwd.y * long_dist, 0.5)
            + carla.Location(right.x * lat_dist, right.y * lat_dist, 0)
        )

        spawn_tf = carla.Transform(self.spawn_loc, carla.Rotation())
        self.walker = manager.spawn_actor(
            "walker.pedestrian.0001", spawn_tf, role="pedestrian"
        )

        manager.obstacle = self.walker
        control = carla.WalkerControl()
        control.direction = carla.Vector3D(0, 0, 0)
        control.speed = 0.0
        self.walker.apply_control(control)

        self.walker_controller = None

    def update_behavior(self, manager, ego_vehicle):
        """
        Logika Homing Missile: W każdej klatce celuj w samochód.
        """
        if not self.walker:
            return
        car_trans = ego_vehicle.get_transform()
        car_center = car_trans.location
        car_fwd = car_trans.get_forward_vector()
        target_pos = carla.Location(
            x=car_center.x + car_fwd.x * 2,
            y=car_center.y + car_fwd.y * 2,
            z=car_center.z,
        )

        loc_ped = self.walker.get_location()
        dist = loc_ped.distance(target_pos)

        if not self.triggered:
            if dist < self.trigger_distance:
                print(f"[SCENARIO] SAMOBÓJCA: Namierzono cel! (Dyst: {dist:.1f}m)")

                if self.walker_controller:
                    self.walker_controller.stop()
                    self.walker_controller.destroy()
                    self.walker_controller = None

                self.triggered = True
            else:
                return

        control = carla.WalkerControl()
        control.jump = False

        if dist > self.stop_distance:
            dx = target_pos.x - loc_ped.x
            dy = target_pos.y - loc_ped.y

            length = math.sqrt(dx**2 + dy**2)

            if length > 0:
                direction = carla.Vector3D(dx / length, dy / length, 0.0)
                control.direction = direction
                control.speed = self.walker_speed
            else:
                control.speed = 0.0
        else:
            control.speed = 0.0

        self.walker.apply_control(control)


class OncomingTrafficTest(BaseScenario):
    """
    Scenariusz: Pojazd nadjeżdża z naprzeciwka (Oncoming), ale pasem obok.
    Cel: Sprawdzenie, czy system NIE zahamuje (False Positive check).
    """

    def __init__(
        self,
        lateral_offset=3.5,
        start_distance=60.0,
        ego_speed=30.0,
        oncoming_speed=30.0,
        max_duration=20.0,
        stop_distance=None,
        should_stop=False,
    ):
        super().__init__(
            f"Oncoming Traffic (Offset {lateral_offset}m, {ego_speed}km/h)",
            target_speed=ego_speed,
            max_duration=max_duration,
            should_stop=should_stop,
        )

        self.lateral_offset = lateral_offset
        self.start_distance = start_distance
        self.oncoming_speed = oncoming_speed
        self.oncoming_car = None
        self.stop_distance = stop_distance
        self.has_stopped = False

    def setup(self, manager):
        self.has_stopped = False
        ego_trans = manager.vehicle.get_transform()
        fwd = ego_trans.get_forward_vector()
        right = ego_trans.get_right_vector()

        spawn_loc = (
            ego_trans.location
            + carla.Location(
                fwd.x * self.start_distance, fwd.y * self.start_distance, 0
            )
            + carla.Location(
                right.x * self.lateral_offset, right.y * self.lateral_offset, 0
            )
        )

        spawn_rot = ego_trans.rotation
        spawn_rot.yaw += 180

        spawn_tf = carla.Transform(spawn_loc, spawn_rot)
        self.oncoming_car = manager.spawn_actor(
            "vehicle.ford.mustang", spawn_tf, role="oncoming"
        )
        manager.obstacle = self.oncoming_car

        if self.oncoming_car:
            self.oncoming_car.set_light_state(carla.VehicleLightState.LowBeam)
            speed_ms = self.oncoming_speed / 3.6
            vec_fwd = self.oncoming_car.get_transform().get_forward_vector()
            velocity = carla.Vector3D(
                vec_fwd.x * speed_ms, vec_fwd.y * speed_ms, vec_fwd.z * speed_ms
            )

            self.oncoming_car.set_target_velocity(velocity)
            ctrl = carla.VehicleControl()
            ctrl.throttle = 0.5
            self.oncoming_car.apply_control(ctrl)

    def set_oncoming_velocity(self, speed_kmh=None):
        """Pomocnicza funkcja do nadawania prędkości pojazdowi z naprzeciwka."""
        if speed_kmh is None:
            speed_kmh = self.oncoming_speed

        speed_ms = speed_kmh / 3.6
        vec_fwd = self.oncoming_car.get_transform().get_forward_vector()
        velocity = carla.Vector3D(
            vec_fwd.x * speed_ms, vec_fwd.y * speed_ms, vec_fwd.z * speed_ms
        )
        self.oncoming_car.set_target_velocity(velocity)

    def update_behavior(self, manager, ego_vehicle):
        """Sprawdzamy dystans i hamujemy nadjeżdżającym autem jeśli trzeba."""
        if not self.oncoming_car or self.has_stopped:
            return
        if self.stop_distance is None:
            return

        dist = manager.get_distance_to_obstacle()

        if dist is not None and dist <= self.stop_distance:
            print(
                f"[SCENARIO] Nadjeżdżający pojazd zatrzymuje się! (Dyst: {dist:.1f}m)"
            )

            self.oncoming_car.set_target_velocity(carla.Vector3D(0, 0, 0))

            ctrl = carla.VehicleControl()
            ctrl.brake = 1.0
            ctrl.hand_brake = True
            self.oncoming_car.apply_control(ctrl)

            self.has_stopped = True
        else:
            self.set_oncoming_velocity()


class CutInTest(BaseScenario):
    """
    Scenariusz 'Cut-In':
    Pojazd jedzie pasem obok (offset), przyspiesza/zrównuje się,
    a następnie gwałtownie zmienia pas przed maskę EGO i hamuje.
    """

    def __init__(
        self,
        lateral_offset=3.5,
        start_distance=15.0,
        speed=30.0,
        brake_after_cutin=True,
    ):
        super().__init__(
            f"Cut-In (StartDist {start_distance}m, Offset {lateral_offset}m)",
            target_speed=speed,
        )

        self.lateral_offset = lateral_offset
        self.start_distance = start_distance
        self.brake_after_cutin = brake_after_cutin

        self.challenger = None
        self.state = "DRIVING_PARALLEL"
        self.maneuver_start_time = None

    def setup(self, manager):
        ego_trans = manager.vehicle.get_transform()
        fwd = ego_trans.get_forward_vector()
        right = ego_trans.get_right_vector()

        spawn_loc = (
            ego_trans.location
            + carla.Location(
                fwd.x * self.start_distance, fwd.y * self.start_distance, 0
            )
            + carla.Location(
                right.x * self.lateral_offset, right.y * self.lateral_offset, 0
            )
        )

        spawn_rot = ego_trans.rotation

        spawn_tf = carla.Transform(spawn_loc, spawn_rot)
        self.challenger = manager.spawn_actor(
            "vehicle.audi.tt", spawn_tf, role="challenger"
        )

        manager.obstacle = self.challenger

        self.ego_width = manager.vehicle.bounding_box.extent.y * 2

        self.state = "DRIVING_PARALLEL"
        if self.challenger:
            self.challenger.set_light_state(carla.VehicleLightState.LowBeam)
            self.set_velocity(self.target_speed)
            self.maneuver_start_time = time.time()

    def set_velocity(self, speed_kmh, lateral_speed_ms=0.0):
        """Ustawia prędkość wzdłużną i poprzeczną."""
        if not self.challenger:
            return

        speed_ms = speed_kmh / 3.6
        tf = self.challenger.get_transform()
        fwd = tf.get_forward_vector()
        right = tf.get_right_vector()

        vel = carla.Vector3D(
            fwd.x * speed_ms - right.x * lateral_speed_ms,
            fwd.y * speed_ms - right.y * lateral_speed_ms,
            fwd.z * speed_ms,
        )
        self.challenger.set_target_velocity(vel)

    def update_behavior(self, manager, ego_vehicle):
        if not self.challenger:
            return
        ego_tf = ego_vehicle.get_transform()
        challenger_loc = self.challenger.get_location()
        vec_to_challenger = challenger_loc - ego_tf.location

        ego_right = ego_tf.get_right_vector()
        lateral_dist = (
            vec_to_challenger.x * ego_right.x
            + vec_to_challenger.y * ego_right.y
            + vec_to_challenger.z * ego_right.z
        )

        now = time.time()

        if self.state == "DRIVING_PARALLEL":
            self.set_velocity(self.target_speed)
            if now - self.maneuver_start_time > 1.0:
                print(f"[SCENARIO] Cut-In start! Zmiana pasa.")
                self.state = "CHANGING_LANE"

        elif self.state == "CHANGING_LANE":
            self.set_velocity(self.target_speed - 5.0, lateral_speed_ms=2.5)

            if abs(lateral_dist) < 0.5:
                print(f"[SCENARIO] Zakończono zmianę pasa. Wyrównanie.")
                self.state = "ALIGNING"
                self.align_time = now

        elif self.state == "ALIGNING":
            self.set_velocity(self.target_speed)

            if self.brake_after_cutin and (now - self.align_time > 0.5):
                print(f"[SCENARIO] Challenger HAMUJE!")
                self.state = "BRAKING"

        elif self.state == "BRAKING":
            self.challenger.set_target_velocity(carla.Vector3D(0, 0, 0))
            ctrl = carla.VehicleControl()
            ctrl.brake = 1.0
            ctrl.hand_brake = True
            self.challenger.apply_control(ctrl)


class ObstructedPedestrianTest(BaseScenario):
    """
    Scenariusz CPNC (Car-to-Pedestrian Nearside Child):
    Duży pojazd (ciężarówka/van) stoi na poboczu i zasłania widok.
    Pieszy wybiega na jezdnię tuż przed maską zaparkowanego pojazdu.
    """

    def __init__(self, trigger_distance=15.0, speed=30.0):
        super().__init__(
            f"Obstructed Pedestrian (Trigger {trigger_distance}m)", target_speed=speed
        )

        self.trigger_distance = trigger_distance
        self.parked_vehicle = None
        self.walker = None
        self.walker_controller = None
        self.triggered = False
        self.direction_vector = None
        self.walker_speed = 4.0

    def setup(self, manager):
        self.triggered = False
        self.walker = None
        self.walker_controller = None
        self.parked_vehicle = None

        ego_trans = manager.vehicle.get_transform()
        fwd = ego_trans.get_forward_vector()
        right = ego_trans.get_right_vector()

        obstruction_dist = 25.0
        obstruction_offset = 4.0

        obs_loc = (
            ego_trans.location
            + carla.Location(fwd.x * obstruction_dist, fwd.y * obstruction_dist, 0.3)
            + carla.Location(
                right.x * obstruction_offset, right.y * obstruction_offset, 0
            )
        )

        obs_tf = carla.Transform(obs_loc, ego_trans.rotation)
        self.parked_vehicle = manager.spawn_actor(
            "vehicle.carlamotors.carlacola", obs_tf, role="blocker", isOther=True
        )

        if self.parked_vehicle:
            self.parked_vehicle.set_simulate_physics(True)
            ctrl = carla.VehicleControl()
            ctrl.hand_brake = True
            self.parked_vehicle.apply_control(ctrl)

        walker_long_dist = obstruction_dist + 3.0
        walker_lat_dist = obstruction_offset + 1.5

        self.spawn_loc = (
            ego_trans.location
            + carla.Location(fwd.x * walker_long_dist, fwd.y * walker_long_dist, 0.5)
            + carla.Location(right.x * walker_lat_dist, right.y * walker_lat_dist, 0)
        )

        target_loc = ego_trans.location + carla.Location(
            fwd.x * walker_long_dist, fwd.y * walker_long_dist, 0
        )

        dx = target_loc.x - self.spawn_loc.x
        dy = target_loc.y - self.spawn_loc.y
        length = math.sqrt(dx**2 + dy**2)
        if length > 0:
            self.direction_vector = carla.Vector3D(dx / length, dy / length, 0.0)
        else:
            self.direction_vector = carla.Vector3D(0, 1, 0)

        walker_tf = carla.Transform(self.spawn_loc, carla.Rotation())
        self.walker = manager.spawn_actor(
            "walker.pedestrian.0002", walker_tf, role="hidden_pedestrian"
        )

        manager.obstacle = self.walker

        if self.walker:
            control = carla.WalkerControl()
            control.direction = carla.Vector3D(0, 0, 0)
            control.speed = 0.0
            self.walker.apply_control(control)
            self.walker_controller = None

    def update_behavior(self, manager, ego_vehicle):
        if self.triggered or not self.walker:
            return

        dist = manager.get_distance_to_obstacle()

        if dist and dist < self.trigger_distance:
            print(f"[SCENARIO] Pieszy wybiega zza przeszkody! (Dyst: {dist:.1f}m)")
            if self.walker_controller:
                self.walker_controller.stop()
                self.walker_controller.destroy()
                self.walker_controller = None

            control = carla.WalkerControl()
            control.speed = self.walker_speed
            control.direction = self.direction_vector
            control.jump = False
            self.walker.apply_control(control)
            self.triggered = True


class ObstacleOnCurveTest(BaseScenario):
    """
    Scenariusz: Przeszkoda stoi na zakręcie.
    Wymaga użycia waypointów do ustalenia pozycji wzdłuż geometrii drogi,
    a nie linii prostej.
    """

    def __init__(self, distance=40.0, speed=30.0):
        super().__init__(
            f"Obstacle on Curve (Dist {distance}m, {speed}km/h)", target_speed=speed
        )
        self.distance = distance

    def setup(self, manager):
        manager.vehicle.destroy()
        manager.vehicle = None
        spawn_points = manager.world.get_map().get_spawn_points()
        manager.spawn_vehicle(spawn_points[62])

        ego_trans = manager.vehicle.get_transform()
        map = manager.world.get_map()

        start_wp = map.get_waypoint(ego_trans.location)
        next_wps = start_wp.next(self.distance)

        if not next_wps:
            print("[SCENARIO ERROR] Koniec drogi, nie można ustawić przeszkody!")
            return

        target_wp = next_wps[0]

        spawn_tf = target_wp.transform
        spawn_tf.location.z += 0.5

        manager.obstacle = manager.spawn_actor(
            "vehicle.mini.cooper_s", spawn_tf, role="obstacle_curve"
        )

        if manager.obstacle:
            ctrl = carla.VehicleControl()
            ctrl.hand_brake = True
            manager.obstacle.apply_control(ctrl)

    def update_behavior(self, manager, ego_vehicle):
        control = manager.vehicle.get_control()
        control.steer = self.get_lane_center_steering(manager.vehicle, manager.world)
        manager.vehicle.apply_control(control)

    def get_lane_center_steering(self, vehicle, world):
        transform = vehicle.get_transform()
        loc = transform.location
        rot = transform.rotation
        waypoint = world.get_map().get_waypoint(
            loc, project_to_road=True, lane_type=carla.LaneType.Driving
        )

        if not waypoint:
            return 0.0
        v = vehicle.get_velocity()
        speed = 3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)
        lookahead_dist = np.clip(3.0 + speed * 0.1, 5.0, 15.0)

        next_waypoints = waypoint.next(lookahead_dist)
        if not next_waypoints:
            return 0.0

        target_wp = next_waypoints[0]
        target_loc = target_wp.transform.location
        target_vec_x = target_loc.x - loc.x
        target_vec_y = target_loc.y - loc.y

        target_yaw_rad = math.atan2(target_vec_y, target_vec_x)
        target_yaw = math.degrees(target_yaw_rad)

        current_yaw = rot.yaw
        diff = target_yaw - current_yaw

        while diff > 180:
            diff -= 360
        while diff < -180:
            diff += 360
        steer = diff * 0.04

        return max(min(steer, 1.0), -1.0)
