import time
import numpy as np
import cv2
import supervision as sv
from vaeb.utils.vision import (
    run_detection,
    filter_lights,
    draw_track_info,
    draw_dashboard_lamp,
)
from vaeb.utils.physics import (
    estimate_speed,
    update_speed_buffer,
    calculate_acceleration,
    estimate_distance_from_bbox,
    cleanup_old_tracks,
)
from vaeb.utils.danger import calculate_confidence_score
from vaeb.utils.control import speed_brake_handler
from vaeb.config.consts import REAL_HEIGHT
import functools


class BaseScenario:
    def __init__(self, name, target_speed=30.0, max_duration=20.0, should_stop=True):
        self.name = name
        self.max_duration = max_duration
        self.target_speed = target_speed
        self.should_stop = should_stop
        self.actors = []

    def reset_metrics(self):
        self.metrics = {
            "name": self.name,
            "target_speed": self.target_speed,
            "collision": False,
            "min_distance": float("inf"),
            "stopped": False,
            "brake_reaction_time": None,
            "result": "N/A",
        }

    def setup(self, manager):
        """Tutaj konkretne testy będą ustawiać scenę."""
        raise NotImplementedError

    def update_behavior(self, manager, ego_vehicle):
        """Tutaj testy mogą modyfikować zachowanie w trakcie (np. pieszy rusza)."""
        pass

    def record(func):
        @functools.wraps(func)
        def wrapper(self, manager, *args, **kwargs):
            safe_name = self.name.replace(" ", "_").replace("/", "-").replace(",", "")
            video_filename = f"{safe_name}.mp4"
            manager.start_recording(video_filename)
            value = func(self, manager, *args, **kwargs)
            manager.stop_recording()
            return value

        return wrapper

    @record
    def run(self, manager, models):
        print(f"\n=== ROZPOCZYNAM TEST: {self.name} ===")
        model, light_model, tracker = models
        self.reset_metrics()

        manager.reset()
        spawn_points = manager.world.get_map().get_spawn_points()
        manager.spawn_vehicle(spawn_points[0])

        self.setup(manager)

        manager.spawn_camera()
        manager.attach_collision_sensor()
        manager.set_vehicle_speed(self.target_speed)

        manager.tm.ignore_vehicles_percentage(manager.vehicle, 100.0)
        manager.tm.vehicle_percentage_speed_difference(
            manager.vehicle, -30
        )  # +30% prędkości

        # time.sleep(0.5)
        start_time = time.time()

        prev_positions = {}
        speed_buffers = {}
        last_size = {}
        last_seen = {}
        last_levels = {}
        last_global_level = None
        start_braking_time = None

        collision_time = 0
        stop_time = 0

        while (time.time() - start_time) < self.max_duration:
            manager.world.tick()
            self.update_behavior(manager, manager.vehicle)

            if len(manager.collision_history) > 0 and not self.metrics["collision"]:
                self.metrics["collision"] = True
                print("!!! KOLIZJA !!!")
                collision_time = time.time()
            elif (
                len(manager.collision_history) > 0 and time.time() - collision_time > 2
            ):
                break

            dist = manager.get_distance_to_obstacle()
            if dist:
                self.metrics["min_distance"] = min(self.metrics["min_distance"], dist)

            v_ego = manager.get_vehicle_speed()
            if v_ego < 0.5 and (time.time() - start_time) > 1.0:
                self.metrics["stopped"] = True
                stop_time = time.time()
            if self.metrics["stopped"] and time.time() - stop_time > 1.0:
                break

            frame = manager.get_image()
            if frame.dtype != np.uint8:
                frame = frame.astype(np.uint8)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            lights_res = run_detection(frame, light_model)
            lights_dets = filter_lights(lights_res)
            brake_lights_on = len(lights_dets) > 0

            results = run_detection(frame, model)
            detections = sv.Detections.from_ultralytics(results)
            tracked_detections = tracker.update_with_detections(detections)

            current_max_level = "safe"
            max_conf = 0

            fps = 30
            car_speed = manager.get_vehicle_speed()

            for i in range(len(tracked_detections.xyxy)):
                xyxy = tracked_detections.xyxy[i]
                class_id = tracked_detections.class_id[i]
                track_id = tracked_detections.tracker_id[i]

                x1, y1, x2, y2 = map(int, xyxy)
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

                delta_x, delta_y, delta_y2 = 0.0, 0.0, 0.0
                if track_id in prev_positions:
                    px, py, py2 = prev_positions[track_id]
                    delta_x = cx - px
                    delta_y = cy - py
                    delta_y2 = y2 - py2

                speed, _ = estimate_speed(
                    track_id, cx, cy, fps, prev_positions, car_speed=car_speed
                )
                update_speed_buffer(track_id, speed, speed_buffers)
                accel = calculate_acceleration(track_id, speed_buffers)

                real_h = REAL_HEIGHT[class_id] if class_id < len(REAL_HEIGHT) else 1.5
                dist_m = estimate_distance_from_bbox(y1, y2, real_h)

                current_size = abs(x1 - x2) * abs(y1 - y2)
                prev_sz = last_size.get(track_id, None)

                conf, level, scores = calculate_confidence_score(
                    accel,
                    speed,
                    dist_m,
                    brake_lights_on,
                    cx,
                    cy,
                    y2,
                    current_size,
                    prev_sz,
                    delta_x,
                    delta_y,
                    delta_y2,
                )

                if conf > max_conf:
                    max_conf = conf
                    current_max_level = level

                last_seen[track_id] = time.time()
                prev_positions[track_id] = (cx, cy, y2)
                last_size[track_id] = current_size
                last_levels[track_id] = level

                frame = draw_dashboard_lamp(frame, current_max_level)
                # frame = draw_track_info(frame, x1, y1, x2, y2, speed, accel, level)
            manager.write_frame(frame)

            cleanup_old_tracks(
                prev_positions, speed_buffers, last_seen, last_size, last_levels
            )

            if (
                current_max_level in ["warning", "critical"]
                and start_braking_time is None
            ):
                start_braking_time = time.time() - start_time
                self.metrics["brake_reaction_time"] = start_braking_time

            current_speed = manager.get_vehicle_speed()
            speed_brake_handler(
                manager.vehicle,
                level=current_max_level,
                confidence=max_conf,
                current_speed_kmh=current_speed,
                last_global_level=last_global_level,
                target_speed_kmh=self.target_speed,
            )
            last_global_level = current_max_level

        control = manager.vehicle.get_control()
        control.throttle = 0.0
        control.brake = 1.0
        manager.vehicle.apply_control(control)

        if self.metrics["collision"]:
            self.metrics["result"] = "FAIL (Collision)"
        elif self.metrics["stopped"] and not self.should_stop:
            self.metrics["result"] = "FAIL (Stopped)"
        elif self.metrics["stopped"]:
            self.metrics["result"] = "PASS (Stopped)"
        else:
            self.metrics["result"] = "PASS (Timeout)"

        print(f"Koniec testu: {self.metrics['result']}")
        return self.metrics
