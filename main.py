import pygame
import cv2
import time
import supervision as sv

from vaeb.config.consts import REAL_HEIGHT, IMAGE_H, IMAGE_W
from vaeb.env.carla import CarlaManager
from vaeb.logger.setup import logger
from vaeb.utils.vision import (
    load_models,
    run_detection,
    filter_lights,
    draw_lights,
    draw_track_info,
    draw_frame_pygame,
)
from vaeb.utils.physics import (
    estimate_speed,
    update_speed_buffer,
    calculate_acceleration,
    estimate_distance_from_bbox,
    cleanup_old_tracks,
    fmt,
)
from vaeb.utils.danger import calculate_confidence_score
from vaeb.utils.control import brake_handler


def main():
    pygame.init()
    clock = pygame.time.Clock()
    screen = pygame.display.set_mode((IMAGE_W, IMAGE_H))
    font = pygame.font.SysFont(None, 24)

    model, light_model, tracker = load_models()

    carla_mgr = CarlaManager()
    try:
        carla_mgr.spawn_vehicle()
        carla_mgr.spawn_camera()

        prev_positions = {}
        speed_buffers = {}
        last_seen = {}
        last_size = {}
        last_levels = {}

        autopilot_active = True
        last_global_level = None
        quit_game = False

        print("Symulacja rozpoczęta. Naciśnij ESC aby wyjść.")

        while not quit_game:
            carla_mgr.world.tick()

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    quit_game = True
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        quit_game = True

            frame = carla_mgr.get_image()
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            car_speed = carla_mgr.get_vehicle_speed()
            fps = clock.get_fps() or 30
            tracker.frame_rate = fps

            # 1. Detekcja świateł
            # lights_res = run_detection(frame, light_model)
            # lights_dets = filter_lights(lights_res)
            # brake_lights_on = len(lights_dets) > 0
            # frame = draw_lights(lights_dets, frame)
            brake_lights_on = False

            # 2. Detekcja obiektów
            results = run_detection(frame, model)
            detections = sv.Detections.from_ultralytics(results)
            tracked_detections = tracker.update_with_detections(detections)

            max_conf = 0
            current_max_level = "safe"

            # 3. Przetwarzanie każdego obiektu
            for i in range(len(tracked_detections.xyxy)):
                xyxy = tracked_detections.xyxy[i]
                class_id = tracked_detections.class_id[i]
                track_id = tracked_detections.tracker_id[i]

                x1, y1, x2, y2 = map(int, xyxy)
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

                # Obliczanie delty ruchu
                delta_x, delta_y, delta_y2 = 0.0, 0.0, 0.0
                if track_id in prev_positions:
                    px, py, py2 = prev_positions[track_id]
                    delta_x, delta_y, delta_y2 = cx - px, cy - py, y2 - py2

                # Estymacja fizyki
                speed, _ = estimate_speed(
                    track_id, cx, cy, fps, prev_positions, car_speed=car_speed
                )
                update_speed_buffer(track_id, speed, speed_buffers)
                accel = calculate_acceleration(track_id, speed_buffers)

                # Estymacja odległości
                dist_m = estimate_distance_from_bbox(y1, y2, REAL_HEIGHT[class_id])

                # Ocena zagrożenia
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

                # Logowanie krytyczne
                if level == "critical" and last_levels.get(track_id) == "critical":
                    ttc_val = (
                        dist_m / speed if (dist_m and speed and speed > 0.1) else None
                    )
                    info = (
                        f"[TRACK {track_id}] CRITICAL | Dist={fmt(dist_m)}m, "
                        f"Speed={fmt(speed)}, Acc={fmt(accel)}, TTC={fmt(ttc_val)}"
                    )
                    logger.critical(info)

                # Śledzenie maksimum ryzyka w tej klatce
                if conf > max_conf:
                    max_conf = conf
                    current_max_level = level

                # Rysowanie
                frame = draw_track_info(frame, x1, y1, x2, y2, speed, accel, level)

                # Aktualizacja stanu obiektu
                last_seen[track_id] = time.time()
                prev_positions[track_id] = (cx, cy, y2)
                last_size[track_id] = current_size
                last_levels[track_id] = level

            # 4. Reakcja Pojazdu (Globalna)
            autopilot_active = brake_handler(
                carla_mgr.vehicle,
                carla_mgr.tm_port,
                current_max_level,
                max_conf,
                autopilot_active,
                last_global_level,
            )

            # Sprzątanie starych śladów
            cleanup_old_tracks(
                prev_positions, speed_buffers, last_seen, last_size, last_levels
            )
            last_global_level = current_max_level

            # 5. Wyświetlanie
            draw_frame_pygame(screen, frame)

            # OSD Statystyki
            txt_spd = font.render(
                f"Speed: {int(car_speed)} km/h", True, (255, 255, 255)
            )
            screen.blit(txt_spd, (10, 10))

            brake_val = carla_mgr.vehicle.get_control().brake
            txt_brk = font.render(
                f"Brake: {brake_val:.2f}",
                True,
                (255, 100, 100) if brake_val > 0 else (100, 255, 100),
            )
            screen.blit(txt_brk, (10, 40))

            pygame.display.flip()
            clock.tick(60)

    except Exception as e:
        print(f"Wystąpił błąd: {e}")
        import traceback

        traceback.print_exc()
    finally:
        print("Zamykanie i sprzątanie...")
        carla_mgr.cleanup()
        pygame.quit()


if __name__ == "__main__":
    main()
