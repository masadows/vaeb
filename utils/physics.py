import math
import numpy as np
from collections import deque
import time
from vaeb.config.consts import BUFFLEN, FOCAL_LENGTH_PX, MIN_DIST, MAX_DIST


def fmt(value, digits=2):
    return f"{value:.{digits}f}" if value is not None else "N/A"


def estimate_speed(track_id, cx, cy, fps, prev_positions, ppm=None, car_speed=None):
    time_const = 1.0 / fps if fps > 0 else 0.1
    speed = None

    if track_id in prev_positions:
        prev_x, prev_y, _ = prev_positions[track_id]
        delta_d = math.hypot(cx - prev_x, cy - prev_y)
        if not ppm and car_speed and car_speed > 0:
            speed_m_per_s = car_speed / 3.6
            ppm = delta_d / (speed_m_per_s * time_const)

        if ppm and ppm > 0:
            speed = (delta_d / ppm) / time_const * 3.6  # km/h

    return speed, ppm


def update_speed_buffer(track_id, speed, speed_buffers, maxlen=BUFFLEN):
    if track_id not in speed_buffers:
        speed_buffers[track_id] = deque(maxlen=maxlen)
    speed_buffers[track_id].append(speed)


def calculate_acceleration(track_id, speed_buffers, beta=0.0625, maxlen=BUFFLEN):
    acceleration = 0.0
    if track_id not in speed_buffers:
        return acceleration

    speeds = [s for s in speed_buffers[track_id] if s is not None]
    currentlen = len(speeds)
    halflen = currentlen // 2

    if currentlen > maxlen // 2 and len(speeds) >= 2:
        initial_avg = sum(speeds[:halflen]) / halflen
        final_avg = sum(speeds[halflen:]) / (maxlen - halflen)
        acceleration = (final_avg - initial_avg) / (currentlen * beta)

    return acceleration


def estimate_distance_from_bbox(y1, y2, real_height_m, pitch_rad=0.0):
    bbox_height_px = y2 - y1
    if bbox_height_px <= 0 or real_height_m is None:
        return None

    est_Z = (FOCAL_LENGTH_PX * real_height_m) / bbox_height_px
    est_Z = est_Z * np.cos(pitch_rad)

    return np.clip(est_Z, MIN_DIST, MAX_DIST)


def cleanup_old_tracks(
    prev_positions, speed_buffers, last_seen, last_size, last_levels, max_age_seconds=10
):
    now = time.time()
    to_delete = [tid for tid, last in last_seen.items() if now - last > max_age_seconds]
    for tid in to_delete:
        prev_positions.pop(tid, None)
        speed_buffers.pop(tid, None)
        last_seen.pop(tid, None)
        last_size.pop(tid, None)
        last_levels.pop(tid, None)
