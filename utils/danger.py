import numpy as np
from vaeb.config.consts import *


def calculate_confidence_score(
    accel_m_s2,
    approach_speed_mps,
    distance_m,
    brake_on,
    cx,
    cy,
    y2,
    current_size,
    last_size,
    delta_x,
    delta_y,
    delta_y2,
):
    # 1. TTC (Time To Collision)
    ttc_score = 0.0
    if (
        distance_m is not None
        and approach_speed_mps is not None
        and approach_speed_mps > MIN_APPROACH_SPEED
    ):
        if last_size is not None and (last_size < current_size or delta_y > 0):
            ttc = distance_m / approach_speed_mps / 3.6
            if ttc <= TTC_BRAKE_THRESHOLD:
                ttc_score = 1.0
            elif ttc < TTC_WARNING_THRESHOLD:
                ttc_score = np.clip(
                    (TTC_WARNING_THRESHOLD - ttc)
                    / (TTC_WARNING_THRESHOLD - TTC_BRAKE_THRESHOLD),
                    0.0,
                    1.0,
                )

    # 2. Acceleration
    accel_score = 0.0
    if (
        accel_m_s2 is not None
        and approach_speed_mps is not None
        and approach_speed_mps > MIN_APPROACH_SPEED
    ):
        accel_score = 1.0 - np.exp(-abs(accel_m_s2) / A_REF)

    # 3. Brake Light
    brake_score = 1.0 if brake_on else 0.0

    # 4. Trajectory / Lane Alignment
    growth_score = 0.0
    traj_score = 0.0
    align_score = 0.0
    dist_score = 0.0
    close_score = 0.0

    frame_area = IMAGE_W * IMAGE_H

    if last_size is not None and last_size > 0:
        growth = np.log1p(max(current_size - last_size, 0)) / np.log1p(last_size)
        growth_score = np.clip(growth, 0.0, 1.0)

    target_x = IMAGE_W / 2.0
    target_y = IMAGE_H

    # Wektory
    vec_motion = np.array([delta_x, delta_y2])
    vec_target = np.array([target_x - cx, target_y - y2])

    norm_motion = np.linalg.norm(vec_motion)
    norm_target = np.linalg.norm(vec_target)

    cos_angle = 0.0
    if norm_motion > 0 and norm_target > 0:
        cos_angle = np.dot(vec_motion, vec_target) / (norm_motion * norm_target)

    # Trapez kolizji
    vertical_dist = IMAGE_H - y2
    top_y = int(IMAGE_H * TRAPEZOID_HEIGHT_RATIO)

    # Sprawdzenie czy obiekt jest w pasie ruchu
    width_ratio = MIN_WIDTH_RATIO + (MAX_WIDTH_RATIO - MIN_WIDTH_RATIO) * (
        (cy - top_y) / (IMAGE_H - top_y) if (IMAGE_H - top_y) != 0 else 0
    )
    lane_half_width = IMAGE_W * width_ratio
    within_lane = (
        abs(cx - target_x) <= lane_half_width and cy > top_y and vertical_dist > EPS
    )

    moving_towards = delta_y > EPS and cos_angle > COS_ANGLE

    align_score = np.clip((cos_angle - COS_ANGLE) / (1 - COS_ANGLE), 0.0, 1.0)
    dist_modifier = CLOSE_HEIGHT * IMAGE_H * 0.75
    dist_score = 1.0 - np.clip((vertical_dist - dist_modifier) / top_y, 0.0, 1.0)

    if within_lane or moving_towards:
        traj_score = np.clip(
            W_GROWTH * growth_score + W_DIST * dist_score + W_ALIGN * align_score,
            0.0,
            1.0,
        )

    # Close proximity check
    close_width_ratio = MIN_WIDTH_RATIO + (MAX_WIDTH_RATIO - MIN_WIDTH_RATIO) * (
        (y2 - top_y) / (IMAGE_H - top_y) if (IMAGE_H - top_y) != 0 else 0
    )
    close_half_width = IMAGE_W * close_width_ratio * CLOSE_WIDT_MODIFIER
    close_within_lane = abs(cx - target_x) <= close_half_width and y2 > top_y

    if current_size >= 0.3 * frame_area:
        close_score = 0.25
    if (
        vertical_dist < IMAGE_H * CLOSE_HEIGHT
        and close_within_lane
        and vertical_dist > EPS * 10
    ):
        close_score = 1.0

    # 5. Final Score
    final_score = (
        CONF_WEIGHTS["ttc"] * ttc_score
        + CONF_WEIGHTS["accel"] * accel_score
        + CONF_WEIGHTS["brake"] * brake_score
        + CONF_WEIGHTS["traj"] * traj_score
        + CONF_WEIGHTS["close"] * close_score
    )

    confidence = np.clip(final_score * 100.0, 0.0, 100.0)

    if confidence >= CONF_CRIT_THRESHOLD:
        level = "critical"
    elif confidence >= CONF_WARN_THRESHOLD:
        level = "warning"
    else:
        level = "safe"

    scores = {
        "ttc": ttc_score,
        "accel": accel_score,
        "brake": brake_score,
        "traj": traj_score,
        "close": close_score,
    }

    return confidence, level, scores


def fmt(value, digits=2):
    return f"{value:.{digits}f}" if value is not None else "N/A"
