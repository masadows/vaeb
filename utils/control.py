from vaeb.config.consts import ALPHA, CONF_CRIT_THRESHOLD


def emergency_brake(vehicle, brake_value):
    control = vehicle.get_control()
    control.throttle = 0.0
    brake = min(max(brake_value, 0.0), 1.0)
    control.brake = float(ALPHA * brake + (1 - ALPHA) * control.brake)
    vehicle.apply_control(control)


def brake_handler(vehicle, tm_port, level, confidence, autopilot_active, last_level):
    if level == "critical" and last_level == "critical":
        brake_value = 0.1 + 0.5 * (confidence - CONF_CRIT_THRESHOLD) / (
            100 - CONF_CRIT_THRESHOLD
        )

        if autopilot_active:
            vehicle.set_autopilot(False, tm_port)
            autopilot_active = False

        emergency_brake(vehicle, brake_value=brake_value)
    else:
        if not autopilot_active:
            vehicle.set_autopilot(True, tm_port)
            autopilot_active = True

    return autopilot_active


def speed_brake_handler(
    vehicle,
    level,
    confidence,
    current_speed_kmh,
    last_global_level,
    target_speed_kmh=30.0,
):
    control = vehicle.get_control()
    if level == "critical" and last_global_level == "critical":
        brake_force = 0.2 + 0.4 * (confidence - CONF_CRIT_THRESHOLD) / (
            100 - CONF_CRIT_THRESHOLD
        )
        brake_force = min(max(brake_force, 0.0), 1.0)
        control.brake = float(ALPHA * brake_force + (1 - ALPHA) * control.brake)
        control.throttle = 0.0
    else:
        control.brake = 0.0
        if current_speed_kmh < target_speed_kmh:
            control.throttle = 0.6
        else:
            control.throttle = 0.0

    vehicle.apply_control(control)
    return control
