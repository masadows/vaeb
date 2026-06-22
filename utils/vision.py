import cv2
import supervision as sv
from ultralytics import YOLO
from vaeb.config.consts import *
import pygame


def load_models():
    model = YOLO(YOLO_DETEC_WEIGHTS)
    light_model = YOLO(YOLO_LIGHT_WEIGHTS)
    tracker = sv.ByteTrack(**BYTETRACKER_KWARGS)
    return model, light_model, tracker


def run_detection(frame, model, conf=CONF_THRES, classes=CLASSES):
    return model.predict(
        frame, conf=conf, classes=classes, device=DEVICE, verbose=False
    )[0]


def filter_lights(results):
    detections = []
    for box in results.boxes:
        cls_id = int(box.cls[0])
        if cls_id:
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            conf = float(box.conf[0])
            detections.append(([x1, y1, x2 - x1, y2 - y1], conf, cls_id))
    return detections


def draw_lights(detections, frame):
    color = (250, 0, 0)
    for (x, y, w, h), *_ in detections:
        cv2.rectangle(frame, (x, y), (x + w, y + h), color, 4)
    return frame


def draw_track_info(frame, x1, y1, x2, y2, speed, acceleration, level):
    if level == "critical":
        color = (255, 0, 0)  # Red
    elif level == "warning":
        color = (0, 255, 255)  # Yellow
    else:
        color = (0, 255, 0)  # Green

    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

    text = ""
    if speed is not None:
        text += f"{speed:.1f} km/h"
    if acceleration is not None:
        text += f" | a:{acceleration:.2f}"

    if text:
        text_scale = 0.5
        text_thickness = 1
        (tw, th), _ = cv2.getTextSize(
            text, cv2.FONT_HERSHEY_SIMPLEX, text_scale, text_thickness
        )

        cv2.rectangle(frame, (x1, y1 - th - 5), (x1 + tw, y1), (255, 255, 255), -1)
        cv2.putText(
            frame,
            text,
            (x1, y1 - 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            text_scale,
            (0, 0, 0),
            text_thickness,
        )

    return frame


def draw_dashboard_lamp(frame, level):
    height, width = frame.shape[:2]
    center = (width - 50, 50)
    radius = 30
    color = (50, 50, 50)

    if level == "critical":
        color = (255, 0, 0)
    elif level == "warning":
        color = (255, 255, 0)
    elif level == "safe":
        color = (0, 255, 0)

    cv2.circle(frame, center, radius, color, -1)
    cv2.circle(frame, center, radius, (200, 200, 200), 2)
    return frame


def draw_frame_pygame(screen, frame):
    frame_surface = pygame.surfarray.make_surface(frame.swapaxes(0, 1))
    screen.blit(frame_surface, (0, 0))
    pygame.display.flip()
