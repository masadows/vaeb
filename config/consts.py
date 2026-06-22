import math
import torch

# Ścieżki i Urządzenie
YOLO_DETEC_WEIGHTS = "../runs/detect/yolo11l-bdd100k/weights/best.pt"
YOLO_LIGHT_WEIGHTS = "../runs/detect/yolov11m-light/weights/best.pt"
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"

# CARLA
CARLA_HOST = "localhost"
CARLA_PORT = 2000
TM_PORT_OFFSET = 8000
SPAWN_POINT_INDEX = 0
CAMERA_POS_Z = 1.5
CAMERA_POS_X = 1.0
IMAGE_W = 640
IMAGE_H = 640
FOCAL_LENGTH_PX = (IMAGE_W / 2) / math.tan(math.radians(90) / 2)

# Detekcja i Tracking
CONF_THRES = 0.3
CLASSES = None
BYTETRACKER_KWARGS = dict()
# ['bike', 'bus', 'car', 'motor', 'person', 'rider', 'traffic light', 'traffic sign', 'train', 'truck']
REAL_HEIGHT = [1.1, 3.5, 1.5, 1.15, 1.7, 1.83, 0.9, 0.6, 3.5, 3.5]

# Fizyka i Logika Zagrożenia
BUFFLEN = 10
TTC_WARNING_THRESHOLD = 3.0
TTC_BRAKE_THRESHOLD = 1.0
CONF_WARN_THRESHOLD = 50.0
CONF_CRIT_THRESHOLD = 60.0
A_REF = 10.0
MIN_APPROACH_SPEED = 0.3
MIN_DIST = 1.0
MAX_DIST = 200.0

# Wagi oceny ryzyka
CONF_WEIGHTS = {"ttc": 0.10, "accel": 0.3, "brake": 0.0, "traj": 0.6, "close": 1.0}


MAX_WIDTH_RATIO = 0.57
MIN_WIDTH_RATIO = 0.02
TRAPEZOID_HEIGHT_RATIO = 0.43
CLOSE_HEIGHT = 0.35
CLOSE_WIDT_MODIFIER = 0.7
EPS = IMAGE_H * 0.001
W_GROWTH = 0.4
W_DIST = 0.42
W_ALIGN = 0.65
COS_ANGLE = 0.5

# Do wygładzania hamowania
ALPHA = 0.65
