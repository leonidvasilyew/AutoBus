import cv2
import numpy as np
import math


def fill_holes(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
    filled = np.zeros_like(mask)
    cv2.drawContours(filled, contours, -1, 255, thickness=cv2.FILLED)
    return filled
def build_bus_masks(image, ranges):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    body = cv2.inRange(hsv, ranges['body_low'], ranges['body_high'])
    marker = cv2.inRange(hsv, ranges['marker_low'], ranges['marker_high'])

    kern = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

    body = cv2.morphologyEx(body, cv2.MORPH_CLOSE, kern)
    body = cv2.morphologyEx(body, cv2.MORPH_OPEN, kern)
    marker = cv2.morphologyEx(marker, cv2.MORPH_OPEN, kern)

    marker = cv2.bitwise_and(marker, cv2.bitwise_not(body))

    body = fill_holes(body)

    return body, marker

def find_white_marker(white_mask, bus_contour, shrink_px=10):
    """
    1. Заливает контур автобуса в маску.
    2. Эрозирует её на shrink_px пикселей внутрь.
    3. В получившейся "сердцевине" ищет белые пиксели (метку).
    4. Возвращает центр масс белой области.
    """
    h_img, w_img = white_mask.shape
    bus_zone = np.zeros((h_img, w_img), dtype=np.uint8)
    cv2.fillPoly(bus_zone, [bus_contour], 255)

    if shrink_px > 0:
        ksize = 2 * shrink_px + 1
        kern = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ksize, ksize))
        bus_zone = cv2.erode(bus_zone, kern, iterations=1)

    local = cv2.bitwise_and(white_mask, bus_zone)
    if cv2.countNonZero(local) < 5:
        return None

    ys, xs = np.where(local > 0)
    weights = local[ys, xs].astype(np.float32)
    total = float(weights.sum())
    fx = float(np.sum(xs * weights) / total)
    fy = float(np.sum(ys * weights) / total)
    return (int(fx), int(fy))

def detect_buses(image, ranges, min_area=300, max_area=100000):
    body_mask, marker_mask = build_bus_masks(image, ranges)

    contours, _ = cv2.findContours(body_mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    buses = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area or area > max_area:
            continue
        rect = cv2.minAreaRect(cnt)
        (cx, cy), (w, h), _ = rect
        rect_area = max(w * h, 1)
        if area / rect_area < 0.8:
            continue
        short, long_side = min(w, h), max(w, h)
        if short == 0 or long_side / short < 1.3:
            continue

        box = cv2.boxPoints(rect).astype(np.int32)
        front = find_white_marker(marker_mask, cnt, shrink_px=10)
        if front is None:
            continue
        angle = math.degrees(math.atan2(-(front[1] - cy), front[0] - cx))

        buses.append({
            'center': (int(cx), int(cy)),
            'angle':  angle,
            'rect':   rect,
            'front':  front,
            'box':    box,
        })
    return buses, body_mask, marker_mask


def normalize_rect_angle(rect_angle, w, h):
    if w < h:
        angle = rect_angle + 90
    else:
        angle = rect_angle
    return angle


def draw_buses(image, buses):
    vis = image.copy()

    for i, bus in enumerate(buses):
        cx, cy = bus['center']
        angle = bus['angle']
        front = bus['front']
        box = bus['box']

        cv2.drawContours(vis, [box], 0, (0, 255, 0), 2)

        cv2.circle(vis, (cx, cy), 5, (0, 255, 0), -1)

        arrow_len = 40
        ax = int(cx + arrow_len * math.cos(math.radians(angle)))
        ay = int(cy - arrow_len * math.sin(math.radians(angle)))
        cv2.arrowedLine(vis, (cx, cy), (ax, ay), (0, 255, 0), 3, tipLength=0.4)

        if front:
            cv2.circle(vis, front, 6, (0, 0, 255), -1)

        label = f"Bus {i}: ({cx},{cy}) {angle:.0f} deg"
        cv2.putText(vis, label, (cx - 60, cy - 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    return vis

class BusTracker:
    def __init__(self, alpha_pos=0.4, alpha_angle=0.3):
        self.center = None
        self.angle  = None
        self.front  = None
        self.alpha_pos = alpha_pos
        self.alpha_ang = alpha_angle

    def update(self, detections):
        if not detections:
            return self.center, self.angle, self.front

        if self.center is not None:
            cx, cy = self.center
            best = min(detections, key=lambda b:
                       (b['center'][0]-cx)**2 + (b['center'][1]-cy)**2)
        else:
            best = max(detections, key=lambda b: cv2.contourArea(b['box']))

        c = np.array(best['center'], dtype=np.float32)
        if self.center is None:
            self.center = tuple(c)
        else:
            self.center = tuple((1 - self.alpha_pos) * np.array(self.center) +
                                self.alpha_pos * c) # Экспоненциальное сглаживание

        if best['front'] is not None:
            self.front = tuple(best['front'])

        if best['angle'] is not None:
            new_a = best['angle']
            if self.angle is None:
                self.angle = new_a
            else:
                diff = ((new_a - self.angle + 180) % 360) - 180
                self.angle = self.angle + self.alpha_ang * diff

        return self.center, self.angle, self.front

def compute_bus_ranges(body_bgr, marker_bgr, pad=(10, 40, 40)):
    """
    Возвращает dict с порогами для inRange в HSV.
    pad — допуск (H, S, V) вокруг минимумов/максимумов выборки.
    """
    def hsv_range(samples):
        bgr = samples.reshape(-1, 1, 3).astype(np.uint8)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV).reshape(-1, 3).astype(np.int32)
        h, s, v = hsv[:, 0], hsv[:, 1], hsv[:, 2]
        # Используем процентили вместо min/max — устойчивее к выбросам
        h_lo, h_hi = np.percentile(h, 5),  np.percentile(h, 95)
        s_lo, s_hi = np.percentile(s, 5),  np.percentile(s, 95)
        v_lo, v_hi = np.percentile(v, 5),  np.percentile(v, 95)
        return (
            np.array([max(0,   h_lo - pad[0]),
                      max(0,   s_lo - pad[1]),
                      max(0,   v_lo - pad[2])], dtype=np.uint8),
            np.array([min(180, h_hi + pad[0]),
                      min(255, s_hi + pad[1]),
                      min(255, v_hi + pad[2])], dtype=np.uint8),
        )

    body_low, body_high = hsv_range(body_bgr)
    marker_low, marker_high = hsv_range(marker_bgr)
    return {
        'body_low':   body_low,
        'body_high':  body_high,
        'marker_low':  marker_low,
        'marker_high': marker_high,
    }



def sample_around_point(image, point, radius=4):
    h, w = image.shape[:2]
    x, y = point
    x0, x1 = max(0, x - radius), min(w, x + radius + 1)
    y0, y1 = max(0, y - radius), min(h, y + radius + 1)
    patch = image[y0:y1, x0:x1].reshape(-1, 3)
    return patch
