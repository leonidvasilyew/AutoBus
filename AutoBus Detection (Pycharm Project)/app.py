import math
import os
import pickle

import cv2
import numpy as np

from bus_detector import (compute_bus_ranges, sample_around_point, build_bus_masks)
from bus_detector import detect_buses, draw_buses, BusTracker
from route_planner import plan_route

LOOKAHEAD_PX = 100
MAPS_DIR = "maps"
BUS_CALIB_PATH = "bus_calib.pkl"


def save_bus_calibration(body_samples, marker_samples, ranges):
    with open(BUS_CALIB_PATH, 'wb') as f:
        pickle.dump({
            'body_samples': body_samples,
            'marker_samples': marker_samples,
            'ranges': ranges,
        }, f)
def load_bus_calibration():
    if not os.path.exists(BUS_CALIB_PATH):
        return None
    with open(BUS_CALIB_PATH, 'rb') as f:
        return pickle.load(f)
def reset_bus_calibration():
    if os.path.exists(BUS_CALIB_PATH):
        os.remove(BUS_CALIB_PATH)
def load_map(path):
    with open(path, 'rb') as f:
        return pickle.load(f)
def list_maps():
    if not os.path.isdir(MAPS_DIR):
        return []
    return sorted(f for f in os.listdir(MAPS_DIR) if f.endswith(".pkl"))
def pick_map():
    files = list_maps()
    if not files:
        print("No maps yet")
        return None
    for i, f in enumerate(files):
        print(f"  [{i}] {f}")
    idx = int(input("Pick map index: ").strip())
    return load_map(os.path.join(MAPS_DIR, files[idx]))


def draw_map_overlay(vis, map_data, route):
    ARROW_TIP = 12

    for lane in map_data['lanes'].values():
        cl = lane['centerline']
        pts = np.array(cl, dtype=np.int32).reshape(-1, 1, 2)
        cv2.polylines(vis, [pts], False, (150, 150, 150), 1, cv2.LINE_AA)

        if len(cl) >= 2:
            mid = len(cl) // 2
            k = min(3, len(cl) - 1 - mid, mid)
            if k > 0:
                p_mid = np.array(cl[mid], dtype=np.float64)
                tan = np.array(cl[mid + k]) - np.array(cl[mid - k])
                tan_len = np.linalg.norm(tan)
                if tan_len > 1e-6:
                    tan /= tan_len
                    p1 = p_mid - tan * (ARROW_TIP / 2)
                    p2 = p_mid + tan * (ARROW_TIP / 2)
                    cv2.arrowedLine(vis,
                                    tuple(map(int, p1)),
                                    tuple(map(int, p2)),
                                    (150, 150, 150), 1, tipLength=1.0,
                                    line_type=cv2.LINE_AA)

    for curve in map_data['junction_paths'].values():
        pts = np.array(curve, dtype=np.int32).reshape(-1, 1, 2)
        cv2.polylines(vis, [pts], False, (150, 150, 150), 1, cv2.LINE_AA)

    if route is not None:
        pts = np.array(route['polyline'], dtype=np.int32).reshape(-1, 1, 2)
        color = (0, 200, 255) if route.get('direct') else (0, 255, 0)
        cv2.polylines(vis, [pts], False, color, 3, cv2.LINE_AA)


def find_lookahead_point(polyline, from_point, lookahead_dist):
    if len(polyline) < 2:
        return polyline[0][0], polyline[0][1], 0

    pts = np.array(polyline, dtype=np.float64)
    p = np.array(from_point, dtype=np.float64)

    best = (float('inf'), 0, 0.0, pts[0])
    for i in range(len(pts) - 1):
        a, b = pts[i], pts[i + 1]
        ab = b - a
        ab2 = float(np.dot(ab, ab))
        if ab2 < 1e-12:
            continue
        u = float(np.dot(p - a, ab) / ab2)
        u = max(0.0, min(1.0, u))
        proj = a + u * ab
        d = float(np.linalg.norm(p - proj))
        if d < best[0]:
            best = (d, i, u, proj)

    _, seg_idx, u, proj = best

    remaining = lookahead_dist
    cur_pos = proj
    i = seg_idx

    seg_end = pts[i + 1]
    seg_left = float(np.linalg.norm(seg_end - cur_pos))
    if seg_left >= remaining:
        direction = (seg_end - cur_pos) / (seg_left + 1e-12)
        target = cur_pos + direction * remaining
        return float(target[0]), float(target[1]), i + 1
    remaining -= seg_left
    cur_pos = seg_end
    i += 1

    while i < len(pts) - 1:
        a, b = pts[i], pts[i + 1]
        seg_len = float(np.linalg.norm(b - a))
        if seg_len >= remaining:
            direction = (b - a) / (seg_len + 1e-12)
            target = a + direction * remaining
            return float(target[0]), float(target[1]), i + 1
        remaining -= seg_len
        i += 1

    last = pts[-1]
    return float(last[0]), float(last[1]), len(pts) - 1


def angle_error_deg(bus_angle_deg, from_pt, to_pt):
    dx = to_pt[0] - from_pt[0]
    dy = to_pt[1] - from_pt[1]
    target_deg = math.degrees(math.atan2(-dy, dx))
    diff = ((target_deg - bus_angle_deg + 180) % 360) - 180
    return diff


def sample_marker_around_point(image, point, radius=6, top_fraction=0.4):
    h, w = image.shape[:2]
    x, y = point
    x0, x1 = max(0, x - radius), min(w, x + radius + 1)
    y0, y1 = max(0, y - radius), min(h, y + radius + 1)
    patch = image[y0:y1, x0:x1].reshape(-1, 3)
    if len(patch) == 0:
        return patch

    hsv = cv2.cvtColor(patch.reshape(-1, 1, 3), cv2.COLOR_BGR2HSV).reshape(-1, 3)

    score = hsv[:, 2].astype(np.float32) - hsv[:, 1].astype(np.float32)

    n_keep = max(1, int(len(score) * top_fraction))
    idx = np.argsort(-score)[:n_keep]
    return patch[idx]


def drive_flow():
    map_data = pick_map()
    if map_data is None:
        return

    src = input("Camera index or video path: ").strip() or "0"
    cap = cv2.VideoCapture(int(src) if src.isdigit() else src)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    cap.set(cv2.CAP_PROP_FPS, 30)

    tracker = BusTracker()
    calib_data = load_bus_calibration()
    ranges = calib_data['ranges'] if calib_data else None

    nav = {
        'route': None,
        'target': None,
    }

    calib_state = {
        'active': False,
        'frozen_frame': None,
        'body_samples': (calib_data['body_samples']
                         if calib_data else np.zeros((0, 3), dtype=np.uint8)),
        'marker_samples': (calib_data['marker_samples']
                           if calib_data else np.zeros((0, 3), dtype=np.uint8)),
    }

    def replan(start_point, heading):
        if nav['target'] is None or start_point is None:
            nav['route'] = None
            return
        nav['route'] = plan_route(
            start_point, nav['target'],
            map_data['lanes'], map_data['connections'],
            map_data['junction_paths'],
            start_heading_deg=heading,
        )

    fh, fw = map_data['frame_shape'][:2]
    DISPLAY_SCALE = 1

    cv2.namedWindow('Drive', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Drive', int(fw * DISPLAY_SCALE), int(fh * DISPLAY_SCALE))

    def on_mouse(event, x, y, *_):
        if calib_state['active']:
            frame = calib_state['frozen_frame']
            if event == cv2.EVENT_LBUTTONDOWN:
                calib_state['body_samples'] = np.concatenate(
                    [calib_state['body_samples'], sample_around_point(frame, (x, y))],
                    axis=0)
            elif event == cv2.EVENT_RBUTTONDOWN:
                (
                    calib_state)['marker_samples'] = np.concatenate([calib_state['marker_samples'],
                                                                     sample_marker_around_point(frame, (x, y))], axis=0)
            return

        if event == cv2.EVENT_LBUTTONDOWN:
            nav['target'] = (x, y)
            start = tracker.front if tracker.front is not None else tracker.center
            replan(start, tracker.angle)
        elif event == cv2.EVENT_RBUTTONDOWN:
            nav['target'] = None
            nav['route'] = None

    cv2.setMouseCallback('Drive', on_mouse)

    target_shape = map_data['frame_shape']

    while True:
        if not calib_state['active']:
            ret, frame = cap.read()
            if not ret:
                break
            if frame.shape != target_shape:
                frame = cv2.resize(frame, (target_shape[1], target_shape[0]))
        else:
            frame = calib_state['frozen_frame']

        buses = []
        if ranges is not None and not calib_state['active']:
            buses, _, _ = detect_buses(frame, ranges)
        bus_center, bus_angle, bus_front = tracker.update(buses)

        if nav['target'] is not None and bus_front is not None:
            d = np.linalg.norm(np.array(bus_front) - np.array(nav['target']))
            if d < 25:  # порог в пикселях
                nav['target'] = None
                nav['route'] = None

        lookahead = None
        if nav['route'] is not None and bus_front is not None:
            lx, ly, _ = find_lookahead_point(
                nav['route']['polyline'], bus_front, LOOKAHEAD_PX
            )
            lookahead = (int(lx), int(ly))

        has_route = nav[
                        'route'] is not None and lookahead is not None and bus_front is not None and bus_angle is not None

        if has_route:
            err_deg = angle_error_deg(bus_angle, bus_front, lookahead)
            angle = drive_command(True, err_deg)
        else:
            angle = drive_command(False, 0.0)

        vis = frame.copy()
        draw_map_overlay(vis, map_data, nav['route'])

        if lookahead is not None and bus_front is not None:
            bf = (int(bus_front[0]), int(bus_front[1]))
            cv2.line(vis, bf, lookahead, (0, 255, 255), 2, cv2.LINE_AA)
            cv2.circle(vis, lookahead, 7, (0, 255, 255), -1)
            cv2.circle(vis, lookahead, 7, (0, 0, 0), 1)

        if buses:
            vis = draw_buses(vis, buses)

        if nav['target'] is not None:
            cv2.circle(vis, nav['target'], 8, (0, 0, 255), -1)
            cv2.circle(vis, nav['target'], 8, (255, 255, 255), 2)

        if calib_state['active']:
            nb = len(calib_state['body_samples'])
            nm = len(calib_state['marker_samples'])

            if nb >= 5 and nm >= 5:
                preview_ranges = compute_bus_ranges(
                    calib_state['body_samples'],
                    calib_state['marker_samples'],
                )
                body_m, marker_m = build_bus_masks(frame, preview_ranges)
                overlay = vis.copy()
                overlay[body_m > 0] = (255, 100, 0)
                overlay[marker_m > 0] = (255, 255, 255)
                vis = cv2.addWeighted(vis, 0.55, overlay, 0.45, 0)

            cv2.putText(vis, f"BUS CALIBRATION  body:{nb} marker:{nm}",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 255, 255), 2)
        if has_route:
            cv2.putText(vis, f"err={err_deg:+.1f} deg ({angle:+.1f} to servo)", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        cv2.imshow('Drive', vis)
        if ranges is not None:
            body_m, marker_m = build_bus_masks(frame, ranges)
            cv2.imshow('Body mask', body_m)
            marker_inside = cv2.bitwise_and(marker_m, body_m)
            cv2.imshow('Marker inside body', marker_inside)
        key = cv2.waitKeyEx(1)
        if key == -1:
            pass
        elif key == 27:  # Esc
            if calib_state['active']:
                calib_state['active'] = False
            else:
                break
        elif key == 0x2E0000 or key == 65535 or key == 65439:
            reset_bus_calibration()
            ranges = None
            calib_state['body_samples'] = np.zeros((0, 3), dtype=np.uint8)
            calib_state['marker_samples'] = np.zeros((0, 3), dtype=np.uint8)
            print("Bus calibration reset")
        elif (key & 0xFF) == ord('b') and not calib_state['active']:
            calib_state['active'] = True
            calib_state['frozen_frame'] = frame.copy()
        elif (key & 0xFF) == ord('r') and not calib_state['active']:
            start = tracker.front if tracker.front is not None else tracker.center
            replan(start, tracker.angle)
        elif key in (13, 10) and calib_state['active']:
            if (len(calib_state['body_samples']) >= 5 and
                    len(calib_state['marker_samples']) >= 5):
                ranges = compute_bus_ranges(
                    calib_state['body_samples'],
                    calib_state['marker_samples'],
                )
                save_bus_calibration(
                    calib_state['body_samples'],
                    calib_state['marker_samples'],
                    ranges,
                )
                calib_state['active'] = False
        if key != -1:
            print(f"key code: {key}")

    cap.release()
    cv2.destroyAllWindows()


import serial
from bisect import bisect_left

speed = 110
ser = serial.Serial('COM5', 115200, timeout=1)
_X = (0, 8.07, 15.04, 21.79, 28.18, 34.01, 39.05, 43.01, 45.57, 46.45)
_Y = (0, 10, 20, 30, 40, 50, 60, 70, 80, 90)


def alpha_to_beta(a):
    if a <= _X[0]:  return _Y[0]
    if a >= _X[-1]: return _Y[-1]
    i = bisect_left(_X, a)
    x0, x1 = _X[i - 1], _X[i]
    y0, y1 = _Y[i - 1], _Y[i]
    return y0 + (y1 - y0) * (a - x0) / (x1 - x0)


def drive_command(rote: bool, angle: float) -> None:
    d = -1
    if angle < 0:
        angle = -angle
        d = 1
    servo = 90 + alpha_to_beta(angle) * d
    if rote:
        ser.write(f"{speed},{int(servo)}\n".encode())
    else:
        ser.write(f"{0},{int(servo)}\n".encode())
    return servo


if __name__ == '__main__':
    drive_flow()
