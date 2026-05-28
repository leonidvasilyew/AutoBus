import os
import cv2
import pickle
import numpy as np
from skimage.morphology import skeletonize
from collections import deque, defaultdict
from scipy.interpolate import splprep, splev



def sample_pixels_along_line(image, p1, p2, thickness=5):
    """Возвращает массив BGR-пикселей вдоль линии p1-p2 толщиной thickness."""
    mask = np.zeros(image.shape[:2], dtype=np.uint8)
    cv2.line(mask, p1, p2, 255, thickness)
    ys, xs = np.where(mask > 0)
    return image[ys, xs]   # shape (N, 3)
def build_road_mask_from_samples(image, road_samples, exclude_samples=None,
                                 thresh=3.0):
    """алгоритм цветовой сегментации дороги с использованием расстояния Махаланобиса"""
    if len(road_samples) < 10:
        return np.zeros(image.shape[:2], dtype=np.uint8)

    samples = road_samples.astype(np.float32)
    mean = samples.mean(axis=0)
    cov  = np.cov(samples, rowvar=False) + np.eye(3) * 1.0   # регуляризация
    inv_cov = np.linalg.inv(cov)

    flat = image.reshape(-1, 3).astype(np.float32)
    diff = flat - mean
    # Расстояние Махаланобиса
    md = np.sqrt(np.einsum('ni,ij,nj->n', diff, inv_cov, diff))
    mask_road = md < thresh

    # Отсекаем пиксели, которые слишком похожи на исключения
    if exclude_samples is not None and len(exclude_samples) >= 5:
        ex = exclude_samples.astype(np.float32)
        ex_mean = ex.mean(axis=0)
        ex_cov  = np.cov(ex, rowvar=False) + np.eye(3) * 1.0
        ex_inv  = np.linalg.inv(ex_cov)
        ex_diff = flat - ex_mean
        md_ex = np.sqrt(np.einsum('ni,ij,nj->n', ex_diff, ex_inv, ex_diff))
        # Если до исключения ближе, чем до дороги — выбрасываем
        mask_road &= (md < md_ex)

    return mask_road.reshape(image.shape[:2]).astype(np.uint8) * 255

class CalibrationPicker:
    def __init__(self, image, window_name='Calibration', stroke_thickness=5):
        self.image = image
        self.window_name = window_name
        self.stroke_thickness = stroke_thickness

        self.corners = []
        self.stage = 'corners'

        self.road_samples    = np.zeros((0, 3), dtype=np.uint8)
        self.exclude_samples = np.zeros((0, 3), dtype=np.uint8)

        self.lmb_down = False
        self.rmb_down = False
        self.last_pt  = None

        self.preview_mask = None

        self.done = False
        self.cancelled = False

        cv2.namedWindow(window_name)
        cv2.setMouseCallback(window_name, self._on_mouse)
        self._redraw()

    def _on_mouse(self, event, x, y, flags, param):
        if self.stage == 'corners':
            if event == cv2.EVENT_LBUTTONDOWN and len(self.corners) < 4:
                self.corners.append((x, y))
                if len(self.corners) == 4:
                    self.stage = 'samples'
                self._redraw()
            return

        # этап 'samples'
        if event == cv2.EVENT_LBUTTONDOWN:
            self.lmb_down = True
            self.last_pt = (x, y)
            self._add_stroke((x, y), (x, y), 'road')
        elif event == cv2.EVENT_RBUTTONDOWN:
            self.rmb_down = True
            self.last_pt = (x, y)
            self._add_stroke((x, y), (x, y), 'exclude')
        elif event == cv2.EVENT_MOUSEMOVE:
            if self.lmb_down:
                self._add_stroke(self.last_pt, (x, y), 'road')
                self.last_pt = (x, y)
            elif self.rmb_down:
                self._add_stroke(self.last_pt, (x, y), 'exclude')
                self.last_pt = (x, y)
        elif event == cv2.EVENT_LBUTTONUP:
            self.lmb_down = False
            self._update_preview()
        elif event == cv2.EVENT_RBUTTONUP:
            self.rmb_down = False
            self._update_preview()

    def _add_stroke(self, p1, p2, kind):
        new = sample_pixels_along_line(self.image, p1, p2, self.stroke_thickness)
        if kind == 'road':
            self.road_samples = np.concatenate([self.road_samples, new], axis=0)
        else:
            self.exclude_samples = np.concatenate([self.exclude_samples, new], axis=0)
        # обновление превью при движении
        if len(self.road_samples) % 200 == 0:
            self._update_preview()
        self._redraw(draw_stroke=(p1, p2, kind))

    def _update_preview(self):
        if len(self.road_samples) >= 10:
            self.preview_mask = build_road_mask_from_samples(
                self.image, self.road_samples,
                self.exclude_samples if len(self.exclude_samples) > 0 else None,
            )
        self._redraw()

    def run(self):
        while True:
            key = cv2.waitKey(20) & 0xFF
            if key == 27:
                self.cancelled = True
                break
            if key == ord('r'):
                self.corners = []
                self.road_samples    = np.zeros((0, 3), dtype=np.uint8)
                self.exclude_samples = np.zeros((0, 3), dtype=np.uint8)
                self.preview_mask = None
                self.stage = 'corners'
                self._redraw()
            if key == ord('c'):
                self.road_samples    = np.zeros((0, 3), dtype=np.uint8)
                self.exclude_samples = np.zeros((0, 3), dtype=np.uint8)
                self.preview_mask = None
                self._redraw()
            if key in (13, 10):
                if len(self.corners) == 4 and len(self.road_samples) >= 10:
                    self.done = True
                    break
        cv2.destroyWindow(self.window_name)
        if self.cancelled:
            return None
        return {
            'corners': self.corners,
            'road_samples': self.road_samples,
            'exclude_samples': self.exclude_samples,
        }

    def _redraw(self, draw_stroke=None):
        vis = self.image.copy()

        # Превью маски — полупрозрачно красным
        if self.preview_mask is not None:
            overlay = vis.copy()
            overlay[self.preview_mask > 0] = (0, 0, 255)
            vis = cv2.addWeighted(vis, 0.6, overlay, 0.4, 0)

        # Углы
        for i, p in enumerate(self.corners):
            cv2.circle(vis, p, 6, (0, 255, 255), -1)
        if len(self.corners) >= 2:
            for i in range(len(self.corners) - 1):
                cv2.line(vis, self.corners[i], self.corners[i+1],
                         (0, 255, 255), 2)
            if len(self.corners) == 4:
                cv2.line(vis, self.corners[3], self.corners[0],
                         (0, 255, 255), 2)

        # Текущий штрих (для визуального отклика)
        if draw_stroke is not None:
            p1, p2, kind = draw_stroke
            color = (0, 255, 0) if kind == 'road' else (0, 100, 255)
            cv2.line(vis, p1, p2, color, self.stroke_thickness)

        # Подсказка
        if self.stage == 'corners':
            msg = f"Click 4 corners ({len(self.corners)}/4)"
        else:
            msg = (f"LMB drag = road  RMB drag = exclude   "
                   f"road:{len(self.road_samples)} ex:{len(self.exclude_samples)}   "
                   f"C=clear samples  R=reset  Enter=ok")
        cv2.putText(vis, msg, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

        cv2.imshow(self.window_name, vis)



def classify_pixels(skeleton):
    skel = (skeleton > 0).astype(np.uint8)

    # Свёртка считает кол-во соседей для каждого пикселя
    kernel = np.array([[1, 1, 1],
                             [1, 0, 1],
                             [1, 1, 1]], dtype=np.uint8)
    neighbors = cv2.filter2D(skel, cv2.CV_16S, kernel)
    neighbors = (neighbors * skel).astype(np.int16) # оставляем только для

    junctions = ((neighbors >= 3) & (skel > 0)).astype(np.uint8) * 255
    endpoints  = ((neighbors == 1) & (skel > 0)).astype(np.uint8) * 255

    return junctions, endpoints

def find_nodes(junctions, endpoints):
    """Возвращает dict: node_id → (x, y)"""
    nodes = {}
    nid = 0

    # Перекрёстки: кластеризуем близкие пиксели
    dilated = cv2.dilate(junctions, np.ones((13, 13), np.uint8))
    n_labels, labels = cv2.connectedComponents(dilated) # каждому "островку" присваивается свой порядковый номер

    for i in range(1, n_labels): # центр масс
        ys, xs = np.where(labels == i)
        cx, cy = int(np.mean(xs)), int(np.mean(ys))
        nodes[nid] = (cx, cy)
        nid += 1

    # тупики
    eys, exs = np.where(endpoints > 0)
    for ey, ex in zip(eys, exs):
        # Не добавляем, если слишком близко к уже существующему узлу
        too_close = any(abs(ex - nx) + abs(ey - ny) < 20
                        for nx, ny in nodes.values())
        if not too_close:
            nodes[nid] = (int(ex), int(ey))
            nid += 1

    return nodes

def build_graph(skeleton, nodes):
    skel = (skeleton > 0).astype(np.uint8)
    h, w = skel.shape
    R = 14

    node_map = np.full((h, w), -1, dtype=np.int32)
    for nid, (nx, ny) in nodes.items():
        y0, y1 = max(0, ny - R), min(h, ny + R + 1)
        x0, x1 = max(0, nx - R), min(w, nx + R + 1)
        for y in range(y0, y1):
            for x in range(x0, x1):
                if skel[y, x] and (x - nx)**2 + (y - ny)**2 <= R * R:
                    node_map[y, x] = nid

    graph = defaultdict(list)  # node -> [(neighbor, edge_id, dist), ...]
    edge_paths = {}  # edge_id -> [(x, y), ...]
    edge_endpoints = {}  # edge_id -> (a, b)
    next_edge_id = 0

    for start_id, (sx, sy) in nodes.items():
        starts = set()
        ys_, xs_ = np.where(node_map == start_id)
        for y, x in zip(ys_, xs_):
            starts.add((y, x))
        if not starts:
            continue

        visited = set(starts)
        parent = {}
        queue = deque()
        for y, x in starts:
            queue.append((y, x, 0.0))
            parent[(y, x)] = None

        while queue:
            cy, cx, dist = queue.popleft()
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    if dy == 0 and dx == 0:
                        continue
                    ny_, nx_ = cy + dy, cx + dx
                    if (ny_, nx_) in visited:
                        continue
                    if not (0 <= ny_ < h and 0 <= nx_ < w and skel[ny_, nx_]):
                        continue

                    visited.add((ny_, nx_))
                    parent[(ny_, nx_)] = (cy, cx)
                    step = 1.414 if abs(dy) + abs(dx) == 2 else 1.0
                    nd = dist + step

                    target = node_map[ny_, nx_]
                    if target >= 0 and target != start_id:
                        # Не дублируем рёбра: создаём только если start_id < target
                        if start_id < target:
                            # Восстанавливаем путь
                            path_pixels = []
                            cur = (ny_, nx_)
                            while cur is not None:
                                path_pixels.append((cur[1], cur[0]))
                                cur = parent.get(cur)
                            path_pixels.reverse()

                            eid = next_edge_id
                            next_edge_id += 1
                            edge_paths[eid] = path_pixels
                            edge_endpoints[eid] = (start_id, target)
                            graph[start_id].append((target, eid, nd))
                            graph[target].append((start_id, eid, nd))
                        continue
                    queue.append((ny_, nx_, nd))

    return dict(graph), edge_paths, edge_endpoints

def make_roi_mask(shape, corners):
    """Заливает четырёхугольник по углам в бинарную маску."""
    mask = np.zeros(shape[:2], dtype=np.uint8)
    pts = np.array(corners, dtype=np.int32)
    cv2.fillPoly(mask, [pts], 255)
    return mask

def cubic_bezier(p0, p1, p2, p3, num=20):
    """Возвращает num точек кубической кривой Безье."""
    t = np.linspace(0, 1, num).reshape(-1, 1)
    p0, p1, p2, p3 = map(np.array, (p0, p1, p2, p3))
    pts = ((1 - t)**3 * p0
           + 3 * (1 - t)**2 * t * p1
           + 3 * (1 - t) * t**2 * p2
           + t**3 * p3)
    return [tuple(p) for p in pts]
def build_junction_curves(lanes, connections, graph,
                          handle_ratio=0.5, uturn_handle=40.0, num_points=20):
    degree = {n: len(neighbors) for n, neighbors in graph.items()}

    junction_paths = {}
    for in_lid, outs in connections.items():
        in_lane = lanes[in_lid]
        cl_in = in_lane['centerline']
        if len(cl_in) < 2:
            continue
        k = min(5, len(cl_in) - 1)
        in_end = np.array(cl_in[-1])
        in_tan = in_end - np.array(cl_in[-1 - k])
        in_tan_len = np.linalg.norm(in_tan)
        if in_tan_len < 1e-6:
            continue
        in_tan /= in_tan_len

        for out_lid in outs:
            out_lane = lanes[out_lid]
            cl_out = out_lane['centerline']
            if len(cl_out) < 2:
                continue
            k2 = min(5, len(cl_out) - 1)
            out_start = np.array(cl_out[0])
            out_tan = np.array(cl_out[k2]) - out_start
            out_tan_len = np.linalg.norm(out_tan)
            if out_tan_len < 1e-6:
                continue
            out_tan /= out_tan_len

            # Разворот определяем строго по структуре графа:
            same_edge = in_lane['edge_id'] == out_lane['edge_id']
            shared_node = in_lane['to_node']           # узел стыка
            is_dead_end = degree.get(shared_node, 0) <= 1
            is_uturn_by_geom  = float(np.dot(in_tan, out_tan)) < -0.7
            is_uturn = (same_edge and is_dead_end) or is_uturn_by_geom

            if is_uturn:
                handle = uturn_handle
            else:
                chord = np.linalg.norm(out_start - in_end)
                handle = chord * handle_ratio

            p0 = in_end
            p1 = in_end + in_tan * handle
            p2 = out_start - out_tan * handle
            p3 = out_start

            curve = cubic_bezier(p0, p1, p2, p3, num=num_points)
            junction_paths[(in_lid, out_lid)] = curve

    return junction_paths

def connect_lanes_at_junctions(lanes, nodes, graph, allow_uturn_at_junctions=True):
    """
    Соединяет полосы в узлах. Тупики (степень 1) всегда разрешают разворот.
    Перекрёстки (степень >=2) — разворот регулируется allow_uturn_at_junctions.
    """
    degree = {n: len(neighbors) for n, neighbors in graph.items()}

    incoming = defaultdict(list)
    outgoing = defaultdict(list)

    for lid, lane in lanes.items():
        cl = lane['centerline']
        if len(cl) < 2:
            continue
        end_dir = np.array(cl[-1]) - np.array(cl[-min(5, len(cl))])
        end_dir /= (np.linalg.norm(end_dir) + 1e-9)
        incoming[lane['to_node']].append((lid, end_dir))

        start_dir = np.array(cl[min(5, len(cl)-1)]) - np.array(cl[0])
        start_dir /= (np.linalg.norm(start_dir) + 1e-9)
        outgoing[lane['from_node']].append((lid, start_dir))

    connections = defaultdict(list)
    for node_id in nodes:
        ins  = incoming.get(node_id, [])
        outs = outgoing.get(node_id, [])
        is_dead_end = degree.get(node_id, 0) <= 1

        for in_lid, in_dir in ins:
            for out_lid, out_dir in outs:
                same_edge = lanes[in_lid]['edge_id'] == lanes[out_lid]['edge_id']

                cross = in_dir[0]*out_dir[1] - in_dir[1]*out_dir[0]
                dot   = in_dir[0]*out_dir[0] + in_dir[1]*out_dir[1]
                angle = np.degrees(np.arctan2(cross, dot))
                is_uturn = abs(angle) > 150

                if is_dead_end:
                    # В тупике соединяем всё со всем (кроме сам с собой)
                    if in_lid != out_lid:
                        connections[in_lid].append(out_lid)
                else:
                    # На перекрёстке
                    if same_edge and not is_uturn:
                        continue
                    if is_uturn and not allow_uturn_at_junctions:
                        continue
                    connections[in_lid].append(out_lid)

    return dict(connections)


def smooth_polyline(pixels, smoothing=5.0, num_points=None):
    """Сглаживает полилинию B-сплайном. Возвращает список (x, y) float."""
    if len(pixels) < 4:
        return [(float(x), float(y)) for x, y in pixels]
    pts = np.array(pixels, dtype=np.float64)
    # Убираем дубликаты подряд (splprep их не любит)
    diffs = np.diff(pts, axis=0)
    keep = np.concatenate([[True], np.any(diffs != 0, axis=1)])
    pts = pts[keep]
    if len(pts) < 4:
        return [(float(x), float(y)) for x, y in pts]
    try:
        tck, u = splprep([pts[:, 0], pts[:, 1]], s=smoothing, k=3)
        n = num_points or len(pts)
        u_new = np.linspace(0, 1, n)
        x_new, y_new = splev(u_new, tck)
        return list(zip(x_new.tolist(), y_new.tolist()))
    except Exception:
        return [(float(x), float(y)) for x, y in pts]
def compute_normals(polyline):
    """Для каждой точки полилинии — единичный вектор нормали (слева от направления движения)."""
    pts = np.array(polyline, dtype=np.float64)
    # Касательные через центральные разности
    tangents = np.zeros_like(pts)
    tangents[1:-1] = pts[2:] - pts[:-2]
    tangents[0]    = pts[1]  - pts[0]
    tangents[-1]   = pts[-1] - pts[-2]
    norms = np.linalg.norm(tangents, axis=1, keepdims=True)
    norms[norms == 0] = 1
    tangents /= norms
    # Левая нормаль в экранных координатах (y растёт вниз):
    # поворот касательной (tx, ty) на 90° против часовой в "обычной" системе =
    # = (-ty, tx); но т.к. y инвертирован, "слева" = (ty, -tx). Проверим эмпирически.
    normals = np.stack([tangents[:, 1], -tangents[:, 0]], axis=1)
    return tangents, normals
def offset_polyline(polyline, dist_transform, side):
    """
    Сдвигает полилинию на половину локальной полу-ширины в сторону side (+1 или -1).
    side=+1 — по нормали (условно "влево"), side=-1 — по противоположной нормали.
    """
    pts = np.array(polyline, dtype=np.float64)
    _, normals = compute_normals(polyline)
    h, w = dist_transform.shape
    offset = []
    for (x, y), n in zip(pts, normals):
        ix, iy = int(round(x)), int(round(y))
        if 0 <= ix < w and 0 <= iy < h:
            half_width = dist_transform[iy, ix]   # расстояние до края дороги
        else:
            half_width = 10.0
        # Центр полосы = центр дороги ± половина полу-ширины
        shift = (half_width / 2.0) * side
        offset.append((x + n[0] * shift, y + n[1] * shift)) # уравнение смещения точки вдоль вектора
    return offset
def trim_polyline_at_node(pixels, node_xy, radius, from_start=True):
    """
    Возвращает индекс первой точки полилинии, которая выходит за пределы
    круга радиуса `radius` вокруг `node_xy`
    """
    nx, ny = node_xy
    r2 = radius * radius
    indices = range(len(pixels)) if from_start else range(len(pixels) - 1, -1, -1)
    for i in indices:
        x, y = pixels[i]
        if (x - nx) ** 2 + (y - ny) ** 2 > r2:
            return i
    # Вся полилиния внутри круга — возвращаем самый дальний конец
    return indices[-1]

def build_lanes_from_edges(edge_paths, edge_endpoints, graph, nodes,
                           dist_transform, junction_radius=50,
                           min_segment_px=10):
    degree = {n: len(neighbors) for n, neighbors in graph.items()}

    lanes = {}
    lane_id = 0
    for eid, pixels in edge_paths.items():
        a, b = edge_endpoints[eid]

        if degree.get(a, 0) >= 2:
            i_start = trim_polyline_at_node(pixels, nodes[a],
                                            junction_radius, from_start=True)
        else:
            i_start = 0

        # Конечный индекс: либо len(pixels), либо выход из зоны узла b
        if degree.get(b, 0) >= 2:
            i_end = trim_polyline_at_node(pixels, nodes[b],
                                          junction_radius, from_start=False) + 1
        else:
            i_end = len(pixels)

        if i_end - i_start < min_segment_px:
            # Если перекрестки a и b находятся слишком близко друг к другу
            shrunk = max(4, junction_radius // 2)
            if degree.get(a, 0) >= 2:
                i_start = trim_polyline_at_node(pixels, nodes[a], shrunk, from_start=True)
            else:
                i_start = 0
            if degree.get(b, 0) >= 2:
                i_end = trim_polyline_at_node(pixels, nodes[b], shrunk, from_start=False) + 1
            else:
                i_end = len(pixels)
            if i_end - i_start < 4:
                continue   # уже совсем некуда

        trimmed = pixels[i_start:i_end]
        smooth = smooth_polyline(trimmed, smoothing=10.0,
                                 num_points=max(20, len(trimmed) // 3))

        for side in (+1, -1):
            lane_pts = offset_polyline(smooth, dist_transform, side)
            if side == +1:
                lane_pts = lane_pts[::-1]
                from_node, to_node = b, a
            else:
                from_node, to_node = a, b

            lanes[lane_id] = {
                'edge_id': eid,
                'side': side,
                'centerline': lane_pts,
                'from_node': from_node,
                'to_node': to_node,
            }
            lane_id += 1
    return lanes


def build_map_from_frame(frame, calib):
    """Принимает кадр и результат калибровки, возвращает карту."""
    roi = make_roi_mask(frame.shape, calib['corners'])
    img_clipped = frame.copy()
    img_clipped[roi == 0] = (0, 0, 0)

    mask = build_road_mask_from_samples(
        img_clipped, calib['road_samples'],
        calib['exclude_samples'] if len(calib['exclude_samples']) > 0 else None,
    )
    kern = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kern, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kern, iterations=1)

    skeleton = skeletonize(mask > 0).astype(np.uint8) * 255
    juncs, endpts = classify_pixels(skeleton)
    nodes = find_nodes(juncs, endpts)
    graph, edge_paths, edge_endpoints = build_graph(skeleton, nodes)
    dist_tr = cv2.distanceTransform(mask, cv2.DIST_L2, 5)
    lanes = build_lanes_from_edges(edge_paths, edge_endpoints, graph, nodes,
                                   dist_tr, junction_radius=50)
    connections = connect_lanes_at_junctions(lanes, nodes, graph,
                                             allow_uturn_at_junctions=True)
    junction_paths = build_junction_curves(lanes, connections, graph,
                                           handle_ratio=0.5, uturn_handle=50.0)

    return {
        'corners': calib['corners'],
        'nodes': nodes,
        'edge_paths': edge_paths,
        'edge_endpoints': edge_endpoints,
        'graph': graph,
        'lanes': lanes,
        'connections': connections,
        'junction_paths': junction_paths,
        'base_image': frame,
        'frame_shape': frame.shape,
        'debug_mask': mask,
        'debug_skeleton': skeleton,
}


def visualize_lanes(image, lanes, connections=None, junction_paths=None):
    vis = image.copy()
    for lid, lane in lanes.items():
        pts = np.array(lane['centerline'], dtype=np.int32).reshape(-1, 1, 2)
        color = (0, 200, 0)
        cv2.polylines(vis, [pts], False, color, 1, cv2.LINE_AA)
        ARROW_TIP = 8

        cl = lane['centerline']
        if len(cl) >= 2:
            mid = int(len(cl) / 1.75)
            p_mid = np.array(cl[mid], dtype=np.float64)

            k = min(3, len(cl) - 1 - mid, mid)
            if k > 0:
                tan = np.array(cl[mid + k]) - np.array(cl[mid - k])
                tan_len = np.linalg.norm(tan)
                if tan_len > 1e-6:
                    tan /= tan_len
                    p1 = p_mid - tan * (ARROW_TIP / 2)
                    p2 = p_mid + tan * (ARROW_TIP / 2)
                    cv2.arrowedLine(vis,
                                    tuple(map(int, p1)),
                                    tuple(map(int, p2)),
                                    color, 2, tipLength=1.0, line_type=cv2.LINE_AA)

    # Связи на перекрёстках — плавными кривыми
    if junction_paths:
        for (in_lid, out_lid), curve in junction_paths.items():
            pts = np.array(curve, dtype=np.int32).reshape(-1, 1, 2)
            cv2.polylines(vis, [pts], False, (0, 255, 255), 1, cv2.LINE_AA)
    elif connections:
        # прямые линии
        for in_lid, outs in connections.items():
            in_end = lanes[in_lid]['centerline'][-1]
            for out_lid in outs:
                out_start = lanes[out_lid]['centerline'][0]
                p1 = tuple(map(int, in_end))
                p2 = tuple(map(int, out_start))
                cv2.line(vis, p1, p2, (0, 255, 255), 1, cv2.LINE_AA)
    return vis

def grab_frame(src):
    """src: 'путь.png' или число-индекс камеры."""
    if isinstance(src, str) and not src.isdigit():
        return cv2.imread("Images/" + src)
    cap = cv2.VideoCapture(int(src))
    ret, frame = cap.read()
    cap.release()
    return frame if ret else None
def visualize_debug(frame, mask, skeleton, nodes, graph, edge_paths, edge_endpoints):
    """Несколько окон для диагностики построения карты."""
    # 1. Маска дороги
    cv2.imshow('1. Road mask', mask)

    # 2. Скелет поверх кадра
    skel_vis = frame.copy()
    skel_vis[skeleton > 0] = (0, 255, 255)
    cv2.imshow('2. Skeleton', skel_vis)

    # 3. Граф: узлы + рёбра
    graph_vis = frame.copy()
    graph_vis[skeleton > 0] = (60, 60, 60)
    for eid, pixels in edge_paths.items():
        a, b = edge_endpoints[eid]
        for i in range(len(pixels) - 1):
            cv2.line(graph_vis, pixels[i], pixels[i + 1], (255, 200, 0), 1)
    degree = {n: len(neighbors) for n, neighbors in graph.items()}
    for nid, (x, y) in nodes.items():
        d = degree.get(nid, 0)
        # Тупики — зелёным, перекрёстки — красным
        color = (0, 255, 0) if d <= 1 else (0, 0, 255)
        cv2.circle(graph_vis, (x, y), 8, color, -1)
        cv2.putText(graph_vis, f"{nid}({d})", (x + 10, y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.imshow('3. Graph (green=dead-end, red=junction)', graph_vis)
def save_map(path, data):
    with open(path, 'wb') as f:
        pickle.dump(data, f)

def build_map_flow():
    src = input("Image path or camera index: ").strip()
    frame = grab_frame(src)
    if frame is None:
        print("Cannot read frame")
        return

    calib = CalibrationPicker(frame).run()
    if calib is None:
        return

    map_data = build_map_from_frame(frame, calib)

    degree = {n: len(neighbors) for n, neighbors in map_data['graph'].items()}
    dead_ends = [n for n, d in degree.items() if d <= 1]
    print(f"Тупиков в графе: {len(dead_ends)}")
    print(f"Полос всего: {len(map_data['lanes'])}")
    print(f"Связей всего: {sum(len(v) for v in map_data['connections'].values())}")

    for nid in dead_ends:
        in_lanes = [lid for lid, l in map_data['lanes'].items() if l['to_node'] == nid]
        out_lanes = [lid for lid, l in map_data['lanes'].items() if l['from_node'] == nid]
        cons_from_in = sum(len(map_data['connections'].get(lid, [])) for lid in in_lanes)
        print(f"  Узел {nid}: входящих полос {len(in_lanes)}, "
              f"исходящих {len(out_lanes)}, связей из входящих {cons_from_in}")

    visualize_debug(frame, map_data['debug_mask'], map_data['debug_skeleton'],
                    map_data['nodes'], map_data['graph'],
                    map_data['edge_paths'], map_data['edge_endpoints'])

    preview = visualize_lanes(map_data['base_image'], map_data['lanes'],
                              map_data['connections'], map_data['junction_paths'])
    cv2.imshow('4. Lanes', preview)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    name = input("Map name (without .pkl): ").strip() or "map"
    os.makedirs("maps", exist_ok=True)
    path = os.path.join("maps", f"{name}.pkl")
    save_map(path, map_data)
    print(f"Saved {path}")


if __name__ == '__main__':
    build_map_flow()