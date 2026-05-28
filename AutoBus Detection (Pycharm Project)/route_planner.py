import heapq
import cv2
import numpy as np


def find_lane_for_point(point, lanes, heading_deg=None, heading_weight=30):
    bus_dir = None
    if heading_deg is not None:
        a = np.radians(heading_deg)
        bus_dir = np.array([np.cos(a), -np.sin(a)])

    best = None
    for lid, lane in lanes.items():
        d, t, proj = closest_point_on_polyline(point, lane['centerline'])
        score = d
        if bus_dir is not None:
            tan = lane_direction_at(lane, t)
            align = float(np.dot(bus_dir, tan))
            score = d - heading_weight * align
        if best is None or score < best[0]:
            best = (score, lid, t, proj, d)
    return None if best is None else (best[1], best[2], best[3], best[4])


def lane_direction_at(lane, t):
    """Единичный касательный вектор в точке"""
    cl = lane['centerline']
    if len(cl) < 2:
        return np.array([1.0, 0.0])
    idx = int(round(t * (len(cl) - 1)))
    idx = max(1, min(len(cl) - 1, idx))
    v = np.array(cl[idx]) - np.array(cl[idx - 1])
    n = np.linalg.norm(v)
    return v / n if n > 1e-9 else np.array([1.0, 0.0])


def polyline_length(polyline):
    pts = np.array(polyline, dtype=np.float64)
    if len(pts) < 2:
        return 0.0
    diffs = np.diff(pts, axis=0)
    return float(np.sum(np.linalg.norm(diffs, axis=1)))


def closest_point_on_polyline(point, polyline):
    pts = np.array(polyline, dtype=np.float64)
    p = np.array(point, dtype=np.float64)
    if len(pts) < 2:
        return float(np.linalg.norm(p - pts[0])), 0.0, tuple(pts[0])

    seg = np.diff(pts, axis=0)
    seg_len = np.linalg.norm(seg, axis=1)
    total = float(np.sum(seg_len))
    if total < 1e-9:
        return float(np.linalg.norm(p - pts[0])), 0.0, tuple(pts[0])

    cum = np.concatenate([[0], np.cumsum(seg_len)])

    best = (float('inf'), 0.0, tuple(pts[0]))
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
            arc = cum[i] + u * seg_len[i]
            best = (d, arc / total, tuple(proj))
    return best


def plan_route(start_point, end_point, lanes, connections, junction_paths,
               start_heading_deg=None, end_heading_deg=None):
    start = find_lane_for_point(start_point, lanes, start_heading_deg)
    end = find_lane_for_point(end_point, lanes, end_heading_deg)
    if start is None or end is None:
        return None

    start_lid, start_t, start_proj, start_d = start
    end_lid, end_t, end_proj, end_d = end

    direct_dist = float(np.linalg.norm(np.array(end_point) - np.array(start_point)))

    if direct_dist <= start_d + end_d:
        return {
            'lanes': [],
            'maneuvers': [],
            'polyline': [tuple(start_point), tuple(end_point)],
            'cost': direct_dist,
            'direct': True,
        }

    if start_lid == end_lid and end_t >= start_t:
        cl = lanes[start_lid]['centerline']
        road_polyline = sub_polyline(cl, start_t, end_t, start_proj, end_proj)
        road_lanes = [start_lid]
        road_maneuvers = []
        road_cost = polyline_length(road_polyline)
    else:
        result = _dijkstra_lanes(start_lid, end_lid, lanes, connections,
                                 junction_paths)
        if result is None:
            return {
                'lanes': [], 'maneuvers': [],
                'polyline': [tuple(start_point), tuple(end_point)],
                'cost': direct_dist, 'direct': True,
            }
        seq, road_cost = result
        road_maneuvers = list(zip(seq[:-1], seq[1:]))
        road_lanes = seq
        road_polyline = _stitch_polyline(seq, road_maneuvers, lanes,
                                         junction_paths,
                                         start_t, start_proj,
                                         end_t, end_proj)

    polyline = list(road_polyline)
    polyline.append(tuple(end_point))

    return {
        'lanes': road_lanes,
        'maneuvers': road_maneuvers,
        'polyline': polyline,
        'cost': start_d + road_cost + end_d,
        'direct': False,
    }


def sub_polyline(polyline, t_start, t_end, override_start=None, override_end=None):
    pts = list(polyline)
    if len(pts) < 2:
        return pts

    seg = np.diff(np.array(pts), axis=0)
    seg_len = np.linalg.norm(seg, axis=1)
    total = float(np.sum(seg_len))
    if total < 1e-9:
        return pts
    cum = np.concatenate([[0], np.cumsum(seg_len)]) / total

    def point_at(t):
        for i in range(len(cum) - 1):
            if cum[i] <= t <= cum[i + 1]:
                local = (t - cum[i]) / (cum[i + 1] - cum[i] + 1e-12)
                a = np.array(pts[i])
                b = np.array(pts[i + 1])
                return tuple(a + local * (b - a)), i
        return pts[-1], len(pts) - 1

    p_start, i_start = point_at(t_start)
    p_end, i_end = point_at(t_end)
    if override_start is not None: p_start = tuple(override_start)
    if override_end is not None: p_end = tuple(override_end)

    result = [p_start]
    for i in range(i_start + 1, i_end + 1):
        result.append(pts[i])
    result.append(p_end)
    return result


def _dijkstra_lanes(start_lid, end_lid, lanes, connections,
                    junction_paths):
    pq = [(0.0, start_lid)]
    dist = {start_lid: 0.0}
    prev = {}
    while pq:
        d, u = heapq.heappop(pq)
        if u == end_lid:
            break
        if d > dist.get(u, float('inf')):
            continue
        for v in connections.get(u, []):
            curve = junction_paths.get((u, v), [])
            curve_len = polyline_length(curve)
            v_len = polyline_length(lanes[v]['centerline'])
            nd = d + curve_len + v_len
            if nd < dist.get(v, float('inf')):
                dist[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd, v))
    if end_lid not in dist:
        return None
    seq = [end_lid]
    while seq[-1] != start_lid:
        seq.append(prev[seq[-1]])
    seq.reverse()
    return seq, dist[end_lid]


def _stitch_polyline(seq, maneuvers, lanes, junction_paths,
                     start_t, start_proj, end_t, end_proj):
    polyline = []
    polyline.extend(sub_polyline(lanes[seq[0]]['centerline'],
                                 start_t, 1.0, start_proj, None))
    for in_lid, out_lid in maneuvers:
        curve = junction_paths.get((in_lid, out_lid))
        if curve:
            polyline.extend(curve[1:])
        is_last = (out_lid == seq[-1])
        cl_out = lanes[out_lid]['centerline']
        if is_last:
            polyline.extend(sub_polyline(cl_out, 0.0, end_t, None, end_proj)[1:])
        else:
            polyline.extend(cl_out[1:])
    return polyline


class RoutePicker:
    def __init__(self, base_image, lanes, connections, junction_paths,
                 window_name='Route'):
        self.base = base_image
        self.lanes = lanes
        self.connections = connections
        self.junction_paths = junction_paths
        self.window_name = window_name

        self.start = None  # (x, y)
        self.end = None
        self.route = None

        cv2.namedWindow(window_name)
        cv2.setMouseCallback(window_name, self._on_mouse)
        self._redraw()

    def _on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.start is None or (self.start is not None and self.end is not None):
                # Новый цикл: ставим старт, сбрасываем конец
                self.start = (x, y)
                self.end = None
                self.route = None
            else:
                # Ставим конец и считаем маршрут
                self.end = (x, y)
                self.route = plan_route(
                    self.start, self.end,
                    self.lanes, self.connections, self.junction_paths,
                )
            self._redraw()

        elif event == cv2.EVENT_RBUTTONDOWN:
            self.start = None
            self.end = None
            self.route = None
            self._redraw()

    def _redraw(self):
        vis = self.base.copy()

        # Маршрут
        if self.route is not None:
            pts = self.route['polyline']
            color = (0, 200, 255) if self.route.get('direct') else (0, 255, 0)
            arr = np.array(pts, dtype=np.int32).reshape(-1, 1, 2)
            cv2.polylines(vis, [arr], False, color, 4, cv2.LINE_AA)

            if not self.route.get('direct') and len(pts) >= 4:
                cv2.line(vis, tuple(map(int, pts[0])), tuple(map(int, pts[1])),
                         (0, 200, 255), 2, cv2.LINE_AA)
                cv2.line(vis, tuple(map(int, pts[-2])), tuple(map(int, pts[-1])),
                         (0, 200, 255), 2, cv2.LINE_AA)

        # Старт
        if self.start is not None:
            cv2.circle(vis, self.start, 10, (0, 255, 0), -1)
            cv2.circle(vis, self.start, 10, (0, 0, 0), 2)
            cv2.putText(vis, 'A', (self.start[0] + 12, self.start[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # Конец
        if self.end is not None:
            cv2.circle(vis, self.end, 10, (0, 0, 255), -1)
            cv2.circle(vis, self.end, 10, (0, 0, 0), 2)
            cv2.putText(vis, 'B', (self.end[0] + 12, self.end[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # Длина и количество полос
        if self.route is not None:
            cv2.putText(vis, f"len={self.route['cost']:.0f}px  "
                             f"lanes={len(self.route['lanes'])}",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (0, 255, 0), 2)
        elif self.start is not None and self.end is not None:
            cv2.putText(vis, "No route found",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (0, 0, 255), 2)

        cv2.imshow(self.window_name, vis)
