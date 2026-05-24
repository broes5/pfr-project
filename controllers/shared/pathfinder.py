import heapq
import math

from .vector3 import Vector3
from .vector3_int import Vector3Int
from .path import Path
from .voxel_world import VoxelWorld

# 6-connected (face-adjacent) movement only — no diagonals.
# Diagonals would allow drones to slip through voxel corners, creating apparent
# collisions with occupied cells that share only an edge.
_NEIGHBOURS = [
    Vector3Int(1, 0, 0), Vector3Int(-1, 0, 0),
    Vector3Int(0, 1, 0), Vector3Int(0, -1, 0),
    Vector3Int(0, 0, 1), Vector3Int(0, 0, -1),
]

_STEP_COSTS = {
    n: math.sqrt(n.x * n.x + n.y * n.y + n.z * n.z)
    for n in _NEIGHBOURS
}


def _h(a: Vector3Int, b: Vector3Int) -> float:
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)


def _cull_collinear(points: list) -> list:
    if len(points) <= 2:
        return points
    result = [points[0]]
    for i in range(1, len(points) - 1):
        d1 = points[i] - points[i - 1]
        d2 = points[i + 1] - points[i]
        if d1.cross(d2).sqr_magnitude > 1e-10:
            result.append(points[i])
    result.append(points[-1])
    return result


def find_path(world: VoxelWorld, start: Vector3, goal: Vector3, goal_tolerance: int = 2):
    """Return a Path of world-space waypoints from start to goal, or None if unreachable.

    Euclidean distance is used as the A* heuristic — admissible and consistent for a
    uniform-cost grid, so the first path found is always optimal.

    If the exact goal voxel is occupied (e.g. another drone already reserved it),
    the nearest free voxel within goal_tolerance in XY at the same Z is tried instead.
    This prevents hard failures when two drones target adjacent pile positions.
    """
    sv = world.world_to_voxel(start)
    gv = world.world_to_voxel(goal)

    if world.is_voxel_occupied(sv):
        return None

    if world.is_voxel_occupied(gv):
        if goal_tolerance <= 0:
            return None
        best_gv = None
        best_d2 = float('inf')
        for dx in range(-goal_tolerance, goal_tolerance + 1):
            for dy in range(-goal_tolerance, goal_tolerance + 1):
                candidate = Vector3Int(gv.x + dx, gv.y + dy, gv.z)
                if not world.is_voxel_occupied(candidate):
                    d2 = dx * dx + dy * dy
                    if d2 < best_d2:
                        best_d2 = d2
                        best_gv = candidate
        if best_gv is None:
            return None
        gv = best_gv

    if sv == gv:
        return Path([world.voxel_to_world(sv)])

    open_heap = [(_h(sv, gv), 0.0, sv)]
    came_from = {}
    g_score = {sv: 0.0}
    closed = set()
    # Guard against runaway search on degenerate worlds (max nodes << world size)
    MAX_ITER = 50_000
    _iter = 0

    while open_heap:
        _iter += 1
        if _iter > MAX_ITER:
            print(f"[A*] Exceeded {MAX_ITER} iterations — aborting pathfind")
            return None
        _, g, current = heapq.heappop(open_heap)

        if current in closed:
            continue
        closed.add(current)

        if current == gv:
            path = Path()
            node = current
            while node in came_from:
                path.append(world.voxel_to_world(node))
                node = came_from[node]
            path.append(world.voxel_to_world(sv))
            path._points.reverse()

            path._points[0] = start
            path._points[-1] = goal

            path._points = _cull_collinear(path._points)
            return path

        for delta in _NEIGHBOURS:
            neighbour = current + delta
            if neighbour in closed or world.is_voxel_occupied(neighbour):
                continue
            new_g = g + _STEP_COSTS[delta]
            if new_g < g_score.get(neighbour, float('inf')):
                g_score[neighbour] = new_g
                came_from[neighbour] = current
                heapq.heappush(open_heap, (new_g + _h(neighbour, gv), new_g, neighbour))

    return None
