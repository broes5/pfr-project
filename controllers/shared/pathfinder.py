import heapq
import math

from .vector3 import Vector3
from .vector3_int import Vector3Int
from .path import Path
from .voxel_world import VoxelWorld

# All 26 neighbours (face, edge, and corner adjacency)
_NEIGHBOURS = [
    Vector3Int(dx, dy, dz)
    for dx in (-1, 0, 1)
    for dy in (-1, 0, 1)
    for dz in (-1, 0, 1)
    if not (dx == 0 and dy == 0 and dz == 0)
]

_STEP_COSTS = {
    n: math.sqrt(n.x * n.x + n.y * n.y + n.z * n.z)
    for n in _NEIGHBOURS
}


def _h(a: Vector3Int, b: Vector3Int) -> float:
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)


def find_path(world: VoxelWorld, start: Vector3, goal: Vector3):
    """Return a Path of world-space waypoints from start to goal, or None if unreachable."""
    sv = world.world_to_voxel(start)
    gv = world.world_to_voxel(goal)

    if world.is_voxel_occupied(sv) or world.is_voxel_occupied(gv):
        return None

    if sv == gv:
        return Path([world.voxel_to_world(sv)])

    open_heap = [(_h(sv, gv), 0.0, sv)]
    came_from = {}
    g_score = {sv: 0.0}
    closed = set()

    while open_heap:
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
