import numpy as np
from .vector3 import Vector3
from .vector3_int import Vector3Int


class VoxelWorld:
    def __init__(self, origin: Vector3, size: Vector3, voxel_size: float):
        self.origin = origin
        self.voxel_size = voxel_size
        self.nx = max(1, int(size.x / voxel_size))
        self.ny = max(1, int(size.y / voxel_size))
        self.nz = max(1, int(size.z / voxel_size))
        self._grid = np.zeros((self.nx, self.ny, self.nz), dtype=bool)

    def world_to_voxel(self, pos: Vector3) -> Vector3Int:
        return Vector3Int(
            int((pos.x - self.origin.x) / self.voxel_size),
            int((pos.y - self.origin.y) / self.voxel_size),
            int((pos.z - self.origin.z) / self.voxel_size),
        )

    def voxel_to_world(self, vox: Vector3Int) -> Vector3:
        half = self.voxel_size * 0.5
        return Vector3(
            self.origin.x + vox.x * self.voxel_size + half,
            self.origin.y + vox.y * self.voxel_size + half,
            self.origin.z + vox.z * self.voxel_size + half,
        )

    def in_bounds(self, vox: Vector3Int) -> bool:
        return 0 <= vox.x < self.nx and 0 <= vox.y < self.ny and 0 <= vox.z < self.nz

    def mark_occupied(self, pos: Vector3):
        vox = self.world_to_voxel(pos)
        if self.in_bounds(vox):
            self._grid[vox.x, vox.y, vox.z] = True

    def mark_free(self, pos: Vector3):
        vox = self.world_to_voxel(pos)
        if self.in_bounds(vox):
            self._grid[vox.x, vox.y, vox.z] = False

    def mark_path_occupied(self, path):
        for wp in path:
            self.mark_occupied(wp)

    def mark_path_free(self, path):
        for wp in path:
            self.mark_free(wp)

    def mark_neighborhood_occupied(self, pos: Vector3, radius: int = 1) -> list:
        center = self.world_to_voxel(pos)
        claimed = []
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                for dz in range(-radius, radius + 1):
                    nbr = Vector3Int(center.x + dx, center.y + dy, center.z + dz)
                    if self.in_bounds(nbr):
                        self._grid[nbr.x, nbr.y, nbr.z] = True
                        claimed.append(self.voxel_to_world(nbr))
        return claimed

    def free_neighborhood(self, pos: Vector3, radius: int = 1) -> list:
        """Free a neighbourhood and return the Vector3Int coords that were occupied."""
        center = self.world_to_voxel(pos)
        freed = []
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                for dz in range(-radius, radius + 1):
                    nbr = Vector3Int(center.x + dx, center.y + dy, center.z + dz)
                    if self.in_bounds(nbr) and self._grid[nbr.x, nbr.y, nbr.z]:
                        freed.append(nbr)
                        self._grid[nbr.x, nbr.y, nbr.z] = False
        return freed

    def restore_voxels(self, voxels: list):
        """Re-mark a list of Vector3Int voxels as occupied."""
        for v in voxels:
            if self.in_bounds(v):
                self._grid[v.x, v.y, v.z] = True

    def mark_path_neighborhood_occupied(self, path, radius: int = 1) -> list:
        """Mark a 3x3x3 neighbourhood for every voxel along each segment of the path.

        Interpolates at voxel_size steps between consecutive culled waypoints so that
        the full straight-line corridor is reserved, not just the direction-change points
        that A* returns after collinear culling.
        """
        points = list(path)
        if not points:
            return []
        claimed = []
        # Single point
        if len(points) == 1:
            claimed.extend(self.mark_neighborhood_occupied(points[0], radius))
            return claimed
        for i in range(len(points) - 1):
            p0, p1 = points[i], points[i + 1]
            dx = p1.x - p0.x
            dy = p1.y - p0.y
            dz = p1.z - p0.z
            dist = (dx * dx + dy * dy + dz * dz) ** 0.5
            steps = max(1, int(dist / self.voxel_size) + 1)
            seen = set()
            for j in range(steps + 1):
                t = j / steps
                p = Vector3(p0.x + t * dx, p0.y + t * dy, p0.z + t * dz)
                vox = self.world_to_voxel(p)
                key = (vox.x, vox.y, vox.z)
                if key not in seen:
                    seen.add(key)
                    claimed.extend(self.mark_neighborhood_occupied(p, radius))
        return claimed

    def is_occupied(self, pos: Vector3) -> bool:
        return self.is_voxel_occupied(self.world_to_voxel(pos))

    def is_voxel_occupied(self, vox: Vector3Int) -> bool:
        if not self.in_bounds(vox):
            return True  # Cannot path out of bounds
        return bool(self._grid[vox.x, vox.y, vox.z])
