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

    def is_occupied(self, pos: Vector3) -> bool:
        return self.is_voxel_occupied(self.world_to_voxel(pos))

    def is_voxel_occupied(self, vox: Vector3Int) -> bool:
        if not self.in_bounds(vox):
            return True  # Cannot path out of bounds
        return bool(self._grid[vox.x, vox.y, vox.z])
