from .vector3 import Vector3

class Path:
    def __init__(self, waypoints=None):
        self._points: list[Vector3] = list(waypoints) if waypoints else []

    def append(self, point: Vector3):
        self._points.append(point)

    def __len__(self):
        return len(self._points)

    def __getitem__(self, index):
        return self._points[index]

    def __iter__(self):
        return iter(self._points)

    def __repr__(self):
        return f"Path({self._points})"

    def cullAdjacents(self):
        printf('System Not Implemented Exception')
