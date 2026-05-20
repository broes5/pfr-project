from .vector3 import Vector3

class BrickState:
    def __init__(self, position: Vector3, rotationZ: float):
        self.position = position
        self.rotationZ = rotationZ
