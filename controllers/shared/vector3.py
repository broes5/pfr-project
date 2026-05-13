import math

class Vector3:
    __slots__ = ('x', 'y', 'z') # Stops typos

    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = float(x), float(y), float(z)

    def __add__(self, other):
        return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other):
        return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, scalar):
        return Vector3(self.x * scalar, self.y * scalar, self.z * scalar)

    __rmul__ = __mul__ 

    def __truediv__(self, scalar):
        return Vector3(self.x / scalar, self.y / scalar, self.z / scalar)

    def __neg__(self):
        return Vector3(-self.x, -self.y, -self.z)

    def __eq__(self, other):
        return (self.x, self.y, self.z) == (other.x, other.y, other.z)

    def __repr__(self):
        return f"Vector3({self.x}, {self.y}, {self.z})"

    @classmethod
    def from_list(cls, values):
        return cls(values[0], values[1], values[2])

    def to_list(self):
        return [self.x, self.y, self.z]

    @property
    def magnitude(self):
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)

    @property
    def sqr_magnitude(self):
        return self.x**2 + self.y**2 + self.z**2

    @property
    def normalized(self):
        m = self.magnitude
        return Vector3(0, 0, 0) if m == 0 else self / m

    def dot(self, other):
        return self.x*other.x + self.y*other.y + self.z*other.z

    def cross(self, other):
        return Vector3(
            self.y*other.z - self.z*other.y,
            self.z*other.x - self.x*other.z,
            self.x*other.y - self.y*other.x,
        )

    @staticmethod
    def distance(a, b):
        return (a - b).magnitude

    @staticmethod
    def lerp(a, b, t):
        t = max(0.0, min(1.0, t))
        return a + (b - a) * t

# Constants
Vector3.zero    = Vector3(0, 0, 0)
Vector3.one     = Vector3(1, 1, 1)
Vector3.up      = Vector3(0, 0, 1)
Vector3.down    = Vector3(0, 0, -1)
Vector3.right   = Vector3(1, 0, 0)
Vector3.left    = Vector3(-1, 0, 0)
Vector3.forward = Vector3(0, 1, 0)
Vector3.back    = Vector3(0, -1, 0)