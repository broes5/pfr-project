class Vector3Int:
    __slots__ = ('x', 'y', 'z')

    def __init__(self, x=0, y=0, z=0):
        self.x, self.y, self.z = int(x), int(y), int(z)

    def __add__(self, other):
        return Vector3Int(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other):
        return Vector3Int(self.x - other.x, self.y - other.y, self.z - other.z)

    def __eq__(self, other):
        return (self.x, self.y, self.z) == (other.x, other.y, other.z)

    def __hash__(self):
        return hash((self.x, self.y, self.z))

    def __lt__(self, other):
        return (self.x, self.y, self.z) < (other.x, other.y, other.z)

    def __repr__(self):
        return f"Vector3Int({self.x}, {self.y}, {self.z})"
