class Color:
    __slots__ = ('r', 'g', 'b')

    def __init__(self, r=0.0, g=0.0, b=0.0):
        self.r, self.g, self.b = float(r), float(g), float(b)

    def __add__(self, other):
        return Color(self.r + other.r, self.g + other.g, self.b + other.b)

    def __sub__(self, other):
        return Color(self.r - other.r, self.g - other.g, self.b - other.b)

    def __mul__(self, scalar):
        if isinstance(scalar, Color):
            return Color(self.r * scalar.r, self.g * scalar.g, self.b * scalar.b)
        return Color(self.r * scalar, self.g * scalar, self.b * scalar)

    __rmul__ = __mul__

    def __truediv__(self, scalar):
        return Color(self.r / scalar, self.g / scalar, self.b / scalar)

    def __eq__(self, other):
        return (self.r, self.g, self.b) == (other.r, other.g, other.b)

    def __iter__(self):
        return iter((self.r, self.g, self.b))

    def __repr__(self):
        return f"Color({self.r}, {self.g}, {self.b})"

    @classmethod
    def from_list(cls, values):
        return cls(values[0], values[1], values[2])

    def to_list(self):
        return [self.r, self.g, self.b]

    @property
    def grayscale(self):
        return 0.299 * self.r + 0.587 * self.g + 0.114 * self.b

    @property
    def max_color_component(self):
        return max(self.r, self.g, self.b)

    @staticmethod
    def lerp(a, b, t):
        t = max(0.0, min(1.0, t))
        return Color(a.r + (b.r - a.r) * t, a.g + (b.g - a.g) * t, a.b + (b.b - a.b) * t)

    @staticmethod
    def lerp_unclamped(a, b, t):
        return Color(a.r + (b.r - a.r) * t, a.g + (b.g - a.g) * t, a.b + (b.b - a.b) * t)

# Constants - Straight from Unity Engine
Color.red     = Color(1, 0, 0)
Color.green   = Color(0, 1, 0)
Color.blue    = Color(0, 0, 1)
Color.white   = Color(1, 1, 1)
Color.black   = Color(0, 0, 0)
Color.yellow  = Color(1, 0.92, 0.016)
Color.cyan    = Color(0, 1, 1)
Color.magenta = Color(1, 0, 1)
Color.gray    = Color(0.5, 0.5, 0.5)
Color.grey    = Color.gray
