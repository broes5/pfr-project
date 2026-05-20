import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from shared.vector3 import Vector3
from shared.brick_state import BrickState

SCALE = 0.02
Z_SCALE = 0.02

def parse_brick_file(path):
    bricks = []

    with open(path, "r") as f:
        for lineno, raw_line in enumerate(f, start=1):
            line = raw_line.strip()

            if not line or line.startswith("#") or line == "new_layer":
                continue

            parts = line.split()
            if len(parts) != 4:
                print(f"Line {lineno}: skipping malformed line: {raw_line!r}")
                continue

            try:
                x, y, z, theta = (float(p) for p in parts)
            except ValueError:
                print(f"Line {lineno}: non-numeric value: {raw_line!r}")
                continue

            brick = (x, y, z, theta)
            if bricks and bricks[-1] == brick:
                continue
            bricks.append(brick)

    return bricks


def to_world_coords(bricks, scale=SCALE):
    return [BrickState(Vector3(x * scale, y * scale, z * Z_SCALE), theta) for x, y, z, theta in bricks]