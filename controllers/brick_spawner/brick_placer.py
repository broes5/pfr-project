import math

class BrickPlacer:
    def __init__(self, supervisor):
        self.root_children = supervisor.getRoot().getField("children")

    def spawn_brick(self, name, x, y, z, theta, physics=True):
        proto_name = "BrickPhy" if physics else "BrickStill"
        z += 0.15 / 2
        rad = theta * math.pi / 180   # Convert degrees to radians

        brick_string = (
            f'{proto_name} {{ '
            f'translation {x} {y} {z} '
            f'rotation {0} {0} {1} {rad} '
            f'name "{name}" '
            f'}}'
        )
        self.root_children.importMFNodeFromString(-1, brick_string)

    def spawn_many(self, bricks, name_prefix="brick", physics=True):
        for i, (x, y, z, theta) in enumerate(bricks):
            self.spawn_brick(f"{name_prefix}_{i:03d}", x, y, z, theta, physics=physics)
        print(f"Spawned {len(bricks)} bricks (physics={physics}).")
