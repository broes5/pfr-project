import math

class BrickPlacer:
    def __init__(self, supervisor):
        self.root_children = supervisor.getRoot().getField("children")

    def spawn_brick(self, name, theta, x, y, z=0.0756, physics=True):
        proto_name = "BrickPhy" if physics else "BrickStill"
        z += 0.15 / 2
        rad=theta*math.pi/180   # Convert degrees to radians.

        brick_string = (
            f'{proto_name} {{ '
            f'translation {x} {y} {z} '
            f'rotation {0} {0} {0} {rad} '
            f'name "{name}" '
            f'}}'
        )
        self.root_children.importMFNodeFromString(-1, brick_string)

    def spawn_many(self, positions, name_prefix="brick", physics=True):
        for i, (x, y, z, theta) in enumerate(positions):
            self.spawn_brick(f"{name_prefix}_{i:03d}", x, y, z, theta, physics=physics)
        print(f"Spawned {len(positions)} bricks (physics={physics}).")
