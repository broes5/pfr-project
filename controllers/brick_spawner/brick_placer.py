class BrickPlacer:
    def __init__(self, supervisor):
        self.root_children = supervisor.getRoot().getField("children")

    def spawn_brick(self, name, x, y, z=0.0756, rotation=(0, 0, 1, 0), physics=True):
        proto_name = "BrickPhy" if physics else "BrickStill"
        rx, ry, rz, ra = rotation
        z += 0.15 / 2
        brick_string = (
            f'{proto_name} {{ '
            f'translation {x} {y} {z} '
            f'rotation {rx} {ry} {rz} {ra} '
            f'name "{name}" '
            f'}}'
        )
        self.root_children.importMFNodeFromString(-1, brick_string)

    def spawn_many(self, positions, name_prefix="brick", physics=True):
        for i, (x, y, z) in enumerate(positions):
            self.spawn_brick(f"{name_prefix}_{i:03d}", x, y, z, physics=physics)
        print(f"Spawned {len(positions)} bricks (physics={physics}).")