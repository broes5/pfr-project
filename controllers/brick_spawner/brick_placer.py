import math

class BrickPlacer:
    def __init__(self, supervisor):
        self.supervisor = supervisor
        self.root_children = supervisor.getRoot().getField("children")
        self._holders = {}

    def _get_or_create_holder(self, holder_name):
        if holder_name in self._holders:
            return self._holders[holder_name]

        existing = self.supervisor.getFromDef(holder_name)
        if existing is None:
            holder_string = (
                f'DEF {holder_name} Pose {{ '
                f'translation 0 0 0 '
                f'children [] '
                f'}}'
            )
            self.root_children.importMFNodeFromString(-1, holder_string)
            existing = self.supervisor.getFromDef(holder_name)

        children_field = existing.getField("children")
        self._holders[holder_name] = children_field
        return children_field

    def spawn_brick(self, name, x, y, z, theta, physics=True, holder="Bricks"):
        proto_name = "BrickPhy" if physics else "BrickStill"
        z += 0.15 / 2
        rad = theta * math.pi / 180

        brick_string = (
            f'{proto_name} {{ '
            f'translation {x} {y} {z} '
            f'rotation 0 0 1 {rad} '
            f'name "{name}" '
            f'}}'
        )

        children_field = self._get_or_create_holder(holder)
        children_field.importMFNodeFromString(-1, brick_string)

    def spawn_many(self, bricks, name_prefix="brick", physics=True, holder="Bricks"):
        for i, (x, y, z, theta) in enumerate(bricks):
            self.spawn_brick(
                f"{name_prefix}_{i:03d}", x, y, z, theta,
                physics=physics, holder=holder,
            )
        print(f"Spawned {len(bricks)} bricks into '{holder}' (physics={physics}).")