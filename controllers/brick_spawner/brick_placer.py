import sys, os, math
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'shared'))

from vector3 import Vector3

POOL_POSITION = Vector3(0, 0, -10)
BRICK_HALF_HEIGHT = 0.075

class BrickPool:
    def __init__(self, supervisor):
        self.supervisor = supervisor
        self.root_children = supervisor.getRoot().getField("children")
        self._pool = []
        self._active = {}
        self._holder = None

    def _ensure_holder(self):
        if self._holder is not None:
            return self._holder
        existing = self.supervisor.getFromDef("Bricks")
        if existing is None:
            self.root_children.importMFNodeFromString(
                -1, 'DEF Bricks Pose { translation 0 0 0 children [] }'
            )
            existing = self.supervisor.getFromDef("Bricks")
        self._holder = existing.getField("children")
        return self._holder

    def pre_spawn(self, count, physics=False):
        holder = self._ensure_holder()
        proto = "BrickPhy" if physics else "BrickStill"
        p = POOL_POSITION
        for _ in range(count):
            node_str = (
                f'{proto} {{ '
                f'translation {p.x} {p.y} {p.z} '
                f'rotation 0 0 1 0 '
                f'}}'
            )
            holder.importMFNodeFromString(-1, node_str)
            self._pool.append(holder.getMFNode(holder.getCount() - 1))
        print(f"Pre-spawned {count} bricks into pool (physics={physics}).")

    def spawn(self, brick_id, position: Vector3, theta: float):
        if not self._pool:
            raise RuntimeError("Brick pool exhausted — call pre_spawn with a larger count")
        node = self._pool.pop()
        rad = theta * math.pi / 180
        node.getField("translation").setSFVec3f([position.x, position.y, position.z + BRICK_HALF_HEIGHT])
        node.getField("rotation").setSFRotation([0, 0, 1, rad])
        self._active[brick_id] = node

    def despawn(self, brick_id):
        node = self._active.pop(brick_id, None)
        if node is None:
            return
        node.getField("translation").setSFVec3f(POOL_POSITION.to_list())
        self._pool.append(node)

    def get(self, brick_id):
        return self._active.get(brick_id)

    @staticmethod
    def brick_name(brick_id: int) -> str:
        return f"brick_{brick_id:03d}"
