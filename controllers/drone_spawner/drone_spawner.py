import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from controller import Supervisor
from shared.config import N_DRONES, DRONE_SPACING, DRONE_START_X, DRONE_START_Z

supervisor    = Supervisor()
timestep      = int(supervisor.getBasicTimeStep())
root_children = supervisor.getRoot().getField("children")

for i in range(N_DRONES):
    y = (i - (N_DRONES - 1) / 2.0) * DRONE_SPACING
    node_str = (
        f'BrickDrone {{ '
        f'translation {DRONE_START_X:.4f} {y:.4f} {DRONE_START_Z:.4f} '
        f'name "BrickDrone_{i}" '
        f'}}'
    )
    root_children.importMFNodeFromString(-1, node_str)

print(f"[drone_spawner] Spawned {N_DRONES} BrickDrones in a line (spacing={DRONE_SPACING}m).")

while supervisor.step(timestep) != -1:
    pass
