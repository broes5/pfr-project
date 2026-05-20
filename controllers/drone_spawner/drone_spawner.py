from controller import Supervisor

N_DRONES = 3
SPACING = 1.2
START_X = -3.0
START_Z = 0.065

supervisor    = Supervisor()
timestep      = int(supervisor.getBasicTimeStep())
root_children = supervisor.getRoot().getField("children")

for i in range(N_DRONES):
    x = (i - (N_DRONES - 1) / 2.0) * SPACING
    node_str = (
        f'BrickDrone {{ '
        f'translation {START_X:.4f} {x:.4f} {START_Z:.4f} '
        f'name "BrickDrone_{i}" '
        f'}}'
    )
    root_children.importMFNodeFromString(-1, node_str)

print(f"[drone_spawner] Spawned {N_DRONES} BrickDrones in a line (spacing={SPACING}m).")

while supervisor.step(timestep) != -1:
    pass
