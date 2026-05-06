from controller import Supervisor
import random

supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

root_children = supervisor.getRoot().getField("children")

def spawn_brick(name, x, y, z=0.0756, rotation=(0, 0, 1, 0)):
    rx, ry, rz, ra = rotation
    brick_string = (
        f'Brick {{ '
        f'translation {x} {y} {z} '
        f'rotation {rx} {ry} {rz} {ra} '
        f'name "{name}" '
        f'}}'
    )

    root_children.importMFNodeFromString(-1, brick_string)

N = 100
for i in range(N):
    spawn_brick(f"brick_{i:02d}", x=0, y=0, z=i)

while supervisor.step(timestep) != -1:
    pass