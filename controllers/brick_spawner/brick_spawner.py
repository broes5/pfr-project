from controller import Supervisor

SCALE = 0.02
LAYER_HEIGHT = 0
BRICK_FILE = "instructions.txt"

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


def parse_brick_file(path):
    bricks = []
    layer = 0

    with open(path, "r") as f:
        for raw_line in f:
            line = raw_line.strip()

            if not line or line.startswith("#"):
                continue

            if line == "new_layer":
                layer += 1
                continue

            if line.startswith("brick:"):
                coords_str = line.split(":", 1)[1].strip().strip("[]")
                parts = [p.strip() for p in coords_str.split(",")]
                if len(parts) != 3:
                    print(f"Skipping malformed line: {raw_line!r}")
                    continue
                x, y, z = (float(p) for p in parts)
                bricks.append((x, y, z, layer))
                continue

            print(f"Unknown line, skipping: {raw_line!r}")

    return bricks


def spawn_bricks_from_file(path, scale=SCALE, layer_height=LAYER_HEIGHT):
    bricks = parse_brick_file(path)

    for i, (x, y, z, layer) in enumerate(bricks):
        wx = x * scale
        wy = y * scale
        wz = z * scale + layer * layer_height

        spawn_brick(f"brick_{i:03d}", wx, wy, wz)

    print(f"Spawned {len(bricks)} bricks from {path}.")


spawn_bricks_from_file(BRICK_FILE)

##This needs to exist otherwise Webots will get very upset
while supervisor.step(timestep) != -1:
    pass