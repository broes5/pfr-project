SCALE = 0.02
LAYER_HEIGHT = 0

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


def to_world_coords(bricks, scale=SCALE, layer_height=LAYER_HEIGHT):
    return [
        (x * scale, y * scale, z * scale + layer * layer_height)
        for (x, y, z, layer) in bricks
    ]