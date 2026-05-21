import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from shared.vector3 import Vector3

PILE_POSITION       = Vector3(-4.0, 0.0, 0.0)
PILE_COLS           = 3
PILE_ROWS           = 3
PILE_LAYERS         = 3
PILE_SPACING_XY     = 0.6   # must exceed voxel_size (0.5) so each pile pos gets its own voxel
PILE_BRICK_HEIGHT   = 0.15
PICKUP_HOVER_OFFSET = 0.20   # m above brick bottom to hover before pickup


def pile_pos_for_brick(brick_id):
    """Return the bottom-of-brick Vector3 for brick_id in the pile grid.

    Layer 0 is the topmost physical layer (highest z), matching top-to-bottom
    pickup order. Returns None if brick_id is out of range.
    """
    layer_size = PILE_COLS * PILE_ROWS
    total = layer_size * PILE_LAYERS
    if brick_id < 0 or brick_id >= total:
        return None
    li  = brick_id // layer_size          # 0 = top layer
    idx = brick_id %  layer_size
    row = idx // PILE_COLS
    col = idx %  PILE_COLS
    return Vector3(
        PILE_POSITION.x + (col - (PILE_COLS  - 1) / 2.0) * PILE_SPACING_XY,
        PILE_POSITION.y + (row - (PILE_ROWS  - 1) / 2.0) * PILE_SPACING_XY,
        (PILE_LAYERS - 1 - li) * PILE_BRICK_HEIGHT,   # bottom of brick
    )


def pile_layer_size():
    return PILE_COLS * PILE_ROWS


def pile_layer_for_brick(brick_id):
    return brick_id // pile_layer_size()
