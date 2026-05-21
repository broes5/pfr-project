import sys, os, math
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from shared.vector3 import Vector3

PILE_POSITION       = Vector3(-4.0, 0.0, 0.0)
PILE_COLS           = 5
PILE_ROWS           = 5
PILE_SPACING_XY     = 0.6    # must exceed voxel_size (0.5) so each pile pos gets its own voxel
PILE_BRICK_HEIGHT   = 0.15
PICKUP_HOVER_OFFSET = 0.15   # m above brick bottom — puts drone right at brick-top level

_layer_size = PILE_COLS * PILE_ROWS
_n_layers   = 1   # overwritten by configure()


def configure(n_bricks):
    """Compute the number of pile layers needed to hold n_bricks. Must be called
    by both brick_manager and controller_device after loading the instruction file."""
    global _n_layers
    _n_layers = math.ceil(n_bricks / _layer_size)


def pile_pos_for_brick(brick_id):
    """Return the bottom-of-brick Vector3 for brick_id in the pile grid.

    Layer 0 is the topmost physical layer (highest z), matching top-to-bottom
    pickup order. configure() must be called first so _n_layers is correct.
    """
    if brick_id < 0:
        return None
    li  = brick_id // _layer_size          # 0 = top layer
    idx = brick_id %  _layer_size
    row = idx // PILE_COLS
    col = idx %  PILE_COLS
    return Vector3(
        PILE_POSITION.x + (col - (PILE_COLS  - 1) / 2.0) * PILE_SPACING_XY,
        PILE_POSITION.y + (row - (PILE_ROWS  - 1) / 2.0) * PILE_SPACING_XY,
        (_n_layers - 1 - li) * PILE_BRICK_HEIGHT,   # bottom of brick
    )


def pile_layer_size():
    return _layer_size


def pile_layers_for_count(n):
    return math.ceil(n / _layer_size)


def pile_layer_for_brick(brick_id):
    return brick_id // _layer_size
