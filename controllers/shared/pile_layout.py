import math

from .vector3 import Vector3

PILE_POSITION       = Vector3(-4.0, 0.0, 0.0)
PILE_COLS           = 5
PILE_ROWS           = 5
PILE_SPACING_XY     = 0.6    # must exceed voxel_size (0.5) so each pile pos gets its own voxel
PILE_BRICK_HEIGHT   = 0.15
PICKUP_HOVER_OFFSET = 0.15   # m above brick bottom — puts drone right at brick-top level

_layer_size = PILE_COLS * PILE_ROWS
_n_layers   = 1   # overwritten by configure()
_n_bricks   = 0   # overwritten by configure()


def configure(n_bricks):
    """Compute the number of pile layers needed to hold n_bricks. Must be called
    by both brick_manager and controller_device after loading the instruction file."""
    global _n_layers, _n_bricks
    _n_bricks = n_bricks
    _n_layers = math.ceil(n_bricks / _layer_size)


def _partial_top():
    """Bricks in the top layer when N isn't a multiple of _layer_size. 0 = all layers full."""
    return _n_bricks % _layer_size


def pile_layer_for_brick(brick_id):
    """Return the layer index (0 = topmost physical layer) for this brick.

    When the total count isn't a multiple of _layer_size, the top layer is
    partial and holds brick_ids 0..(partial-1). Full layers sit below it so
    every top-layer brick is physically supported.
    """
    partial = _partial_top()
    if partial > 0 and brick_id < partial:
        return 0  # partial top layer
    effective = brick_id - partial if partial > 0 else brick_id
    base = effective // _layer_size
    return base + (1 if partial > 0 else 0)


def pile_pos_for_brick(brick_id):
    """Return the bottom-of-brick Vector3 for brick_id in the pile grid.

    Layer 0 is the topmost physical layer (highest z). When N is not a
    multiple of _layer_size the top layer is partial, so every top-layer
    brick has a supporting brick directly below it in the full layer beneath.
    """
    if brick_id < 0:
        return None
    partial = _partial_top()
    li = pile_layer_for_brick(brick_id)
    if partial > 0 and brick_id < partial:
        idx = brick_id  # position 0..partial-1 within the partial top layer
    elif partial > 0:
        idx = (brick_id - partial) % _layer_size
    else:
        idx = brick_id % _layer_size
    row = idx // PILE_COLS
    col = idx %  PILE_COLS
    return Vector3(
        PILE_POSITION.x + (col - (PILE_COLS - 1) / 2.0) * PILE_SPACING_XY,
        PILE_POSITION.y + (row - (PILE_ROWS - 1) / 2.0) * PILE_SPACING_XY,
        (_n_layers - 1 - li) * PILE_BRICK_HEIGHT,  # layer 0 at top, bottom layer at z=0
    )


def pile_layer_size():
    return _layer_size


def pile_layer_count_for_layer(li):
    """Actual number of bricks in layer li (may be less than _layer_size for the top layer)."""
    partial = _partial_top()
    if partial > 0 and li == 0:
        return partial
    return _layer_size


def pile_layers_for_count(n):
    return math.ceil(n / _layer_size)
