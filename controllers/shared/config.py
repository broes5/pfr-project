# ── Fleet ────────────────────────────────────────────────────────────────────
N_DRONES   = 4
BRICK_FILE = "../instructions/uni1.txt"

# ── Flight altitudes (shared between controller_device and drone) ─────────────
TASK_ALT         = 3.0   # cruise altitude during transit
TAKEOFF_ALT      = 3.0   # target altitude after takeoff
PICKUP_ALT       = 0.15  # minimum placement altitude / pickup hover floor
PLACE_ALT_OFFSET = 0.15  # m above target brick z to approach before placing

# ── Drone task thresholds ─────────────────────────────────────────────────────
TASK_ARRIVAL_THRESHOLD = 0.20  # m — 3D distance to task pos before emitting signal
WAYPOINT_THRESHOLD     = 0.50  # m — advance to next waypoint when within this distance

# ── Feature flags ─────────────────────────────────────────────────────────────
OBSTACLE_AVOIDANCE = False

# ── Starting formation ────────────────────────────────────────────────────────
DRONE_SPACING = 1.5
DRONE_START_X = 2.0
DRONE_START_Z = 0.065
