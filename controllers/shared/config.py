# ── Fleet ────────────────────────────────────────────────────────────────────
N_DRONES   = 4            # number of drones to spawn and coordinate
BRICK_FILE = "../instructions/uni1.txt"  # target structure definition (242 bricks, 6 layers)

# ── Flight altitudes (shared between controller_device and drone) ─────────────
TASK_ALT         = 3.0   # cruise altitude during transit — high enough to clear the pile and structure
TAKEOFF_ALT      = 3.0   # target altitude after takeoff
PICKUP_ALT       = 0.17  # hover floor at pile: just above a brick so the drone can grab it
PLACE_ALT_OFFSET = 0.17  # approach this far above the target brick z before signalling PLACE

# ── Drone task thresholds ─────────────────────────────────────────────────────
# Loose waypoint threshold (0.5 m) keeps drones moving without requiring pixel-perfect alignment.
# Tight arrival threshold (0.2 m) ensures the PICKUP/PLACE signal fires close to the actual brick.
TASK_ARRIVAL_THRESHOLD = 0.20  # m — 3D distance to task pos before emitting PICKUP/PLACE signal
WAYPOINT_THRESHOLD     = 0.40 # m — advance to next waypoint when within this distance

# ── Feature flags ─────────────────────────────────────────────────────────────
OBSTACLE_AVOIDANCE = False  # distance-sensor-based inter-drone collision avoidance

# ── Starting formation ────────────────────────────────────────────────────────
DRONE_SPACING = 2.5   # m between drones in the initial line formation
DRONE_START_X = -2.0   # starting X (forward of origin)
DRONE_START_Z = 0.065 # ground level clearance at spawn
