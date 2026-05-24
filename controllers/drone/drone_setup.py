import sys, os

import math
from controller import Robot, Keyboard

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from shared.vector3 import Vector3
from shared.path import Path
from shared.config import (
    TASK_ALT, TAKEOFF_ALT, PICKUP_ALT, PLACE_ALT_OFFSET,
    TASK_ARRIVAL_THRESHOLD, WAYPOINT_THRESHOLD, OBSTACLE_AVOIDANCE,
)

def clamp(value, low, high):
    return max(low, min(value, high))

robot = Robot()
timestep = int(robot.getBasicTimeStep())
dt = timestep / 1000.0
drone_name = robot.getName()

# ── State machine constants
IDLE, TAKEOFF, FLY, RETURN, LAND, DONE = 'IDLE', 'TAKEOFF', 'FLY', 'RETURN', 'LAND', 'DONE'

state = IDLE
land_stage = 0
land_stage_timer = 0.0
print_counter = 0  # To prevent console flooding

# target position
# Format: vector3(x, y, z) where z = Altitude
targetPos = Vector3(0, 0, 0)

# target rotation in degrees
targetYaw = 0.0

# Path following state
current_path: Path = None
path_index: int = 0
path_queue = []   # buffered (Path, yaw_degrees | None) pairs waiting to execute

# Task queue state (TASK PICKUP / TASK PLACE assignments from controller_device)
# Each task: ('PICKUP', brick_id, flight_target: Vector3)
#         or ('PLACE',  brick_id, flight_target: Vector3, place_x, place_y, place_z, place_rot)
task_queue = []
current_task = None

# ── Motors
fl = robot.getDevice('front left propeller')
fr = robot.getDevice('front right propeller')
rl = robot.getDevice('rear left propeller')
rr = robot.getDevice('rear right propeller')

MOTOR_SPEED_LIMIT = 576.0

for m in [fl, fr, rl, rr]:
    m.setPosition(float('inf'))
    m.setVelocity(0.0)

# ── Sensors
imu = robot.getDevice('inertial unit'); imu.enable(timestep)
gyro = robot.getDevice('gyro'); gyro.enable(timestep)
gps = robot.getDevice('gps'); gps.enable(timestep)
kb = robot.getKeyboard(); kb.enable(timestep)
# controller communication
receiver = robot.getDevice('receiver')
if receiver:
    receiver.enable(timestep)
emitter = robot.getDevice('emitter')
# distance sensors for obstacle avoidance
sensor_names = {
    'front': 'ds_front',
    'back': 'ds_back',
    'left': 'ds_left',
    'right': 'ds_right'
    } 
distance_sensors = {}
for direction, name in sensor_names.items():
    ds = robot.getDevice(name)
    if ds:
        ds.enable(timestep)
        distance_sensors[direction] = ds

# ── TUNED CONSTANTS ──────────────────
# Gravity compensation: empirically matched to the drone's simulated mass so that
# K_VERTICAL_THRUST alone holds steady altitude with zero vertical error.
K_VERTICAL_THRUST = 68.5
K_VERTICAL_OFFSET = 0.6  # residual hover correction after PID settles

# ── Obstacle avoidance parameters ────
AVOID_THRESHOLD = 1.5  # metres — distance at which an obstacle starts influencing flight
AVOID_WEIGHT = 2.0     # repulsion magnitude at the threshold boundary
MIN_DIST = 1.0         # at this distance repulsion is at full saturation
SLIDE_WEIGHT = AVOID_WEIGHT * 0.9  # tangential slide force (slightly weaker than direct repulsion)
MAX_PULL = 2.0  # caps target attraction so nearby goals don't override obstacle avoidance

# Persistent direction latches: once a bypass direction is chosen it is held until the
# obstacle clears. This prevents oscillation where the drone flip-flops between left and
# right because both sides read the same distance.
bypass_dir_y = 0.0  # 1.0 = Left, -1.0 = Right, 0.0 = Undecided/Clear
bypass_dir_x = 0.0  # 1.0 = Forward, -1.0 = Backward, 0.0 = Undecided/Clear

# ── Altitude PID ─────────────────────
# High P/D relative to I: altitude must track quickly to changes in target Z during
# the three-leg path descents, but integral must not fight gravity compensation.
K_VERTICAL_P = 2.0
K_VERTICAL_D = 6.0
K_VERTICAL_I = 0.5

# ── Attitude (tilt) PID ──────────────
# High P because the drone can only translate by tilting; faster tilt response =
# tighter position tracking. Gyro derivative (K_GYRO_D) damps oscillation from the
# stiff attitude springs.
K_ROLL_P = 15.0
K_PITCH_P = 15.0
K_GYRO_D = 3.0
# Partial compensation: full correction would require knowing the carried brick mass;
# 0.6 is the fraction applied to avoid over-thrusting on extreme tilts.
K_COMP_AGGRESSION = 0.6

# ── Horizontal position PID ──────────
# Low P/I to prevent the drone from chasing its target too aggressively and
# oscillating over it. D damps GPS-velocity noise.
K_POS_P = 0.4
K_POS_D = 0.8
K_POS_I = 0.3

# ── Yaw PID ──────────────────────────
K_YAW_P = 2.0

V_FILTER = 0.1
ALT_REACHED = 0.1 # Bracket  of error allowance for target altitude being reached

# ── Memory Variables
prevPos = Vector3(0.0, 0.0, 0.0)

f_xVel, f_yVel, f_zVel = 0.0, 0.0, 0.0 # filtered velocities
i_ex, i_ey, i_ez = 0.0, 0.0, 0.0 # Integral accumulators

# Avoidance Filtering Memory:
f_rx, f_ry = 0.0, 0.0
f_sx, f_sy = 0.0, 0.0
f_target_scale_x, f_target_scale_y = 1.0, 1.0
ATTACK = 0.2 # application rate of evasion forces
RELEASE = 0.006 # Rate evasion force tapers off

# ── Auto-landing sequence: (target_altitude_m,hold_seconds)
LAND_SEQUENCE = [
    (3.0, 3.0),
    (2.0, 3.0),
    (0.5, 3.0),
    (0.0, 2.0), # ground — cut motors
]

def filter_force(curr, tgt):
            # If force is increasing or changing direction completely -> Attack
            if abs(tgt) > abs(curr) or (curr > 0 and tgt < 0) or (curr < 0 and tgt > 0):
                return curr + (tgt - curr) * ATTACK
            else:
                # If force is decaying back to 0 -> Slow Release
                return curr + (tgt - curr) * RELEASE

# ── Wait for physics to settle
home_pos = Vector3(0.0, 0.0, 0.0)  # recorded once GPS stabilises — used as landing target

while robot.step(timestep) != -1:
    if robot.getTime() > 1.0:
        vals = gps.getValues()

        prevPos = Vector3(vals[0], vals[1], vals[2])
        home_pos = Vector3(vals[0], vals[1], vals[2])  # save spawn XY for return-to-home

        # Initialize targetYaw to current orientation
        _, _, start_yaw = imu.getRollPitchYaw()
        targetYaw = math.degrees(start_yaw) % 360.0
        print(f'Physics settled. Initial GPS: {prevPos.x:.2f}, {prevPos.y:.2f}\n\n')
        break

