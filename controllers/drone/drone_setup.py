import sys, os

import math
from controller import Robot, Keyboard

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from shared.vector3 import Vector3
from shared.path import Path

def clamp(value, low, high):
    return max(low, min(value, high))

robot = Robot()
timestep = int(robot.getBasicTimeStep())
dt = timestep / 1000.0
drone_name = robot.getName()

# ── State machine constants
IDLE, TAKEOFF, FLY, LAND, DONE = 'IDLE', 'TAKEOFF', 'FLY', 'LAND', 'DONE'

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
WAYPOINT_THRESHOLD = 0.5

# Task queue state (TASK PICKUP / TASK PLACE assignments from controller_device)
# Each task: ('PICKUP', brick_id, flight_target: Vector3)
#         or ('PLACE',  brick_id, flight_target: Vector3, place_x, place_y, place_z, place_rot)
task_queue = []
current_task = None
TASK_ARRIVAL_THRESHOLD = 0.3  # m — how close to the task position before emitting the signal
TASK_ALT = 3.0                # cruise altitude (m) used when transiting between pile and target
PICKUP_ALT = 0.5              # altitude (m) to descend to over the pile before emitting PICKUP
PLACE_ALT_OFFSET = 0.5        # m above the brick's target z to descend to before emitting PLACE

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
K_VERTICAL_THRUST = 68.5 # thrust required to counteract gravity (from drones own mass, will not compensate if mass changes due to lifting an object)
K_VERTICAL_OFFSET = 0.6 # offset to hover on target altitude due to the drones mass

AVOID_THRESHOLD = 1.5 # Distance in metres to start reacting to obstacles
AVOID_WEIGHT = 2.0 # How aggressive the drone avoids obstacles
MIN_DIST = 1.0 # distance where suppression due to obstacle avoidance is at its maximum
SLIDE_WEIGHT = AVOID_WEIGHT * 0.9
MAX_PULL = 2.0 # Limit target attraction so distant target position's don't overpower obstacle avoidance
# ── Persistent Avoidance Latches (Remembers which way it chose to go around an object)
bypass_dir_y = 0.0  # 1.0 = Left, -1.0 = Right, 0.0 = Undecided/Clear
bypass_dir_x = 0.0  # 1.0 = Forward, -1.0 = Backward, 0.0 = Undecided/Clear

# Altitude Constants
K_VERTICAL_P = 1.8 # rate of aggression towards reaching target altitude
K_VERTICAL_D = 1.5 # vertical velocity damper (reduces overshoot of target altitude)
K_VERTICAL_I = 0.5 # prevents steady state error in altitude

# Attitude (Tilt) Constants
K_ROLL_P = 15.0 # rate of aggression towards reaching roll angle
K_PITCH_P = 15.0 # rate of aggression towards reaching pitch angle
K_GYRO_D = 3.0 # Tilt damper, uses gyro to stop drone from rotating or swinging around like a pendulum
K_COMP_AGGRESSION = 0.6 # rate of altitude loss compensation whil drone is tilted

# Position (GPS) Constants
K_POS_P = 0.3 # Rate of attraction to coordinates on the horizontal plane
K_POS_D = 0.6 # Horizontal velocity damper (reduces overshoot of target position)
K_POS_I = 0.3 # Prevents steady state error from attraction to correct coordinates not being high enough to move the drone

# Rotation Constants
K_YAW_P = 2.0

V_FILTER = 0.1
TAKEOFF_ALT = 3.0 # default altitude to take off to
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
while robot.step(timestep) != -1:
    if robot.getTime() > 1.0:
        vals = gps.getValues()
        
        prevPos = Vector3(vals[0], vals[1], vals[2])
        
        # Initialize targetYaw to current orientation
        _, _, start_yaw = imu.getRollPitchYaw()
        targetYaw = math.degrees(start_yaw) % 360.0
        print(f'Physics settled. Initial GPS: {prevPos.x:.2f}, {prevPos.y:.2f}\n\n')
        break

#print("──────────────────────────────────────────────")
#print("                 CONTROLS                     ")
#print("──────────────────────────────────────────────")

#print("\nT = Takeoff")

#print("──────────────   While Flying   ──────────────")
#print("L = Land")
#print("WASD = Move Target")
#print("Arrow keys = Change Target Altitude/Yaw")
