from controller import Robot, Keyboard

print("Starting up drone controller...")

def clamp(value, low, high):
    return max(low, min(value, high))

robot = Robot()

timestep = int(robot.getBasicTimeStep())
dt = timestep / 1000.0

fl = robot.getDevice('front left propeller')
fr = robot.getDevice('front right propeller')
rl = robot.getDevice('rear left propeller')
rr = robot.getDevice('rear right propeller')

for m in [fl, fr, rl, rr]:
    m.setPosition(float('inf'))
    m.setVelocity(1.0)

imu = robot.getDevice('inertial unit'); imu.enable(timestep)
gps = robot.getDevice('gps');
gps.enable(timestep)
gyro = robot.getDevice('gyro');
gyro.enable(timestep)
kb = robot.getKeyboard();
kb.enable(timestep)

#camera_roll_motor = robot.getDevice('camera roll')
#camera_pitch_motor = robot.getDevice('camera pitch')

K_VERTICAL_THRUST = 68.5
K_VERTICAL_OFFSET = 0.6
K_VERTICAL_P = 3.0
K_ROLL_P = 50.0
K_PITCH_P = 30.0
TARGET_ALT = 6.0 # metres — auto-takeoff
ALT_REACHED = 0.1 # metres — tolerance for "reached target"

LAND_SEQUENCE = [
(4.0, 2.0),
(2.0, 2.0),
(0.5, 2.0),
(0.0, 1.0), # ground — cut motors
]

# ── State machine constants
IDLE = 'IDLE'
TAKEOFF = 'TAKEOFF'
FLY = 'FLY'
LAND = 'LAND'
DONE = 'DONE'

# ── State variables
state = IDLE
target_altitude = 0.0
land_stage = 0
land_stage_timer = 0.0

# ── Wait 1 second for physics to settle
while robot.step(timestep) != -1:
    if robot.getTime() > 1.0:
        break
print('t=Takeoff l=Auto-land Arrows=fly Shift+Arrows=altitude/strafe')

while robot.step(timestep) != -1:
    t = robot.getTime()
    altitude = gps.getValues()[2]
    rpy = imu.getRollPitchYaw()
    roll, pitch = rpy[0], rpy[1]
    gv = gyro.getValues()
    roll_vel, pitch_vel = gv[0], gv[1]

    # ── Keyboard input
    roll_d = pitch_d = yaw_d = 0.0 # reset deltas every step
    key = kb.getKey()
    while key > 0:
        if key == Keyboard.UP or key == ord('W'):
            pitch_d = -2.0
        elif key == Keyboard.DOWN or key == ord('S'):
            pitch_d = 2.0
        elif key == Keyboard.RIGHT or key == ord('D'):
            yaw_d = -1.3
        elif key == Keyboard.LEFT or key == ord('A'):
            yaw_d = 1.3
        elif key == Keyboard.SHIFT + Keyboard.RIGHT:
            roll_d = -1.0
        elif key == Keyboard.SHIFT + Keyboard.LEFT:
            roll_d = 1.0
        elif key == Keyboard.SHIFT + Keyboard.UP:
            target_altitude += 0.05
            print(f'alt target: {target_altitude:.2f} m')
        elif key == Keyboard.SHIFT + Keyboard.DOWN:
            target_altitude -= 0.05
            print(f'alt target: {target_altitude:.2f} m')
        elif key == ord('T') and state == IDLE:
            target_altitude = TARGET_ALT
            state = TAKEOFF
            print(f'>> TAKEOFF to {TARGET_ALT} m')
        elif key == ord('L') and state == FLY:
            state = LAND
            land_stage = 0
            land_stage_timer = t
            print('>> AUTO LAND')
        key = kb.getKey() # drain remaining keys in buffer

    # ── State transition — TAKEOFF → FLY
    if state == TAKEOFF and abs(altitude - TARGET_ALT) < ALT_REACHED:
        state = FLY
        print(f'>> FLY — reached {altitude:.2f} m')

    # ── DONE — cut all motors and skip the rest of the loop
    if state == DONE:
        for m in [fl, fr, rl, rr]:
            m.setVelocity(0.0)
            #camera_roll_motor.setPosition(0.0)
            #camera_pitch_motor.setPosition(0.0)
        continue # skip PID and mixer this step

    # ── LAND — staged descent
    if state == LAND:
        if land_stage < len(LAND_SEQUENCE):
            tgt, hold = LAND_SEQUENCE[land_stage]
            target_altitude = tgt
            # Advance only when altitude is reached AND hold time has elapsed
            if abs(altitude - tgt) < ALT_REACHED or (t - land_stage_timer) > hold:
                print(f'>> Land stage {land_stage + 1} done')
                land_stage += 1
                land_stage_timer = t # reset timer for the next stage
        else:
            state = IDLE # The drone has landed.

    # ── PID computation
    clamped_diff = clamp(target_altitude - altitude + K_VERTICAL_OFFSET, -1.0, 1.0)
    vertical_input = K_VERTICAL_P * (clamped_diff ** 3)
    roll_input = K_ROLL_P * clamp(roll, -1.0, 1.0) + roll_vel + roll_d
    pitch_input = K_PITCH_P * clamp(pitch, -1.0, 1.0) + pitch_vel + pitch_d
    yaw_input = yaw_d

    # ── Motor mixer
    fl_v = K_VERTICAL_THRUST + vertical_input - roll_input + pitch_input - yaw_input
    fr_v = K_VERTICAL_THRUST + vertical_input + roll_input + pitch_input + yaw_input
    rl_v = K_VERTICAL_THRUST + vertical_input - roll_input - pitch_input + yaw_input
    rr_v = K_VERTICAL_THRUST + vertical_input + roll_input - pitch_input - yaw_input

    fl.setVelocity( fl_v) # clockwise — positive
    fr.setVelocity(-fr_v) # counter-clockwise — negated
    rl.setVelocity(-rl_v) # counter-clockwise — negated
    rr.setVelocity( rr_v) # clockwise — positive

    # ── Camera gimbal
    #camera_roll_motor.setPosition( -0.115 * roll_vel)
    #camera_pitch_motor.setPosition(-0.1 * pitch_vel)

