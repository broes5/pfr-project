import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'shared'))
import math
from controller import Robot, Keyboard
from vector3 import Vector3

robot = Robot()
timestep = int(robot.getBasicTimeStep())
dt = timestep / 1000.0

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

# ── Motors
fl = robot.getDevice('front left propeller')
fr = robot.getDevice('front right propeller')
rl = robot.getDevice('rear left propeller')
rr = robot.getDevice('rear right propeller')

for m in [fl, fr, rl, rr]:
    m.setPosition(float('inf'))
    m.setVelocity(0.0)

# ── Sensors
imu = robot.getDevice('inertial unit'); imu.enable(timestep)
gyro = robot.getDevice('gyro'); gyro.enable(timestep)
gps = robot.getDevice('gps'); gps.enable(timestep)
kb = robot.getKeyboard(); kb.enable(timestep)

# ── TUNED CONSTANTS ──────────────────
K_VERTICAL_THRUST = 68.5 # thrust required to counteract gravity (from drones own mass, will not compensate if mass changes due to lifting an object)
K_VERTICAL_OFFSET = 0.6 # offset to hover on target altitude due to the drones mass

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

# ── Auto-landing sequence: (target_altitude_m,hold_seconds)
LAND_SEQUENCE = [
    (3.0, 3.0),
    (2.0, 3.0),
    (0.5, 3.0),
    (0.0, 2.0), # ground — cut motors
]

def clamp(value, low, high):
    return max(low, min(value, high))

# ── Wait for physics to settle
while robot.step(timestep) != -1:
    if robot.getTime() > 1.0:
        vals = gps.getValues()
        
        # old positioning
        # prevX, prevY = vals[0], vals[1]
        prevPos = Vector3(vals[0], vals[1], vals[2])
        
        # Initialize targetYaw to current orientation
        _, _, start_yaw = imu.getRollPitchYaw()
        targetYaw = math.degrees(start_yaw) % 360.0
        print(f'Physics settled. Initial GPS: {prevPos.x:.2f}, {prevPos.y:.2f}')
        break

print('\n──────────────────────────────────────────────────\n                  CONTROLS\n──────────────────────────────────────────────────\nT = Takeoff\n────────────────   While Flying   ────────────────\nL = Land\nWASD = Move Target\nArrow keys = Change Target Altitude/Yaw')

# ──────────────────────────────────────────────────
#                Main Loop
# ──────────────────────────────────────────────────

while robot.step(timestep) != -1:
    t = robot.getTime()
    
    # 1. Sensors
    gps_vals = gps.getValues()
    currentPos = Vector3(gps_vals[0], gps_vals[1], gps_vals[2])
    
    ### OLD POSITIONING CODE ##
    # xPos, yPos, gpsAlt = gps_vals[0], gps_vals[1], gps_vals[2]
    roll, pitch, yaw = imu.getRollPitchYaw()
    roll_vel, pitch_vel, yaw_vel = gyro.getValues()
    
    # 2. Filtered Velocity Calculation
    raw_xVel = (currentPos.x - prevPos.x) / dt
    raw_yVel = (currentPos.y - prevPos.y) / dt
    raw_zVel = (currentPos.z - prevPos.z) / dt
    
    f_xVel = (f_xVel * (1 - V_FILTER)) + (raw_xVel * V_FILTER)
    f_yVel = (f_yVel * (1 - V_FILTER)) + (raw_yVel * V_FILTER)
    f_zVel = (f_zVel * (1 - V_FILTER)) + (raw_zVel * V_FILTER)
    
    prevPos = currentPos
    roll_d = pitch_d = yaw_d = 0.0
    
    # 3. Keyboard Input
    key = kb.getKey()
    while key > 0:
        if key == ord('W') and state == FLY: 
            targetPos.x += 0.1
            print(f'[Input] Target X moved to: {targetPos.x:.2f}')
        elif key == ord('S') and state == FLY: 
            targetPos.x -= 0.1
            print(f'[Input] Target X moved to: {targetPos.x:.2f}')
        elif key == ord('D') and state == FLY: 
            targetPos.y -= 0.1
            print(f'[Input] Target Y moved to: {targetPos.y:.2f}')
        elif key == ord('A') and state == FLY: 
            targetPos.y += 0.1
            print(f'[Input] Target Y moved to: {targetPos.y:.2f}')
            
        # Yaw Control (Changes target angle)
        elif key == Keyboard.RIGHT and state == FLY: 
            targetYaw = (targetYaw - 0.1) % 360.0
        elif key == Keyboard.LEFT and state == FLY: 
            targetYaw = (targetYaw + 0.1) % 360.0

        # Altitude Control (changes target altitude)
        elif key == Keyboard.UP and state == FLY: 
            targetPos.z += 0.1
            print(f'[Input] Target Alt: {targetPos.z:.2f}')
        elif key == Keyboard.DOWN and state == FLY: 
            targetPos.z -= 0.1
            print(f'[Input] Target Alt: {targetPos.z:.2f}')
        
        # special keys (Takeoff and landing)
        elif key == ord('T') and state == IDLE:
            targetPos.x = currentPos.x
            targetPos.y = currentPos.y
            targetPos.z = TAKEOFF_ALT
            state = TAKEOFF
            print(f'>> TAKEOFF sequence initiated to {TAKEOFF_ALT}m')
            
        elif key == ord('L') and state == FLY:
            state = LAND
            land_stage = 0
            land_stage_timer = t
            print(f'>> AUTO-LAND initiated at current position')
        key = kb.getKey() 
        
    # 4. State Management & Periodic Debug Printing
    print_counter += 1
    if state == TAKEOFF and abs(currentPos.z - TAKEOFF_ALT) < ALT_REACHED:
        targetPos.x = currentPos.x
        targetPos.y = currentPos.y
        targetYaw = yaw
        state = FLY
        print(f'>> TAKEOFF COMPLETE. Holding position at X:{targetPos.x:.2f}, Y:{targetPos.y:.2f}. Switching to fly state')
    
    if state == FLY and print_counter % 100 == 0:
        # Periodic status update
        actual_yaw_deg = math.degrees(yaw) % 360.0
        print(f'Target: (X: {targetPos.x:.2f}, Y: {targetPos.y:.2f}) | Yaw: {targetYaw:.2f}° | Altitude: {targetPos.z:.2f})\nActual: (X: {currentPos.x:.2f}, Y: {currentPos.y:.2f}) | Yaw: {targetYaw:.2f}° | Altitude: {currentPos.z:.2f}')
            
    # 5. Position Controller (World to Body)
    cosY, sinY = math.cos(yaw), math.sin(yaw)
    ex, ey = (targetPos.x - currentPos.x), (targetPos.y - currentPos.y)
    
    body_ex = ex * cosY + ey * sinY
    body_ey = ey * cosY - ex * sinY
    body_vx = f_xVel * cosY + f_yVel * sinY
    body_vy = f_yVel * cosY - f_xVel * sinY

    if state in [FLY, TAKEOFF, LAND]:
        # Update Integrals with clamping to prevent Windup
        i_ex = clamp(i_ex + body_ex * dt, -0.5, 0.5)
        i_ey = clamp(i_ey + body_ey * dt, -0.5, 0.5)

        # PID Formula: (Proportional + Integral) - Derivative
        pitch_cmd = (K_POS_P * body_ex) + (K_POS_I * i_ex) - (K_POS_D * body_vx)
        roll_cmd  = -((K_POS_P * body_ey) + (K_POS_I * i_ey) - (K_POS_D * body_vy))
        
        desired_pitch = clamp(pitch_cmd + pitch_d, -0.3, 0.3)
        desired_roll = clamp(roll_cmd + roll_d, -0.3, 0.3)
        
        # 6. Altitude Controller
        raw_v_error = targetPos.z - currentPos.z
        
        i_ez = clamp(i_ez + raw_v_error * dt, -2.0, 2.0)
        
        # calculate vertical input
        # mix: Proportional (P) + Integral (I) - Derivative(D) + Offset
        p_term = K_VERTICAL_P * raw_v_error
        i_term = K_VERTICAL_I * i_ez
        d_term = K_VERTICAL_D * f_zVel
        
        # subtract d_term to oppose direction of travel
        vertical_input = p_term + i_term - d_term+ K_VERTICAL_OFFSET
    
        # 7. Attitude Controller with Gyro Damping
        roll_input = K_ROLL_P * clamp(roll - desired_roll, -1.0, 1.0) + (roll_vel * K_GYRO_D)
        pitch_input = K_PITCH_P * clamp(pitch - desired_pitch, -1.0, 1.0) + (pitch_vel * K_GYRO_D)
        
        target_rad = math.radians(targetYaw)
        # Shift 0-2pi to -pi to pi to match IMU
        if target_rad > math.pi: target_rad -= 2.0 * math.pi
        
        yaw_error = target_rad - yaw
        
        # Shortest path wrap-around for rotation
        while yaw_error > math.pi: yaw_error -= 2.0 * math.pi
        while yaw_error < -math.pi: yaw_error += 2.0 * math.pi
        
        yaw_input = (K_YAW_P * yaw_error) - (yaw_vel * K_GYRO_D)
        
        # Altitude loss compensation while moving
        # calculate thrust loss due to tilt
        cos_factor = math.cos(roll) * math.cos(pitch)
        
        # "ideal" multiplier for counteracting gravity
        # clamp factor to prevent motor oversaturation (maximum 40% boost)
        # 0.707 corresponss to ~45 degree tilt
        boost_requirement = 1.0 / max(cos_factor, 0.707)
        
        thrustCompensation = 1.0 + (boost_requirement - 1.0) * K_COMP_AGGRESSION
        
        # apply compensation to vertical components only
        base_thrust = (K_VERTICAL_THRUST + vertical_input) * thrustCompensation
        
        # 8. Motor Mixer
        fl_v = base_thrust - roll_input + pitch_input - yaw_input
        fr_v = base_thrust + roll_input + pitch_input + yaw_input
        rl_v = base_thrust - roll_input - pitch_input + yaw_input
        rr_v = base_thrust + roll_input - pitch_input - yaw_input
        
        # 9. Set Velocities
        fl.setVelocity( fl_v); fr.setVelocity(-fr_v)
        rl.setVelocity(-rl_v); rr.setVelocity( rr_v)
    else:
        # IDLE or DONE: Keep motors off
        fl.setVelocity(0.0); fr.setVelocity(0.0)
        rl.setVelocity(0.0); rr.setVelocity(0.0)
    
     # ── LAND — staged descent
    if state == LAND:
        if land_stage < len(LAND_SEQUENCE):
            tgt, hold = LAND_SEQUENCE[land_stage]
            targetPos.z = tgt
            
            # Advance only when altitude is reached AND hold time has elapsed
            if abs(currentPos.z - tgt) < ALT_REACHED and (t- land_stage_timer) > hold:
                print(f'>> Land stage {land_stage + 1} complete')
                land_stage += 1
                land_stage_timer = t # reset timer for the next stage
        else:
            state = DONE # all stages complete — cut motors
   
    # ── DONE — cut all motors and skip the rest of the loop
    if state == DONE:
        for m in [fl, fr, rl, rr]:
            m.setVelocity(0.0)
            continue # skip PID and mixer this step