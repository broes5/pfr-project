from controller import Robot

N_DRONES = 5
DEBUG_TARGETS = [
    (0.0, -2.0, 2.0),
    (0.0, -1.0, 2.0),
    (0.0,  0.0, 2.0),
    (0.0,  1.0, 2.0),
    (0.0,  2.0, 2.0),
]

robot = Robot()
timestep = int(robot.getBasicTimeStep())
emitter = robot.getDevice('emitter')

targets_sent = False
takeoff_sent = False

while robot.step(timestep) != -1:
    t = robot.getTime()

    if not targets_sent:
        for i in range(N_DRONES):
            name = f"BrickDrone_{i}"
            x, y, z = DEBUG_TARGETS[i]
            msg = f"{name} TARGET {x} {y} {z}"
            emitter.send(msg.encode('utf-8'))
            print(f"[CTRL] Sent: {msg}")
        targets_sent = True

    if not takeoff_sent and t >= 2.0:
        for i in range(N_DRONES):
            name = f"BrickDrone_{i}"
            msg = f"{name} TAKEOFF"
            emitter.send(msg.encode('utf-8'))
            print(f"[CTRL] Sent: {msg}")
        takeoff_sent = True
