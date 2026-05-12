:x


#from webots_drone.utils import bytes2image


class Drone:
    def __init__(self, robot):
        # Time helpers
        self.time_counter = 0

        # Variables
        self.lift_thrust = 68.5  # with this thrust, the drone lifts.
        self.init_dist_sensors(robot, int(robot.getBasicTimeStep()))
        self.init_devices(robot, int(robot.getBasicTimeStep()))
        self._position = np.array([0.0, 0.0, 0.0])

    def init_dist_sensors(self, drone_node, timestep):
        """Initialize each sensor distance of the Mavic 2 Pro.

        :param drone_node Robot: The instantiated Robot Node class.
        :param integer timestep: The simulation timestep, 8ms mus be setted,
            unexpected behaviour can occur with a different value.
        """
        self.sensors_id = ['front left dist sonar',
                           'front right dist sonar',
                           'rear top dist sonar',
                           'rear bottom dist sonar',
                           'left side dist sonar',
                           'right side dist sonar',
                           'down front dist sonar',
                           'down back dist sonar',
                           'top dist infrared']
        # instantiate distance sensors
        self.sensors = list()
        for sid in self.sensors_id:
            sensor = drone_node.getDevice(sid)
            sensor.enable(timestep)
            self.sensors.append(sensor)

        return True

    def init_devices(self, drone_node, timestep):
        # Position coordinates [X, Y, Z]
        self.gps = drone_node.getDevice("gps")
        self.gps.enable(timestep)
        print("Initialised the GPS.")
        ## Angles respect global coordinates [roll, pitch, yaw]
        #self.imu = drone_node.getDevice("inertial unit")
        #self.imu.enable(timestep)
        ## Acceleration angles [roll, pitch, yaw]
        #self.gyro = drone_node.getDevice("gyro")
        #self.gyro.enable(timestep)
        ## Direction degree with north as reference
        #self.compass = drone_node.getDevice("compass")
        #self.compass.enable(timestep)

        # Motors
        #self.motors_id = ['front left propeller',
        #                  'front right propeller',
        #                  'rear left propeller',
        #                  'rear right propeller']
        #self.motors = list()
        #for mid in self.motors_id:
        #    motor = drone_node.getDevice(mid)
        #    motor.setPosition(float('inf'))
        #    motor.setVelocity(1.)
        #    self.motors.append(motor)

        #return True

    def get_odometry(self):
        """Get the drone's current acceleration, angles and position."""
        orientation = self.imu.getRollPitchYaw()
        angular_velocity = self.gyro.getValues()
        position = self.gps.getValues()
        speed = self.gps.getSpeedVector()
        compass = self.compass.getValues()
        north_rad = np.arctan2(compass[1], compass[0])

        return orientation, angular_velocity, position, speed, north_rad

    def get_dist_sensors(self):
        """Get the Distance sensors Nodes' measurements."""
        sensors = dict()
        for i, sensor_name in enumerate(self.sensors_id):
            dist_sensor = self.sensors[i]
            sensors[sensor_name] = [dist_sensor.getValue(),
                                    dist_sensor.getMinValue(),
                                    dist_sensor.getMaxValue()]
        return sensors

    def get_camera_image_shape(self):
        """Get the camera image dimension and channels."""
        return (self.camera.getHeight(), self.camera.getWidth(), 4)  # channels

    def set_motors_velocity(self, fl_motor, fr_motor, rl_motor, rr_motor):
        """Set the drone's motor velocity."""
        # Actuate over the motors
        if not np.isnan(fl_motor):
            self.motors[0].setVelocity(self.lift_thrust + fl_motor)
            self.motors[1].setVelocity(-(self.lift_thrust + fr_motor))
            self.motors[2].setVelocity(-(self.lift_thrust + rl_motor))
            self.motors[3].setVelocity(self.lift_thrust + rr_motor)
