from motor import Motor

class ReflexTakktileMotor(Motor):
    def __init__(self, name):
        super(ReflexTakktileHand, self).__init__(name)
        self.motor_cmd = 0.0
        self.speed = 0.0
        self.finger = None
        self.tactile_stops_enabled = False
        self.position_update_occurred = False
        self.speed_update_occurred = False

    def get_commanded_position(self):
        return self.motor_cmd

    def get_commanded_speed(self):
        return self.speed

    def set_motor_angle(self, goal_pos):
        '''
        Bounds the given position command and sets it to the motor
        '''
        self.motor_cmd = self.check_motor_angle_command(goal_pos)
        self.position_update_occurred = True

    def check_motor_angle_command(self, angle_command):
        '''
        Returns given command if within the allowable range,
        returns bounded command if out of range
        '''
        bounded_command = min(max(angle_command, 0.0), self.MAX_MOTOR_TRAVEL)
        return bounded_command

    def set_motor_speed(self, goal_speed):
        '''
        Bounds the given position command and sets it to the motor
        '''
        self.speed = self.check_motor_speed_command(goal_speed)
        self.speed_update_occurred = True

    def reset_motor_speed(self):
        '''
        Resets speed to default
        '''
        self.speed = self.DEFAULT_MOTOR_SPEED
        self.speed_update_occurred = True

    def set_motor_velocity(self, goal_vel):
        '''
        Sets speed and commands finger in or out based on sign of velocity
        '''
        self.set_motor_speed(goal_vel)
        if goal_vel > 0.0:
            self.set_motor_angle(self.MAX_MOTOR_TRAVEL)
        elif goal_vel <= 0.0:
            self.set_motor_angle(0.0)

    def tighten(self, tighten_angle=0.05):
        '''
        Takes the given angle offset in radians and tightens the motor
        '''
        self.set_motor_angle(self.motor_msg.joint_angle + tighten_angle)

    def loosen(self, loosen_angle=0.05):
        '''
        Takes the given angle offset in radians and loosens the motor
        '''
        self.set_motor_angle(self.motor_msg.joint_angle - loosen_angle)

    def receive_state_cb(self, data):
        self.motor_msg = data
        self.handle_motor_load(data.load)

    def handle_motor_load(self, load):
        if self.in_control_force_mode:
            self.control_force(load, k=16e-4*0.025)
        elif self.finger and self.tactile_stops_enabled:
            self.loosen_if_in_contact()
        self.loosen_if_overloaded(load)

    def loosen_if_in_contact(self):
        '''
        Takes the finger tactile data, loosens motor if in contact
        '''
        tolerance = 0.001
        if self.finger.is_finger_in_contact() and (self.motor_cmd > self.motor_msg.joint_angle + tolerance):
            rospy.logdebug("Motor %s in contact", self.name)
            self.loosen(0)

    def disable_tactile_stops(self):
        self.tactile_stops_enabled = False

    def enable_tactile_stops(self):
        self.tactile_stops_enabled = True
