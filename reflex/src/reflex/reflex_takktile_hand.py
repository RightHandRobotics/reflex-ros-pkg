#!/usr/bin/env python

import rospy

import reflex_msgs.srv
import finger
from reflex_takktile_motor import ReflexTakktileMotor
from reflex_hand import ReflexHand


class ReflexTakktileHand(ReflexHand):
    def __init__(self):
        super(ReflexTakktileHand, self).__init__('/reflex_takktile', ReflexTakktileMotor)
        self.motor_cmd_pub = rospy.Publisher(self.namespace + '/radian_hand_command',
                                             reflex_msgs.msg.RadianServoCommands, queue_size=10)
        self.fingers = {self.namespace + '_f1': finger.Finger(),
                        self.namespace + '_f2': finger.Finger(),
                        self.namespace + '_f3': finger.Finger()}
        self.connect_motors_to_fingers()
        self.set_speed_service = rospy.ServiceProxy(self.namespace + '/set_speed', reflex_msgs.srv.SetSpeed)
        self.calibrate_fingers_service = rospy.ServiceProxy(self.namespace + '/calibrate_fingers', Empty)
        self.calibrate_tactile_service = rospy.ServiceProxy(self.namespace + '/calibrate_tactile', Empty)
        rospy.Service(self.namespace + '/enable_tactile_stops', Empty, self.enable_tactile_stops)
        rospy.Service(self.namespace + '/disable_tactile_stops', Empty, self.disable_tactile_stops)
        self.tactile_stops_enabled = False
        self.comms_timeout = 5.0  # Seconds with no communications until hand stops
        self.latest_update = rospy.get_rostime()
        rospy.Subscriber(self.namespace + '/hand_state',
                         reflex_msgs.msg.Hand, self.receive_hand_state_cb)

    def receive_cmd_cb(self, data):
        reset = self.tactile_stops_enabled
        if reset:
            self.disable_tactile_stops()
        self.disable_force_control()
        self.set_speeds(data.velocity)
        self.set_angles(data.pose)
        if reset:
            self.enable_tactile_stops()

    def receive_angle_cmd_cb(self, data):
        reset = self.tactile_stops_enabled
        if reset:
            self.disable_tactile_stops()
        self.disable_force_control()
        self.reset_speeds()
        self.set_angles(data)
        if reset:
            self.enable_tactile_stops()

    def receive_vel_cmd_cb(self, data):
        reset = self.tactile_stops_enabled
        if reset:
            self.disable_tactile_stops()
        self.disable_force_control()
        self.set_velocities(data)
        if reset:
            self.enable_tactile_stops()

    def receive_force_cmd_cb(self, data):
        self.disable_tactile_stops()
        self.disable_force_control()
        self.reset_speeds()
        self.set_force_cmds(data)
        self.enable_force_control()

    def connect_motors_to_fingers(self):
        for ID, motor in self.motors.items():
            if ID != self.namespace + '_preshape':
                motor.finger = self.fingers[ID]

    def receive_hand_state_cb(self, data):
        self.latest_update = rospy.get_rostime()
        self.motors[self.namespace + '_f1'].receive_state_cb(data.motor[0])
        self.motors[self.namespace + '_f2'].receive_state_cb(data.motor[1])
        self.motors[self.namespace + '_f3'].receive_state_cb(data.motor[2])
        self.motors[self.namespace + '_preshape'].receive_state_cb(data.motor[3])
        self.fingers[self.namespace + '_f1'].receive_state_cb(data.finger[0])
        self.fingers[self.namespace + '_f2'].receive_state_cb(data.finger[1])
        self.fingers[self.namespace + '_f3'].receive_state_cb(data.finger[2])

    def disable_tactile_stops(self, data=None):
        self.tactile_stops_enabled = False
        for ID, motor in self.motors.items():
            motor.disable_tactile_stops()
        rospy.sleep(0.05)  # Lets commands stop before allowing any other actions
        return []

    def enable_tactile_stops(self, data=None):
        rospy.sleep(0.07)  # Lets other actions happen before beginning constant commands
        self.tactile_stops_enabled = True
        for ID, motor in self.motors.items():
            motor.enable_tactile_stops()
        return []

    def calibrate_fingers(self):
        self.calibrate_fingers_service()

    def calibrate_tactile(self):
        self.calibrate_tactile_service()


def main():
    rospy.sleep(2.0)  # To allow services and parameters to load
    hand = ReflexTakktileHand()
    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        hand.publish_motor_commands()
        if (rospy.get_rostime().secs > (hand.latest_update.secs + hand.comms_timeout)):
            rospy.logfatal('Hand going down, no ethernet data for %d seconds', hand.comms_timeout)
            rospy.signal_shutdown('Comms timeout')
        r.sleep()


if __name__ == '__main__':
    main()
