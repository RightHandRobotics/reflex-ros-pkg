import unittest

import mock

import reflex_takktile_hand
from reflex_msgs.msg import RadianServoCommands
from reflex_msgs.srv import SetSpeedRequest


class TestReflexTakktileHand(unittest.TestCase):
    @mock.patch('rospy.get_rostime')
    @mock.patch('rospy.Subscriber')
    @mock.patch('motor.Motor')
    @mock.patch('rospy.loginfo')
    @mock.patch('rospy.init_node')
    def setUp(self, init_mock, loginfo_mock, motor_mock, sub_mock, rostime_mock):
        self.rh = reflex_takktile_hand.ReflexTakktileHand()

    @mock.patch('reflex_takktile_hand.ReflexTakktileHand.publish_motor_commands')
    @mock.patch('reflex_takktile_hand.ReflexTakktileHand.set_angles')
    @mock.patch('reflex_takktile_hand.ReflexTakktileHand.set_speeds')
    def test_receive_cmd_cb(self, set_speeds_mock, set_angles_mock, publish_mock):
        data = mock.MagicMock()
        data.velocity = 5.0
        data.pose = 14.0
        self.rh.receive_cmd_cb(data)
        set_speeds_mock.called_once_with(5.0)
        set_angles_mock.called_once_with(14.0)
        publish_mock.called_once_with()

    @mock.patch('reflex_takktile_hand.ReflexTakktileHand.publish_motor_commands')
    @mock.patch('reflex_takktile_hand.ReflexTakktileHand.set_angles')
    @mock.patch('reflex_takktile_hand.ReflexTakktileHand.reset_speeds')
    def test_receive_angle_cmd_cb(self, reset_speeds_mock, set_angles_mock, publish_mock):
        data = 12345
        self.rh.receive_angle_cmd_cb(data)
        reset_speeds_mock.called_once_with()
        set_angles_mock.called_once_with(12345)
        publish_mock.called_once_with()

    @mock.patch('reflex_takktile_hand.ReflexTakktileHand.publish_motor_commands')
    @mock.patch('reflex_takktile_hand.ReflexTakktileHand.set_velocities')
    def test_receive_vel_cmd_cb(self, vel_mock, publish_mock):
        data = 12345
        self.rh.receive_vel_cmd_cb(data)
        vel_mock.called_once_with(12345)
        publish_mock.called_once_with()

    def test_set_angles(self):
        pose = mock.MagicMock()
        pose.f1 = 1.1
        pose.f2 = 2.2
        pose.f3 = 3.3
        pose.preshape = 4.4
        self.rh.set_angles(pose)
        self.rh.motors['/reflex_takktile_f1'].set_motor_angle.called_once_with(1.1)
        self.rh.motors['/reflex_takktile_f2'].set_motor_angle.called_once_with(2.2)
        self.rh.motors['/reflex_takktile_f3'].set_motor_angle.called_once_with(3.3)
        self.rh.motors['/reflex_takktile_preshape'].set_motor_angle.called_once_with(4.4)

    def test_set_velocities(self):
        velocity = mock.MagicMock()
        velocity.f1 = 1.1
        velocity.f2 = 2.2
        velocity.f3 = 3.3
        velocity.preshape = 4.4
        self.rh.set_velocities(velocity)
        self.rh.motors['/reflex_takktile_f1'].set_motor_velocity.called_once_with(1.1)
        self.rh.motors['/reflex_takktile_f2'].set_motor_velocity.called_once_with(2.2)
        self.rh.motors['/reflex_takktile_f3'].set_motor_velocity.called_once_with(3.3)
        self.rh.motors['/reflex_takktile_preshape'].set_motor_velocity.called_once_with(4.4)

    def test_set_speeds(self):
        speed = mock.MagicMock()
        speed.f1 = 1.1
        speed.f2 = 2.2
        speed.f3 = 3.3
        speed.preshape = 4.4
        self.rh.set_speeds(speed)
        self.rh.motors['/reflex_takktile_f1'].reset_motor_speed.called_once_with(1.1)
        self.rh.motors['/reflex_takktile_f2'].set_motor_speed.called_once_with(2.2)
        self.rh.motors['/reflex_takktile_f3'].set_motor_speed.called_once_with(3.3)
        self.rh.motors['/reflex_takktile_preshape'].set_motor_speed.called_once_with(4.4)

    def test_reset_speeds(self):
        self.rh.reset_speeds()
        self.rh.motors['/reflex_takktile_f1'].reset_motor_speed.called_once_with()
        self.rh.motors['/reflex_takktile_f2'].reset_motor_speed.called_once_with()
        self.rh.motors['/reflex_takktile_f3'].reset_motor_speed.called_once_with()
        self.rh.motors['/reflex_takktile_preshape'].reset_motor_speed.called_once_with()

    def test_calibrate_fingers(self):
        self.rh.calibrate_fingers_service = mock.MagicMock()
        self.rh.calibrate_fingers()
        self.rh.calibrate_fingers_service.called_once_with()

    def test_calibrate_tactile(self):
        self.rh.calibrate_tactile_service = mock.MagicMock()
        self.rh.calibrate_tactile()
        self.rh.calibrate_tactile_service.called_once_with()

    # Something is going wrong with shared mocks
    # def test_publish_motor_commands(self):
    #     self.rh.motors['/reflex_takktile_f1'].get_commanded_position.return_value = 1.1
    #     self.rh.motors['/reflex_takktile_f1'].get_commanded_speed.return_value = 0.1
    #     self.rh.motors['/reflex_takktile_f2'].get_commanded_position.return_value = 2.2
    #     self.rh.motors['/reflex_takktile_f2'].get_commanded_speed.return_value = 0.2
    #     self.rh.motors['/reflex_takktile_f3'].get_commanded_position.return_value = 3.3
    #     self.rh.motors['/reflex_takktile_f3'].get_commanded_speed.return_value = 0.3
    #     self.rh.motors['/reflex_takktile_preshape'].get_commanded_position.return_value = 4.4
    #     self.rh.motors['/reflex_takktile_preshape'].get_commanded_speed.return_value = 0.4
    #     self.rh.motor_cmd_pub = mock.MagicMock()
    #     self.rh.set_speed_service = mock.MagicMock()
    #     self.rh.publish_motor_commands()
    #     self.rh.motor_cmd_pub.called_once_with(RadianServoCommands([1.1, 2.2, 3.3, 4.4]))
    #     self.rh.set_speed_service.called_once_with(SetSpeedRequest([0.1, 0.2, 0.3, 0.4]))


if __name__ == '__main__':
    unittest.main()
