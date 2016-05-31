import unittest

import mock

import motor


class TestMotor(unittest.TestCase):
    @mock.patch('rospy.get_param')
    def setUp(self, param_mock):
        self.name = '/reflex_takktile_f1'
        self.motor = motor.Motor(self.name)

    def test_check_motor_angle_command(self):
        self.motor.MAX_MOTOR_TRAVEL = 5.0
        self.assertAlmostEqual(self.motor.check_motor_angle_command(9.0), 5.0)
        self.assertAlmostEqual(self.motor.check_motor_angle_command(-9.0), 0.0)
        self.assertAlmostEqual(self.motor.check_motor_angle_command(2.5), 2.5)

    def test_check_motor_speed_command(self):
        self.motor.MAX_MOTOR_SPEED = 5.0
        self.assertAlmostEqual(self.motor.check_motor_speed_command(9.0), 5.0)
        self.assertAlmostEqual(self.motor.check_motor_speed_command(-9.0), 5.0)
        self.assertAlmostEqual(self.motor.check_motor_speed_command(4.9), 4.9)
        self.assertAlmostEqual(self.motor.check_motor_speed_command(-3.5), 3.5)

    @mock.patch('motor.Motor.set_motor_angle')
    @mock.patch('motor.Motor.set_motor_speed')
    def test_set_motor_velocity(self, set_speed_mock, set_angle_mock):
        self.motor.set_motor_velocity(4.11)
        set_speed_mock.called_once_with(4.11)
        set_angle_mock.called_once_with(self.motor.MAX_MOTOR_TRAVEL)

        self.motor.set_motor_velocity(-3.98)
        set_speed_mock.called_once_with(-3.98)
        set_angle_mock.called_once_with(0.0)

    @mock.patch('motor.Motor.loosen_if_overloaded')
    def test_receive_state_cb(self, loose_mock):
        data = mock.MagicMock()
        data.joint_angle = 3.75
        data.raw_angle = 20.0
        data.load = 10.0
        self.motor.zero_point = 12.5
        self.motor._motor_msg.load = 22.0

        self.motor.receive_state_cb(data)
        self.assertAlmostEqual(self.motor._motor_msg.joint_angle, 3.75)
        self.assertAlmostEqual(self.motor._motor_msg.raw_angle, 20.0)
        self.assertAlmostEqual(self.motor._motor_msg.load, 20.8)
        loose_mock.called_once_with(20.8)


if __name__ == '__main__':
    unittest.main()
