#!/usr/bin/env python

# Example code for a script using the ReFlex Takktile hand
# Note: you must connect a hand by running "roslaunch reflex reflex.launch" before you can run this script

from math import pi, cos
import sys
from time import sleep
from reflex_one_hand_NoROS import ReflexOneHand
sys.path.insert(0, './reflex_msg/msg/')
test_hand = ReflexOneHand()
name = test_hand.namespace
motors = test_hand.motors


def main():

    ##################################################################################################################
    # Calibrate the fingers (make sure hand is opened in zero position)
    raw_input("== When ready to calibrate the hand, press [Enter]\n")
    print('Go to the window where reflex_sf was run, you will see the calibration prompts')
    test_hand.calibrate()
    raw_input("...\n")

    ##################################################################################################################
    # Demonstration of position control
    raw_input("== When ready to wiggle fingers with position control, hit [Enter]\n")
    for i in range(60):
        setpoint = (-cos(i / 5.0) + 1) * 1.75
        test_hand._receive_angle_cmd_cb({"f1":setpoint, "f2":setpoint, "f3":setpoint, "preshape1":0.0, "preshape2":0},True)
        sleep(0.25)
    raw_input("...\n")

    ##################################################################################################################
    # Demonstration of preshape joint
    raw_input("== When ready to test preshape joint, hit [Enter]\n")
    test_hand._receive_angle_cmd_cb({"preshape1":0.9, "preshape2":0.9},True)
    sleep(2.0)
    test_hand._receive_angle_cmd_cb({},True)
    sleep(2.0)
    raw_input("...\n")

    ##################################################################################################################
    #Demonstration of velocity control - variable closing speed
    raw_input("== When ready to open and close fingers with velocity control, hit [Enter]\n")
    for i in range(3):
    #    test_hand._receive_angle_cmd_cb({"f1":i/6.0, "f2":i/6.0, "f3":i/6.0},True)
        sleep(2.0)
        setpoint = (2 -i *1.5)
        test_hand._receive_vel_cmd_cb({"f1":setpoint, "f2":setpoint, "f3":setpoint}, True)

        sleep(7.0 - setpoint)
    raw_input("...\n")
    test_hand._receive_angle_cmd_cb({}, True)

    ##################################################################################################################
    # Demonstration of blended control - asymptotic approach to goal - uses hand_state
    raw_input("== When ready to approach target positions with blended control, hit [Enter]\n")
    data = {}
    data["pose"] = {}
    data["pose"] = {"f1":1, "f2":1, "f3":1.0}

    data["velocity"] = {}
    for i in range(1, 5):


        data["velocity"]["f1"] = round((data["pose"]["f1"] - test_hand.hand_state["_f1"].get("position", 1) + 0.5))
        data["velocity"]["f2"] = round((data["pose"]["f2"] - test_hand.hand_state["_f2"].get("position", 1) + 0.5))
        data["velocity"]["f3"] = round((data["pose"]["f3"] - test_hand.hand_state["_f3"].get("joint_angle", 1) + 0.5))/5.0

        test_hand._receive_cmd_cb(data, True)
        sleep(0.75)
    raw_input("...\n")

    ##################################################################################################################
    # # Demonstration of force control - square wave
    # # NEEDS TESTING
    # raw_input("== When ready to feel variable force control, hit [Enter]\n")
    # raw_input("== Putting your arm or hand in the hand will allow you to feel the effect [Enter]\n")
    # test_hand._receive_angle_cmd_cb({"f1":1.5, "f2":1.5, "f3":1.5})
    # sleep(2.0)
    # test_hand._receive_force_cmd_cb({"f1":300.0})
    # sleep(5.0)
    # test_hand._receive_force_cmd_cb({"f1":100.0})
    # sleep(3.0)
    # test_hand._receive_force_cmd_cb({"f2":300.0})
    # sleep(5.0)
    # test_hand._receive_force_cmd_cb({"f2":100.0})
    # sleep(3.0)
    # test_hand._receive_force_cmd_cb({"f3":300.0})
    # sleep(5.0)
    # test_hand._receive_force_cmd_cb({"f3":100.0})
    # sleep(3.0)
    # test_hand._receive_force_cmd_cb({"f1":300.0})
    # test_hand._receive_vel_cmd_cb({"f1":5.0, "f2":5.0, "f3":5.0})
    # sleep(2.0)
    # raw_input("...\n")


def hand_state_cb(data):
    global hand_state
    hand_state = data


if __name__ == '__main__':
    main()
