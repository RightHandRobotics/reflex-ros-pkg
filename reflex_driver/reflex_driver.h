/////////////////////////////////////////////////////////////////////////////
//
//   Copyright 2014 Open Source Robotics Foundation, Inc.
//   Copyright 2014-2015 Right Hand Robotics
//
//   Licensed under the Apache License, Version 2.0 (the "License");
//   you may not use this file except in compliance with the License.
//   You may obtain a copy of the License at
//
//       http://www.apache.org/licenses/LICENSE-2.0
//
//   Unless required by applicable law or agreed to in writing, software
//   distributed under the License is distributed on an "AS IS" BASIS,
//   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//   See the License for the specific language governing permissions and
//   limitations under the License.
//
/////////////////////////////////////////////////////////////////////////////

#ifndef REFLEX_DRIVER_H
#define REFLEX_DRIVER_H

#include <reflex_msgs/Hand.h>
#include <reflex_msgs/RawServoCommands.h>
#include <reflex_msgs/RadianServoCommands.h>
#include <reflex_msgs/SetSpeed.h>
#include <reflex_msgs/SetTactileThreshold.h>
#include <ros/ros.h>

#include <stdio.h>
#include <stdlib.h>
#include <std_srvs/Empty.h>

#include <string>
#include <signal.h>
#include <vector>
#include <fstream>

#define CALIBRATION_ERROR 0.05  // Encoder delta signifying movement in calibration

namespace reflex_driver {

  class ReflexDriver {
  public:
    ReflexDriver(const std::string &network_interface, const int pb, const std::string &hand_prefix, const char* mcast_addr_str);
    static const int TACTILE_BASE_IDX[NUM_TAKKTILE];               // Constant
    static const uint16_t CALIBRATION_DYN_OFFSET[NUM_DYNAMIXELS];  // Constant

    reflex_hand::ReflexHand rh;

    ros::Publisher hand_pub;
    ros::Publisher raw_pub;

    reflex_msgs::SetTactileThreshold::Request contact_thresholds;   // Set by /hand[x]/set_tactile_threshold ROS service

    std::ofstream tactile_file;        // Accesses tactile calibration file
    std::string tactile_file_address;  // Set in constructor
    std::ofstream finger_file;         // Accesses finger calibration file
    std::string finger_file_address;   // Set in constructor

    std::string hand;
    
    // tactile file
    std::vector<int> tactile_offset_f1;              // Loaded from yaml and reset during calibration
    std::vector<int> tactile_offset_f2;              // Loaded from yaml and reset during calibration
    std::vector<int> tactile_offset_f3;              // Loaded from yaml and reset during calibration

    // finger file
    std::vector<double> dynamixel_zero_point;  // Loaded from yaml and reset during calibration
    std::vector<double> encoder_zero_point;    // Loaded from yaml and reset during calibration
    int default_contact_threshold;

    // motor constants
    std::vector<int> MOTOR_TO_JOINT_INVERTED;        // Loaded from yaml
    std::vector<double> MOTOR_TO_JOINT_GEAR_RATIO;   // Loaded from yaml

    bool acquire_tactile;         // Updated by /hand[x]/calibrate_tactile ROS service and in reflex_hand_state_cb()
    bool acquire_fingers;         // Updated by /hand[x]/calibrate_tactile ROS service and in reflex_hand_state_cb()
    bool first_capture;           // Updated by /hand[x]/calibrate_tactile ROS service and in reflex_hand_state_cb()
    bool all_fingers_moved;       // Updated in reflex_hand_state_cb()

    int encoder_last_value[NUM_ENCODERS];     // Updated constantly in reflex_hand_state_cb()
    int encoder_offset[NUM_ENCODERS];         // Updated constantly in reflex_hand_state_cb()
    float load_last_value[NUM_DYNAMIXELS];    // Updated constantly in reflex_hand_state_cb()
    int raw_cmd_last_value[NUM_DYNAMIXELS];   // Updated in receive_raw_cmd_cb and receive_angle_cmd_cb

    uint16_t calibration_dyn_increase[NUM_DYNAMIXELS];   // Updated in reflex_hand_state_cb() during calibration

    ros::Time latest_calibration_time;

    static void signal_handler(int signum) {
      if (signum == SIGINT || signum == SIGTERM) {
        printf("Exiting. Have a nice day.\n");
        exit(1);
      }
    }

    void init_params();
    void load_params(ros::NodeHandle nh);

    uint16_t pos_rad_to_raw(float rad_command, int motor_idx);
    uint16_t speed_rad_to_raw(float rad_per_s_command, int motor_idx);
    float load_raw_to_signed(int load, int motor_idx);
    void check_for_potential_motor_wraps_and_rezero();

    void set_raw_position(const uint16_t *targets);
    void move_fingers_in(const reflex_hand::ReflexHandState * const state);

    void populate_tactile_threshold(int threshold);
    bool set_tactile_threshold(reflex_msgs::SetTactileThreshold::Request &req,
                              reflex_msgs::SetTactileThreshold::Response &res);

    bool set_motor_speed(reflex_msgs::SetSpeed::Request &req, reflex_msgs::SetSpeed::Response &res);
    bool enable_torque(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool disable_torque(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    bool calibrate_tactile(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool calibrate_fingers(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    int pressure_offset(int finger, int sensor);
    int update_encoder_offset(int raw_value, int last_value, int current_offset);

    float calc_proximal_angle(int raw_enc_value, int offset, double zero);
    float calc_motor_angle(int inversion, int raw_dyn_value, double ratio, double zero);
    float calc_distal_angle(float spool, float proximal);
    int calc_pressure(const reflex_hand::ReflexHandState * const state, int finger, int sensor);
    int calc_contact(reflex_msgs::Hand hand_msg, int finger, int senso);

    void calibrate_tactile_sensors(const reflex_hand::ReflexHandState* const state, reflex_msgs::Hand hand_msg);
    void calibrate_encoders_locally(const reflex_hand::ReflexHandState* const state);
    void calibrate_motors_locally(const reflex_hand::ReflexHandState* const state);
    
    void log_current_tactile_locally(const reflex_hand::ReflexHandState* const state);
    void log_current_tactile_to_file(const reflex_hand::ReflexHandState* const state, int finger);
    void log_encoder_zero_to_file();
    void log_motor_zero_to_file_and_close();

    bool check_for_finger_movement(const reflex_hand::ReflexHandState* const state);
    void populate_motor_state(reflex_msgs::Hand* hand_msg, const reflex_hand::ReflexHandState* const state);
  	
    void receive_raw_cmd_cb(const reflex_msgs::RawServoCommands::ConstPtr &msg);
  	void receive_angle_cmd_cb(const reflex_msgs::RadianServoCommands::ConstPtr &msg);
    void reflex_hand_state_cb(const reflex_hand::ReflexHandState * const state);
  };
}

#endif
