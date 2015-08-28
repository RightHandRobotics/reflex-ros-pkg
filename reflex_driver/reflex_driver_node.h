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

#ifndef REFLEX_DRIVER_NODE_H
#define REFLEX_DRIVER_NODE_H

#define CALIBRATION_ERROR 0.05  // Encoder delta signifying movement in calibration

void signal_handler(int signum);
void load_params(ros::NodeHandle nh);

void receive_raw_cmd_cb(reflex_hand::ReflexHand *rh,
                        const reflex_msgs::RawServoCommands::ConstPtr &msg);
void receive_angle_cmd_cb(reflex_hand::ReflexHand *rh,
                          const reflex_msgs::RadianServoCommands::ConstPtr &msg);
uint16_t pos_rad_to_raw(float rad_command, int motor_idx);
bool set_motor_speed(reflex_hand::ReflexHand *rh,
					 reflex_msgs::SetSpeed::Request &req, reflex_msgs::SetSpeed::Response &res);
uint16_t speed_rad_to_raw(float rad_per_s_command, int motor_idx);
void check_for_potential_motor_wraps_and_rezero();
bool enable_torque(reflex_hand::ReflexHand *rh, std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
bool disable_torque(reflex_hand::ReflexHand *rh, std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
bool calibrate_tactile(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
bool calibrate_fingers(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
void populate_tactile_threshold(int threshold);
bool set_tactile_threshold(reflex_msgs::SetTactileThreshold::Request &req,
                           reflex_msgs::SetTactileThreshold::Response &res);
int pressure_offset(int finger, int sensor);
int update_encoder_offset(int raw_value, int last_value, int current_offset);
float calc_proximal_angle(int raw_enc_value, int offset, double zero);
float calc_motor_angle(int inversion, int raw_dyn_value, double ratio, double zero);
float calc_distal_angle(float spool, float proximal);
int calc_pressure(const reflex_hand::ReflexHandState* const state, int finger, int sensor);
int calc_contact(reflex_msgs::Hand hand_msg, int finger, int sensor);
void reflex_hand_state_cb(const reflex_hand::ReflexHandState * const state);
void calibrate_tactile_sensors(const reflex_hand::ReflexHandState* const state, reflex_msgs::Hand hand_msg);
void log_current_tactile_locally(const reflex_hand::ReflexHandState* const state);
void log_current_tactile_to_file(const reflex_hand::ReflexHandState* const state, int finger);
void calibrate_encoders_locally(const reflex_hand::ReflexHandState* const state);
bool check_for_finger_movement(const reflex_hand::ReflexHandState* const state);
void move_fingers_in(const reflex_hand::ReflexHandState* const state);
void calibrate_motors_locally(const reflex_hand::ReflexHandState* const state);
void log_encoder_zero_to_file();
void log_motor_zero_to_file_and_close();
void populate_motor_state(reflex_msgs::Hand* hand_msg, const reflex_hand::ReflexHandState* const state);
float load_raw_to_signed(int load, int motor_idx);

#endif
