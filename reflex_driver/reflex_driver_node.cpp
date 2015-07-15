/////////////////////////////////////////////////////////////////////////////
//
//   Copyright 2014 Open Source Robotics Foundation, Inc.
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


#include <reflex_msgs/Hand.h>
#include <reflex_msgs/RawServoCommands.h>
#include <reflex_msgs/RadianServoCommands.h>
#include <reflex_msgs/SetSpeed.h>
#include <ros/ros.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <std_srvs/Empty.h>

#include <algorithm>
#include <fstream>
#include <string>
#include <vector>

#include "./reflex_hand.h"
#include "./reflex_driver_node.h"
using namespace std;


ros::Publisher hand_pub;
ros::Publisher debug_pub;
ros::Publisher raw_pub;

ofstream tactile_file;
string tactile_file_address;
ofstream finger_file;
string finger_file_address;

vector<int> motor_inversion;
vector<double> dyn_ratio;
int contact_threshold;  // TODO(Eric): Make this individual for every sensor
vector<int> tactile_offset_f1;
vector<int> tactile_offset_f2;
vector<int> tactile_offset_f3;
int tactile_base_idx[] = {0, 18, 9};
int encoder_last_value[] = {0, 0, 0};
int encoder_offset[] = {-1, -1, -1};

bool aqcuire_tactile, aqcuire_fingers, first_capture, all_fingers_moved = false;
vector<double> dyn_zero;
vector<double> enc_zero;
double calibration_error[] = {0.0, 0.0, 0.0};
uint16_t calibration_dyn_increase[] = {5, 5, 5, 0};
const uint16_t calibration_dyn_offset[] = {50, 50, 50, 0};

bool g_done = false;


void signal_handler(int signum) {
  if (signum == SIGINT || signum == SIGTERM)
    g_done = true;
}


void load_params(ros::NodeHandle nh) {
  string topic = "no error";
  if (!nh.getParam("motor_inversion", motor_inversion))
    topic = "motor_inversion";
  if (!nh.getParam("motor_zero_reference", dyn_zero))
    topic = "motor_zero_reference";
  if (!nh.getParam("encoder_zero_reference", enc_zero))
    topic = "encoder_zero_reference";
  if (!nh.getParam("motor_gear_ratio", dyn_ratio))
    topic = "motor_gear_ratio";
  if (!nh.getParam("contact_threshold", contact_threshold))
    topic = "contact_threshold";
  if (!nh.getParam("tactile_offset_f1", tactile_offset_f1))
    topic = "tactile_offset_f1";
  if (!nh.getParam("tactile_offset_f2", tactile_offset_f2))
    topic = "tactile_offset_f2";
  if (!nh.getParam("tactile_offset_f3", tactile_offset_f3))
    topic = "tactile_offset_f3";
  if (topic != "no error") {
    ROS_FATAL("Failed to load %s parameter", topic.c_str());
    ROS_FATAL("This is likely because the corresponding yaml file in");
    ROS_FATAL("reflex_driver/yaml were never loaded, or it's because");
    ROS_FATAL("they have been corrupted. If they were corrupted, they");
    ROS_FATAL("can be repaired by pasting the *_backup.yaml file text in");
  }
  ROS_INFO("Succesfully loaded all parameters");
}


// Takes raw Dynamixel values (0-4095) and writes them directly to the motors
void receive_raw_cmd_cb(reflex_hand::ReflexHand *rh,
                          const reflex_msgs::RawServoCommands::ConstPtr &msg) {
  uint16_t targets[reflex_hand::ReflexHand::NUM_SERVOS];
  for (int i = 0; i < reflex_hand::ReflexHand::NUM_SERVOS; i++) {
    targets[i] = msg->raw_positions[i];
  }
  rh->setServoTargets(targets);
}


// Commands the motors from radians, using the zero references from
// yaml/finger_calibrate.yaml to translate into the raw Dynamixel values
void receive_angle_cmd_cb(reflex_hand::ReflexHand *rh,
                          const reflex_msgs::RadianServoCommands::ConstPtr &msg) {
  rh->setServoControlModes(reflex_hand::ReflexHand::CM_POSITION);
  uint16_t targets[4];
  for (int i = 0; i < reflex_hand::ReflexHand::NUM_SERVOS; i++) {
    targets[i] = pos_rad_to_raw(msg->radian_commands[i], i);
  }
  rh->setServoTargets(targets);
}


// Takes a rad/s command and returns Dynamixel command
uint16_t pos_rad_to_raw(float rad_command, int motor_idx) {
  float zeroed_command = motor_inversion[motor_idx] * rad_command + dyn_zero[motor_idx];
  float motor_ratio = (dyn_ratio[motor_idx] / reflex_hand::ReflexHand::DYN_POS_SCALE);
  uint16_t command = (uint16_t) (zeroed_command * motor_ratio);
  if (command > reflex_hand::ReflexHand::DYN_MIN_RAW_WRAPPED) {
    ROS_WARN("Finger %d set out of range (%d), reset to %d", motor_idx + 1,
             (int16_t) command, reflex_hand::ReflexHand::DYN_MIN_RAW);
    command = reflex_hand::ReflexHand::DYN_MIN_RAW;
  }
  return command;
}


// Sets motor speed as a service
bool set_motor_speed(reflex_hand::ReflexHand *rh,
                     reflex_msgs::SetSpeed::Request &req, reflex_msgs::SetSpeed::Response &res) {
  rh->setServoControlModes(reflex_hand::ReflexHand::CM_VELOCITY);
  uint16_t targets[4];
  for (int i = 0; i < reflex_hand::ReflexHand::NUM_SERVOS; i++) {
    targets[i] = speed_rad_to_raw(req.motor[i], i);
  }
  rh->setServoTargets(targets);
  ros::Duration(0.01).sleep();
  rh->setServoControlModes(reflex_hand::ReflexHand::CM_POSITION);
  return true;
}


// Takes a rad/s command and returns Dynamixel command
// http://support.robotis.com/en/product/dynamixel/mx_series/mx-28.htm#Actuator_Address_20
uint16_t speed_rad_to_raw(float rad_per_s_command, int motor_idx) {
  uint16_t command = abs(rad_per_s_command) *
                    (dyn_ratio[motor_idx] / reflex_hand::ReflexHand::DYN_VEL_SCALE);
  if (motor_inversion[motor_idx] * rad_per_s_command < 0) {
    command += 1024;
  }
// TODO(Eric): Add speed bounds
  return command;
}


bool enable_torque(reflex_hand::ReflexHand *rh, std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  rh->setServoControlModes(reflex_hand::ReflexHand::CM_POSITION);
}

bool disable_torque(reflex_hand::ReflexHand *rh, std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  rh->setServoControlModes(reflex_hand::ReflexHand::CM_IDLE);
}


// Sets the procedure to calibrate the tactile values in motion
bool zero_tactile(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  aqcuire_tactile = true;
  ROS_INFO("Zeroing tactile data at current values...");
  ROS_INFO("tactile_file_address: %s", tactile_file_address.c_str());
  return true;
}


// Sets the procedure to calibrate the fingers in motion
bool zero_fingers(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  ROS_INFO("Beginning finger zero sequence...");
  ROS_INFO("finger_file_address: %s", finger_file_address.c_str());
  printf("This process assumes the fingers zeroed (opened as much as\n");
  printf("and that the hand in aligned in a cylindrical grasp. The\n");
  printf("preshape jointwill be set to zero just as it is now. If\n");
  printf("reset and rerun the calibration after this is finished...\n");
  aqcuire_fingers = true;
  first_capture = true;
  all_fingers_moved = false;
  return true;
}


// Returns the correct pressure calibration offset for finger[sensor]
int pressure_offset(int finger, int sensor) {
  if (finger == 0)
    return tactile_offset_f1[sensor];
  else if (finger == 1)
    return tactile_offset_f2[sensor];
  else
    return tactile_offset_f3[sensor];
}


// Given raw value and a past value, tracks encoder wraps: enc_offset variable
int update_encoder_offset(int raw_value, int last_value, int current_offset) {
  int enc_offset = current_offset;
  if (enc_offset == -1) {
    // This case happens upon startup
    enc_offset = 0;
  } else {
    // If the encoder value jumps, that means it has wrapped a revolution
    if (last_value - raw_value > 5000)
      enc_offset = enc_offset + 16383;
    else if (last_value - raw_value < -5000)
      enc_offset = enc_offset - 16383;
  }
  return enc_offset;
}


// Calculates actual proximal angle using raw sensor value, wrap offset
// for that finger, and calibrated "zero" point for that encoder
float calc_proximal_angle(int raw_enc_value, int offset, double zero) {
  int wrapped_enc_value = raw_enc_value + offset;
  float rad_value = (float) wrapped_enc_value * reflex_hand::ReflexHand::ENC_SCALE;
  return (rad_value - zero);
}


// Calculates joint angle from raw sensor value, motor gear ratio,
// and calibrated "zero" point for the joint
float calc_motor_angle(int inversion, int raw_dyn_value, double ratio, double zero) {
  float rad_value = raw_dyn_value * reflex_hand::ReflexHand::DYN_POS_SCALE / ratio;
  float zeroed_value = rad_value - zero;
  return inversion * zeroed_value;
}


// Calculates distal angle, "tendon spooled out" - "proximal encoder" angles
// Could be improved
float calc_distal_angle(float joint_angle, float proximal) {
  float diff = joint_angle - proximal;
  return (diff < 0) ? 0 : diff;
}


// Takes hand state and returns calibrated tactile data for given finger
int calc_pressure(const reflex_hand::ReflexHandState* const state, int finger, int sensor) {
  int raw_value = state->tactile_pressures_[tactile_base_idx[finger] + sensor];
  return raw_value - pressure_offset(finger, sensor);
}


// Checks given finger/sensor for contact threshold
int calc_contact(reflex_msgs::Hand hand_msg, int finger, int sensor) {
  int pressure_value = abs(hand_msg.finger[finger].pressure[sensor]);
  return pressure_value > contact_threshold;
}


// Takes in hand state data and publishes to reflex_hand topic
// Also does calibration when certain booleans are enabled
void reflex_hand_state_cb(const reflex_hand::ReflexHandState * const state) {
  reflex_msgs::Hand hand_msg;

  for (int i = 0; i < reflex_hand::ReflexHandState::NUM_FINGERS; i++) {
    encoder_offset[i] = update_encoder_offset(state->encoders_[i],
                                              encoder_last_value[i],
                                              encoder_offset[i]);
    encoder_last_value[i] = state->encoders_[i];
    hand_msg.motor[i].joint_angle = calc_motor_angle(motor_inversion[i],
                                                     state->dynamixel_angles_[i],
                                                     dyn_ratio[i],
                                                     dyn_zero[i]);
    hand_msg.finger[i].proximal = calc_proximal_angle(state->encoders_[i],
                                                      encoder_offset[i],
                                                      enc_zero[i]);
    hand_msg.finger[i].distal_approx = calc_distal_angle(hand_msg.motor[i].joint_angle,
                                                         hand_msg.finger[i].proximal);
    for (int j=0; j < reflex_hand::ReflexHand::NUM_SENSORS_PER_FINGER; j++) {
      hand_msg.finger[i].pressure[j] = calc_pressure(state, i, j);
      hand_msg.finger[i].contact[j] = calc_contact(hand_msg, i, j);
    }
  }
  hand_msg.motor[3].joint_angle = calc_motor_angle(motor_inversion[3],
                                                   state->dynamixel_angles_[3],
                                                   dyn_ratio[3],
                                                   dyn_zero[3]);
  populate_motor_state(&hand_msg, state);
  hand_pub.publish(hand_msg);

  // Capture the current tactile data and save it as a zero reference
  if (aqcuire_tactile) {
    calibrate_tactile_sensors(state, hand_msg);
  }

  if (aqcuire_fingers) {
    if (first_capture) {
      calibrate_encoders(state);
      first_capture = false;
    }
    all_fingers_moved = check_for_finger_movement(state);
    move_fingers_in(state);
    if (all_fingers_moved) {
      ROS_INFO("FINISHED ZEROING: Finger movement detected, zeroing motors");
      log_motor_zero_locally(state);
      check_anomalous_motor_values();
      log_motor_zero_to_file_and_close();
      aqcuire_fingers = false;
    }
  }
  return;
}


// Opens tactile calibration data file, changes local tactile_offset, and saves
// current tactile values to file as the new calibrated "zero"
void calibrate_tactile_sensors(const reflex_hand::ReflexHandState* const state,
                               reflex_msgs::Hand hand_msg) {
  tactile_file.open(tactile_file_address.c_str(), ios::out|ios::trunc);
  tactile_file << "# Captured sensor values from unloaded state\n";
  log_current_tactile_locally(state);
  for (int i = 0; i < reflex_hand::ReflexHandState::NUM_FINGERS; i++) {
    log_current_tactile_to_file(state, i);
  }
  aqcuire_tactile = false;
  tactile_file.close();
}


// Save local variables tactile_offset_f* with current tactile position
void log_current_tactile_locally(const reflex_hand::ReflexHandState* const state) {
  for (int j = 0; j < reflex_hand::ReflexHand::NUM_SENSORS_PER_FINGER; j++) {
    tactile_offset_f1[j] = state->tactile_pressures_[tactile_base_idx[0] + j];
    tactile_offset_f2[j] = state->tactile_pressures_[tactile_base_idx[1] + j];
    tactile_offset_f3[j] = state->tactile_pressures_[tactile_base_idx[2] + j];
  }
}


void log_current_tactile_to_file(const reflex_hand::ReflexHandState* const state,
                                 int finger) {
  tactile_file << "tactile_offset_f" << finger + 1 << ": ["
               << state->tactile_pressures_[tactile_base_idx[finger] + 0] << ", "
               << state->tactile_pressures_[tactile_base_idx[finger] + 1] << ", "
               << state->tactile_pressures_[tactile_base_idx[finger] + 2] << ", "
               << state->tactile_pressures_[tactile_base_idx[finger] + 3] << ", "
               << state->tactile_pressures_[tactile_base_idx[finger] + 4] << ", "
               << state->tactile_pressures_[tactile_base_idx[finger] + 5] << ", "
               << state->tactile_pressures_[tactile_base_idx[finger] + 6] << ", "
               << state->tactile_pressures_[tactile_base_idx[finger] + 7] << ", "
               << state->tactile_pressures_[tactile_base_idx[finger] + 8] << "]\n";
}


// Capture current encoder position locally as "zero" and save to file
void calibrate_encoders(const reflex_hand::ReflexHandState* const state) {
  for (int i = 0; i < reflex_hand::ReflexHandState::NUM_FINGERS; i++)
    enc_zero[i] = state->encoders_[i] * reflex_hand::ReflexHand::ENC_SCALE;
  finger_file.open(finger_file_address.c_str(), ios::out|ios::trunc);
  finger_file << "# Calbration constants for [f1, f2, f3, preshape]\n";
  finger_file << "encoder_zero_reference: ["
              << enc_zero[0] << ", "
              << enc_zero[1] << ", "
              << enc_zero[2] << "]\n";
}


// Save current dynamixel location (plus an offset) as "zero" and then
// write the dynamixels to the spot
void log_motor_zero_locally(const reflex_hand::ReflexHandState* const state) {
  reflex_msgs::RawServoCommands servo_pos;
  int motor_offset;
  float motor_scalar;

  for (int i = 0; i < reflex_hand::ReflexHand::NUM_SERVOS; i++) {
    motor_offset = motor_inversion[i] * calibration_dyn_offset[i];
    motor_scalar = reflex_hand::ReflexHand::DYN_POS_SCALE / dyn_ratio[i];
    dyn_zero[i] = (state->dynamixel_angles_[i] - motor_offset) * motor_scalar;
    servo_pos.raw_positions[i] = state->dynamixel_angles_[i] -
                                 (motor_inversion[i] * calibration_dyn_offset[i]);
  }
  raw_pub.publish(servo_pos);
}


void log_motor_zero_to_file_and_close() {
  finger_file << "motor_zero_reference: ["
              << dyn_zero[0] << ", "
              << dyn_zero[1] << ", "
              << dyn_zero[2] << ", "
              << dyn_zero[3] << "]\n";
  finger_file.close();
}


// Checks whether all fingers have moved more than CALIBRATION_ERROR, halts
// the motion of fingers past that point
bool check_for_finger_movement(const reflex_hand::ReflexHandState* const state) {
  bool all_fingers_moved = true;
  for (int i = 0; i < reflex_hand::ReflexHandState::NUM_FINGERS; i++) {
    float enc_pos = enc_zero[i] -
                    state->encoders_[i] * reflex_hand::ReflexHand::ENC_SCALE;
    if (abs(enc_pos) > CALIBRATION_ERROR) {
      calibration_dyn_increase[i] = 0;
    } else {
      all_fingers_moved = false;
    }
  }
  return all_fingers_moved;
}


// Steps the fingers in by a calibration amount
void move_fingers_in(const reflex_hand::ReflexHandState* const state) {
  reflex_msgs::RawServoCommands servo_pos;
  int motor_step;
  ROS_INFO("Stepping the fingers inwards:\t%d+%d\t%d+%d\t%d+%d\t%d+%d",
           state->dynamixel_angles_[0], calibration_dyn_increase[0],
           state->dynamixel_angles_[1], calibration_dyn_increase[1],
           state->dynamixel_angles_[2], calibration_dyn_increase[2],
           state->dynamixel_angles_[3], calibration_dyn_increase[3]);
  for (int i=0; i < reflex_hand::ReflexHand::NUM_SERVOS; i++) {
    motor_step = motor_inversion[i] * calibration_dyn_increase[i];
    servo_pos.raw_positions[i] = state->dynamixel_angles_[i] + motor_step;
  }
  raw_pub.publish(servo_pos);
}


// Sometimes Dynamixels glitch and report very high values, this catches those
void check_anomalous_motor_values() {
  for (int i = 0; i < reflex_hand::ReflexHand::NUM_SERVOS; i++) {
    if (dyn_zero[i] > 14 * 3.1415) {
        ROS_WARN("Something went wrong in calibration - motor %d was", i+1);
        ROS_WARN("\tset anomalously high. Motor zero reference value:");
        ROS_WARN("\t%4f radians (nothing should be over 2*pi)", dyn_zero[i]);
        ROS_WARN("\tTry redoing the calibration, depowering/repowering");
        ROS_WARN("\tif it repeats");
    }
  }
}


// Captures the current hand state to a debug message and publishes
void populate_motor_state(reflex_msgs::Hand* hand_msg, const reflex_hand::ReflexHandState* const state) {
  char buffer[10];
  for (int i = 0; i < reflex_hand::ReflexHand::NUM_SERVOS; i++) {
    ROS_WARN("raw angle[%d]: %d", i, state->dynamixel_angles_[i]);
    hand_msg->motor[i].raw_angle = (float) state->dynamixel_angles_[i];
    hand_msg->motor[i].velocity = (float) state->dynamixel_speeds_[i];
    hand_msg->motor[i].load = (float) state->dynamixel_loads_[i];
    hand_msg->motor[i].voltage = (float) state->dynamixel_voltages_[i];
    hand_msg->motor[i].temperature = state->dynamixel_temperatures_[i];
    sprintf(buffer, "0x%02x", state->dynamixel_error_states_[i]);
    hand_msg->motor[i].error_state = buffer;
  }
}


int main(int argc, char **argv) {
  // Initialize ROS node
  ros::init(argc, argv, "reflex_hand_driver");
  ros::NodeHandle nh, nh_private("~");
  load_params(nh);

  // Advertise necessary topics
  hand_pub = nh.advertise<reflex_msgs::Hand>("/reflex_takktile/hand_state", 10);
  ROS_INFO("Publishing the /reflex_takktile/hand_state topic");
  raw_pub = nh.advertise<reflex_msgs::RawServoCommands>("/reflex_takktile/raw_hand_command", 1);

  // Intialize the reflex_hand object
  string network_interface;
  nh_private.param<string>("network_interface", network_interface, "eth0");
  ROS_INFO("Starting reflex_hand_driver on network interface %s",
           network_interface.c_str());
  reflex_hand::ReflexHand rh(network_interface);
  if (!rh.happy()) {
    ROS_FATAL("Error during initialization. bailing now. have a nice day.");
    return 1;
  }
  rh.setStateCallback(reflex_hand_state_cb);
  rh.setServoControlModes(reflex_hand::ReflexHand::CM_POSITION);
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  // Subscribe to the hand command topics
  ros::Subscriber raw_positions_sub =
    nh.subscribe<reflex_msgs::RawServoCommands>("reflex_takktile/raw_hand_command", 1,
                                                 boost::bind(receive_raw_cmd_cb, &rh, _1));
  ros::Subscriber radian_positions_sub =
    nh.subscribe<reflex_msgs::RadianServoCommands>("reflex_takktile/radian_hand_command", 1,
                                                   boost::bind(receive_angle_cmd_cb, &rh, _1));

  // Initialize the hand command services
  ros::ServiceServer set_speed_service =
    nh.advertiseService<reflex_msgs::SetSpeed::Request, reflex_msgs::SetSpeed::Response>
    ("/reflex_takktile/set_speed", boost::bind(set_motor_speed, &rh, _1, _2));
  ros::ServiceServer enable_service =
    nh.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>
      ("reflex_takktile/enable_torque", boost::bind(enable_torque, &rh, _1, _2));
  ROS_INFO("Advertising the /enable_torque service");
  ros::ServiceServer disable_service =
    nh.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>
      ("reflex_takktile/disable_torque", boost::bind(disable_torque, &rh, _1, _2));
  ROS_INFO("Advertising the /disable_torque service");

  // Initialize the /zero_tactile and /zero_finger services
  string buffer;
  nh.getParam("yaml_dir", buffer);
  tactile_file_address = buffer + "/tactile_calibrate.yaml";
  finger_file_address = buffer + "/finger_calibrate.yaml";
  ros::ServiceServer zero_tactile_srv = nh.advertiseService("/zero_tactile", zero_tactile);
  ROS_INFO("Advertising the /zero_tactile service");
  ros::ServiceServer zero_fingers_srv = nh.advertiseService("/zero_fingers", zero_fingers);
  ROS_INFO("Advertising the /zero_fingers service");

  ROS_INFO("Entering main loop...");
  while (!g_done) {
    ros::spinOnce();
    if (!rh.listen(0.001))
      ROS_ERROR("Error in listen");
  }

  rh.setServoControlModes(reflex_hand::ReflexHand::CM_IDLE);
  ros::Duration(0.01).sleep();
  ROS_INFO("Have a nice day");
  return 0;
}
