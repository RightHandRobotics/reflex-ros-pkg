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


#include <algorithm>
#include <boost/bind.hpp>
#include <ros/console.h>

#include "./reflex_hand.h"
#include "./reflex_driver.h"

using namespace reflex_driver;


// Public exposure by this driver
// Published topics
//      /hand_state             Contains the current state of the hand
// Subscribed topics
//      /radian_hand_command    Executes radian commands, assuming hand is calibrated
// Advertised services
//      /calibrate_tactile      Calibrates tactile sensors at current level
//      /calibrate_fingers      Calibrates encoder and motor values
//      /disable_torque         Disables finger torque
//      /set_tactile_threshold  Sets threshold for "contact" for each sensor individually


const int ReflexDriver::TACTILE_BASE_IDX[NUM_TAKKTILE] = {0, 18, 9};
const uint16_t ReflexDriver::CALIBRATION_DYN_OFFSET[NUM_DYNAMIXELS] = {50, 50, 50, 0};

ReflexDriver::ReflexDriver(const std::string &network_interface, const int pb, const std::string &hand_prefix, const char* mcast_addr_str): rh(network_interface, pb, mcast_addr_str) {
  
  ROS_INFO("Starting ReflexDriver constructor");
  ROS_INFO("checking interface reflex_driver_node: %s", network_interface.c_str());

  hand = hand_prefix;
  init_params();

  ROS_INFO("Initialized local params");

  ros::NodeHandle nh, nh_private("~");
  load_params(nh);
  populate_tactile_threshold(default_contact_threshold);

  std::string ns;// = "/" + hand_prefix + "/reflex_takktile"; // TODO: document that hand_prefix should not start with /
  if (hand_prefix == "hand1"){
    ns = "/reflex_takktile";
  }
  else{
    ns = "/" + hand_prefix + "/reflex_takktile"; // TODO: document that hand_prefix should not start with /
  } 

  // Advertise necessary topics
  hand_pub = nh.advertise<reflex_msgs::Hand>(ns + "/hand_state", 10);
  ROS_INFO("Publishing the %s/hand_state topic", ns.c_str());

  raw_pub = nh.advertise<reflex_msgs::RawServoCommands>(ns + "/raw_hand_command", 1);
  ROS_INFO("Publishing the %s/raw_hand_command topic", ns.c_str());

  // Intialize the reflex_hand object
  ROS_INFO("Starting %s reflex_hand_driver on network interface %s",
           hand_prefix.c_str(), network_interface.c_str());
  
  if (!rh.happy()) {
    ROS_FATAL("Error during initialization. bailing now. have a nice day.");
    ros::shutdown();
  }
  rh.setStateCallback(boost::bind(&ReflexDriver::reflex_hand_state_cb, this, _1));
  rh.setServoControlModes(reflex_hand::ReflexHand::CM_POSITION);

  // // Initialize the /calibrate_tactile and /calibrate_fingers services
  std::string buffer;
  nh.getParam("yaml_dir", buffer);

  if (hand_prefix == "hand1"){
    finger_file_address = buffer + "/finger_calibrate.yaml";
    tactile_file_address = buffer + "/tactile_calibrate.yaml";
  }
  else{
    finger_file_address = buffer + "/" + hand_prefix + "_finger_calibrate.yaml";
    tactile_file_address = buffer + "/" + hand_prefix + "_tactile_calibrate.yaml";
  }
  latest_calibration_time = ros::Time::now();
}

void ReflexDriver::init_params() {
  for (int i = 0; i < NUM_ENCODERS; i++) {
    encoder_last_value[i] = 0;
    encoder_offset[i] = -1;
    calibration_dyn_increase[i] = 6;
  }

  for (int i = 0; i < NUM_DYNAMIXELS; i++) {
    load_last_value[i] = 0;
    raw_cmd_last_value[i] = 0;
  }

  calibration_dyn_increase[NUM_DYNAMIXELS-1] = 0;
}

void ReflexDriver::load_params(ros::NodeHandle nh) {
  std::string topic = "no error";
  if (!nh.getParam(hand + "/motor_zero_reference", dynamixel_zero_point))
    topic = hand + "/motor_zero_reference";
  if (!nh.getParam(hand + "/encoder_zero_reference", encoder_zero_point))
    topic = hand + "/encoder_zero_reference";
  if (!nh.getParam("motor_to_joint_inverted", MOTOR_TO_JOINT_INVERTED))
    topic = "motor_to_joint_inverted";
  if (!nh.getParam("motor_to_joint_gear_ratio", MOTOR_TO_JOINT_GEAR_RATIO))
    topic = "motor_to_joint_gear_ratio";
  if (!nh.getParam("default_contact_threshold", default_contact_threshold))
    topic = "default_contact_threshold";
  if (!nh.getParam(hand + "/tactile_offset_f1", tactile_offset_f1))
    topic = hand + "/tactile_offset_f1";
  if (!nh.getParam(hand + "/tactile_offset_f2", tactile_offset_f2))
    topic = hand + "/tactile_offset_f2";
  if (!nh.getParam(hand + "/tactile_offset_f3", tactile_offset_f3))
    topic = hand + "/tactile_offset_f3";
  if (topic != "no error") {
    ROS_FATAL("Failed to load %s parameter", topic.c_str());
    ROS_FATAL("This is likely because the corresponding yaml file in");
    ROS_FATAL("reflex_driver/yaml were never loaded, or because they");
    ROS_FATAL("have been corrupted. If they were corrupted, they can");
    ROS_FATAL("be repaired by pasting the *_backup.yaml file text in");
    ros::shutdown();
  }
  ROS_INFO("Succesfully loaded all parameters");
}


// Takes a rad command and returns Dynamixel command
uint16_t ReflexDriver::pos_rad_to_raw(float rad_command, int motor_idx) {
  float zeroed_command = MOTOR_TO_JOINT_INVERTED[motor_idx] * rad_command + dynamixel_zero_point[motor_idx];
  float motor_ratio = (MOTOR_TO_JOINT_GEAR_RATIO[motor_idx] / reflex_hand::ReflexHand::DYN_POS_SCALE);
  uint16_t command = (uint16_t) (zeroed_command * motor_ratio);
  if (command > reflex_hand::ReflexHand::DYN_MIN_RAW_WRAPPED) {
    ROS_WARN("Finger %d set out of range (%d), reset to %d", motor_idx + 1,
             (int16_t) command, reflex_hand::ReflexHand::DYN_MIN_RAW);
    command = reflex_hand::ReflexHand::DYN_MIN_RAW;
  }
  return command;
}


// Takes a rad/s command and returns Dynamixel command
//     http://support.robotis.com/en/product/dynamixel/mx_series/mx-28.htm#Actuator_Address_20
uint16_t ReflexDriver::speed_rad_to_raw(float rad_per_s_command, int motor_idx) {
  uint16_t command = abs(rad_per_s_command) *
                     (MOTOR_TO_JOINT_GEAR_RATIO[motor_idx] / reflex_hand::ReflexHand::DYN_VEL_SCALE);
  if (command > 1023) {
    command = 1023;
  }
  if (MOTOR_TO_JOINT_INVERTED[motor_idx] * rad_per_s_command < 0) {
    command += 1024;
  }
  if (command == 0) {
    command = 1;  // 0 doesn't actually stop the motor
  }
  return command;
}


// Takes the load, converts it to a float, then does a rolling filter
float ReflexDriver::load_raw_to_signed(int load, int motor_idx) {
  if (load > 1023) {
    load = (load - 1023);
  } else {
    load = -1 * load;
  }
  float signed_load = (float) (MOTOR_TO_JOINT_INVERTED[motor_idx] * load);
  float load_filter = 0.25;  // Rolling filter of noisy data
  float filter_load = load_filter * signed_load + (1 - load_filter) * load_last_value[motor_idx];
  load_last_value[motor_idx] = filter_load;
  return filter_load;
}


// Checks whether the motor positions will reset after mode switch, and corrects zero point
// When switching modes (VELOCITY and POSITION) the motor will wrap values if above 14024 or below 13000
void ReflexDriver::check_for_potential_motor_wraps_and_rezero() {
  double motor_wrap;
  for (int i = 0; i < NUM_SERVOS; i++) {
    motor_wrap = 1025 * (reflex_hand::ReflexHand::DYN_POS_SCALE / MOTOR_TO_JOINT_GEAR_RATIO[i]);
  }
}


// Steps the fingers in by a calibration amount
void ReflexDriver::move_fingers_in(const reflex_hand::ReflexHandState* const state) {
  reflex_msgs::RawServoCommands servo_pos;
  int motor_step;
  ROS_INFO("Step fingers inwards:\t%d+%d\t%d+%d\t%d+%d\t%d+%d",
           state->dynamixel_angles_[0], calibration_dyn_increase[0],
           state->dynamixel_angles_[1], calibration_dyn_increase[1],
           state->dynamixel_angles_[2], calibration_dyn_increase[2],
           state->dynamixel_angles_[3], calibration_dyn_increase[3]);
  for (int i=0; i < NUM_SERVOS; i++) {
    motor_step = MOTOR_TO_JOINT_INVERTED[i] * calibration_dyn_increase[i];
    servo_pos.raw_positions[i] = state->dynamixel_angles_[i] + motor_step;
  }
  raw_pub.publish(servo_pos);
}


// Sets the tactile threshold levels to be all one value
void ReflexDriver::populate_tactile_threshold(int threshold) {
  for (int i = 0; i < NUM_FINGERS; i++) {
    for (int j = 0; j < NUM_SENSORS_PER_FINGER; j++) {
      contact_thresholds.finger[i].sensor[j] = threshold;
    }
  }
}


// Sets the threshold levels on tactile sensors for reporting contact
bool ReflexDriver::set_tactile_threshold(reflex_msgs::SetTactileThreshold::Request &req,
                           reflex_msgs::SetTactileThreshold::Response &res) {
  contact_thresholds = req;
  return true;
}


// Changes the travel speed of the motor
bool ReflexDriver::set_motor_speed(reflex_msgs::SetSpeed::Request &req, reflex_msgs::SetSpeed::Response &res) {
  rh.setServoControlModes(reflex_hand::ReflexHand::CM_VELOCITY);
  ros::Duration(0.02).sleep();
  uint16_t targets[NUM_SERVOS];
  for (int i = 0; i < NUM_SERVOS; i++) {
    targets[i] = speed_rad_to_raw(req.motor[i], i);
  }
  rh.setServoTargets(targets);
  ros::Duration(0.035).sleep();  // Sleep necessary to prevent Brain Board freezing
  rh.setServoControlModes(reflex_hand::ReflexHand::CM_POSITION);
  check_for_potential_motor_wraps_and_rezero();
  ros::Duration(0.035).sleep();  // Sleep necessary to prevent Brain Board freezing
  return true;
}


// Puts the dynamixels in idle mode
bool ReflexDriver::disable_torque(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  rh.setServoControlModes(reflex_hand::ReflexHand::CM_IDLE);
  return true;
}


// Sets the procedure to calibrate the tactile values in motion
bool ReflexDriver::calibrate_tactile(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  acquire_tactile = true;
  ROS_INFO("Zeroing tactile data at current values...");
  return true;
}


// Sets the procedure to calibrate the fingers in motion
bool ReflexDriver::calibrate_fingers(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  ROS_INFO("Beginning finger calibration sequence...");
  acquire_fingers = true;
  first_capture = true;
  all_fingers_moved = false;
  return true;
}


// Returns the correct pressure calibration offset for finger[sensor]
int ReflexDriver::pressure_offset(int finger, int sensor) {
  if (finger == 0)
    return tactile_offset_f1[sensor];
  else if (finger == 1)
    return tactile_offset_f2[sensor];
  else
    return tactile_offset_f3[sensor];
}


// Given raw value and a past value, tracks encoder wraps: enc_offset variable
int ReflexDriver::update_encoder_offset(int raw_value, int last_value, int current_offset) {
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
float ReflexDriver::calc_proximal_angle(int raw_enc_value, int offset, double zero) {
  int wrapped_enc_value = raw_enc_value + offset;
  float rad_value = (float) wrapped_enc_value * reflex_hand::ReflexHand::ENC_SCALE;
  return (zero - rad_value);
}


// Calculates joint angle from raw sensor value, motor gear ratio,
// and calibrated "zero" point for the joint
float ReflexDriver::calc_motor_angle(int inversion, int raw_dyn_value, double ratio, double zero) {
  float rad_value = raw_dyn_value * reflex_hand::ReflexHand::DYN_POS_SCALE / ratio;
  float zeroed_value = rad_value - zero;
  return inversion * zeroed_value;
}


// Calculates distal angle, "tendon spooled out" - "proximal encoder" angles
// Could be improved
float ReflexDriver::calc_distal_angle(float joint_angle, float proximal) {
  float diff = joint_angle - proximal;
  return (diff < 0) ? 0 : diff;
}


// Takes hand state and returns calibrated tactile data for given finger
int ReflexDriver::calc_pressure(const reflex_hand::ReflexHandState* const state, int finger, int sensor) {
  int raw_value = state->tactile_pressures_[TACTILE_BASE_IDX[finger] + sensor];
  return raw_value - pressure_offset(finger, sensor);
}


// Checks given finger/sensor for contact threshold
int ReflexDriver::calc_contact(reflex_msgs::Hand hand_msg, int finger, int sensor) {
  int pressure_value = hand_msg.finger[finger].pressure[sensor];
  return pressure_value > contact_thresholds.finger[finger].sensor[sensor];
}


// Opens tactile calibration data file, changes local tactile_offset, and saves
// current tactile values to file as the new calibrated "zero"
void ReflexDriver::calibrate_tactile_sensors(const reflex_hand::ReflexHandState* const state,
                               reflex_msgs::Hand hand_msg) {
  tactile_file.open(tactile_file_address.c_str(), std::ios::out|std::ios::trunc);
  tactile_file << "# Captured sensor values from unloaded state\n";
  log_current_tactile_locally(state);
  for (int i = 0; i < NUM_FINGERS; i++) {
    log_current_tactile_to_file(state, i);
  }
  acquire_tactile = false;
  tactile_file.close();
}


// Capture current encoder position locally as "zero" and save to file
void ReflexDriver::calibrate_encoders_locally(const reflex_hand::ReflexHandState* const state) {
  for (int i = 0; i < NUM_FINGERS; i++) {
    encoder_zero_point[i] = state->encoders_[i] * reflex_hand::ReflexHand::ENC_SCALE;
    encoder_offset[i] = 0;
  }
}


// Save current dynamixel location (plus an offset) as "zero" and then
// write the dynamixels to the spot
void ReflexDriver::calibrate_motors_locally(const reflex_hand::ReflexHandState* const state) {
  reflex_msgs::RawServoCommands servo_pos;
  int motor_offset;
  float motor_scalar;

  for (int i = 0; i < NUM_SERVOS; i++) {
    motor_offset = MOTOR_TO_JOINT_INVERTED[i] * CALIBRATION_DYN_OFFSET[i];
    motor_scalar = reflex_hand::ReflexHand::DYN_POS_SCALE / MOTOR_TO_JOINT_GEAR_RATIO[i];
    dynamixel_zero_point[i] = (state->dynamixel_angles_[i] - motor_offset) * motor_scalar;
    servo_pos.raw_positions[i] = state->dynamixel_angles_[i] - motor_offset;
  }
  raw_pub.publish(servo_pos);
}


// Save local variables tactile_offset_f* with current tactile position
void ReflexDriver::log_current_tactile_locally(const reflex_hand::ReflexHandState* const state) {
  for (int j = 0; j < NUM_SENSORS_PER_FINGER; j++) {
    tactile_offset_f1[j] = state->tactile_pressures_[TACTILE_BASE_IDX[0] + j];
    tactile_offset_f2[j] = state->tactile_pressures_[TACTILE_BASE_IDX[1] + j];
    tactile_offset_f3[j] = state->tactile_pressures_[TACTILE_BASE_IDX[2] + j];
  }
}


// Writes the current tactile data to the data file
void ReflexDriver::log_current_tactile_to_file(const reflex_hand::ReflexHandState* const state,
                                 int finger) {
  tactile_file << "tactile_offset_f" << finger + 1 << ": ["
               << state->tactile_pressures_[TACTILE_BASE_IDX[finger] + 0] << ", "
               << state->tactile_pressures_[TACTILE_BASE_IDX[finger] + 1] << ", "
               << state->tactile_pressures_[TACTILE_BASE_IDX[finger] + 2] << ", "
               << state->tactile_pressures_[TACTILE_BASE_IDX[finger] + 3] << ", "
               << state->tactile_pressures_[TACTILE_BASE_IDX[finger] + 4] << ", "
               << state->tactile_pressures_[TACTILE_BASE_IDX[finger] + 5] << ", "
               << state->tactile_pressures_[TACTILE_BASE_IDX[finger] + 6] << ", "
               << state->tactile_pressures_[TACTILE_BASE_IDX[finger] + 7] << ", "
               << state->tactile_pressures_[TACTILE_BASE_IDX[finger] + 8] << "]\n";
}


// Opens data file and writes calibrated encoder zeros to it
void ReflexDriver::log_encoder_zero_to_file() {
  finger_file.open(finger_file_address.c_str(), std::ios::out|std::ios::trunc);
  finger_file << "# Calibration constants for [f1, f2, f3, preshape]\n";
  finger_file << "encoder_zero_reference: ["
              << encoder_zero_point[0] << ", "
              << encoder_zero_point[1] << ", "
              << encoder_zero_point[2] << "]\n";
}


// Writes calibrated dynamixel zeros to the data file and closes it
void ReflexDriver::log_motor_zero_to_file_and_close() {
  finger_file << "motor_zero_reference: ["
              << dynamixel_zero_point[0] << ", "
              << dynamixel_zero_point[1] << ", "
              << dynamixel_zero_point[2] << ", "
              << dynamixel_zero_point[3] << "]\n";
  finger_file.close();
}


// Checks whether all fingers have moved more than CALIBRATION_ERROR, halts
// the motion of fingers past that point
bool ReflexDriver::check_for_finger_movement(const reflex_hand::ReflexHandState* const state) {
  bool all_fingers_moved = true;
  for (int i = 0; i < NUM_FINGERS; i++) {
    float enc_pos = encoder_zero_point[i] -
                    state->encoders_[i] * reflex_hand::ReflexHand::ENC_SCALE;
    if (std::abs(enc_pos) > CALIBRATION_ERROR) {
      calibration_dyn_increase[i] = 0;
    } else {
      calibration_dyn_increase[i] = 6;
      all_fingers_moved = false;
    }
  }
  return all_fingers_moved;
}


// Captures the current hand state to a debug message and publishes
void ReflexDriver::populate_motor_state(reflex_msgs::Hand* hand_msg, const reflex_hand::ReflexHandState* const state) {
  char buffer[10];
  for (int i = 0; i < NUM_SERVOS; i++) {
    hand_msg->motor[i].raw_angle = (float) state->dynamixel_angles_[i];
    hand_msg->motor[i].velocity = (float) state->dynamixel_speeds_[i];
    hand_msg->motor[i].load = load_raw_to_signed(state->dynamixel_loads_[i], i);
    hand_msg->motor[i].voltage = (float) state->dynamixel_voltages_[i];
    hand_msg->motor[i].temperature = state->dynamixel_temperatures_[i];
    sprintf(buffer, "0x%02x", state->dynamixel_error_states_[i]);
    hand_msg->motor[i].error_state = buffer;
  }
}


// Takes raw Dynamixel values (0-4095) and writes them directly to the motors
// NOTE: The Dynamixels have a resolution divider of 4. See what that means here:
//     http://support.robotis.com/en/product/dynamixel/mx_series/mx-28.htm#Actuator_Address_0B1
void ReflexDriver::receive_raw_cmd_cb(const reflex_msgs::RawServoCommands::ConstPtr &msg) {
  uint16_t targets[NUM_SERVOS];
  for (int i = 0; i < NUM_SERVOS; i++) {
    targets[i] = msg->raw_positions[i];
    raw_cmd_last_value[i] = targets[i];
  }
  rh.setServoTargets(targets);
}


// Commands the motors from radians, using the zero references from
// yaml/finger_calibrate.yaml to translate into the raw Dynamixel values
void ReflexDriver::receive_angle_cmd_cb(const reflex_msgs::RadianServoCommands::ConstPtr &msg) {
  uint16_t targets[NUM_SERVOS];
  for (int i = 0; i < NUM_SERVOS; i++) {
    targets[i] = pos_rad_to_raw(msg->radian_commands[i], i);
    raw_cmd_last_value[i] = targets[i];
  }
  rh.setServoTargets(targets);
}


// Takes in hand state data and publishes to reflex_hand topic
// Also does calibration when certain booleans are enabled
void ReflexDriver::reflex_hand_state_cb(const reflex_hand::ReflexHandState * const state) {
  reflex_msgs::Hand hand_msg;

  for (int i = 0; i < NUM_FINGERS; i++) {
    encoder_offset[i] = update_encoder_offset(state->encoders_[i],
                                              encoder_last_value[i],
                                              encoder_offset[i]);
    encoder_last_value[i] = state->encoders_[i];
    hand_msg.motor[i].joint_angle = calc_motor_angle(MOTOR_TO_JOINT_INVERTED[i],
                                                     state->dynamixel_angles_[i],
                                                     MOTOR_TO_JOINT_GEAR_RATIO[i],
                                                     dynamixel_zero_point[i]);
    hand_msg.finger[i].proximal = calc_proximal_angle(state->encoders_[i],
                                                      encoder_offset[i],
                                                      encoder_zero_point[i]);
    hand_msg.finger[i].distal_approx = calc_distal_angle(hand_msg.motor[i].joint_angle,
                                                         hand_msg.finger[i].proximal);
    for (int j=0; j < NUM_SENSORS_PER_FINGER; j++) {
      hand_msg.finger[i].pressure[j] = calc_pressure(state, i, j);
      hand_msg.finger[i].contact[j] = calc_contact(hand_msg, i, j);
    }
  }
  hand_msg.motor[3].joint_angle = calc_motor_angle(MOTOR_TO_JOINT_INVERTED[3],
                                                   state->dynamixel_angles_[3],
                                                   MOTOR_TO_JOINT_GEAR_RATIO[3],
                                                   dynamixel_zero_point[3]);
  populate_motor_state(&hand_msg, state);
  hand_pub.publish(hand_msg);

  // Capture the current tactile data and save it as a zero reference
  if (acquire_tactile) {
    calibrate_tactile_sensors(state, hand_msg);
  }

  if (acquire_fingers && (ros::Time::now() > latest_calibration_time + ros::Duration(0.05))) {
    if (first_capture) {
      calibrate_encoders_locally(state);
      first_capture = false;
    }
    all_fingers_moved = check_for_finger_movement(state);
    if (all_fingers_moved) {
      acquire_fingers = false;
      ROS_INFO("FINISHED FINGER CALIBRATION: Encoder movement detected");
      calibrate_motors_locally(state);
      log_encoder_zero_to_file();
      log_motor_zero_to_file_and_close();
    } else {
      move_fingers_in(state);
    }
    latest_calibration_time = ros::Time::now();
  }
  return;
}
