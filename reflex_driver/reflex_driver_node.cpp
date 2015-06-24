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


#include <reflex_msgs/RawServoPositions.h>
#include <reflex_msgs/RadianServoPositions.h>
#include <reflex_msgs/Hand.h>
#include <reflex_msgs/MotorDebug.h>
#include <reflex_msgs/StateDebug.h>
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
using namespace std;


#define CAL_ERROR 0.05  // Encoder delta signifying movement in calibration


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
vector<int> tactile_offset_palm;
int tactile_base_idx[] = {0, 18, 9};
int encoder_last_value[] = {0, 0, 0};
int encoder_offset[] = {-1, -1, -1};

bool aqcuire_tactile, aqcuire_fingers, first_capture, last_capture = false;
vector<double> dyn_zero;
vector<double> enc_zero;
double calibration_error[] = {0.0, 0.0, 0.0};

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
  if (!nh.getParam("tactile_offset_palm", tactile_offset_palm))
    topic = "tactile_offset_palm";
  if (topic != "no error") {
    ROS_FATAL("Failed to load %s parameter", topic.c_str());
    ROS_FATAL("This is likely because the corresponding yaml file in");
    ROS_FATAL("reflex_driver/yaml were never loaded, or it's because");
    ROS_FATAL("they have been corrupted. If they were corrupted, they");
    ROS_FATAL("can be repaired by pasting the *_backup.yaml file text in");
  }
  ROS_INFO("Succesfully loaded all parameters");
}

int pressure_offset(int i, int j) {
  if (i == 0)
    return tactile_offset_f1[j];
  else if (i == 1)
    return tactile_offset_f2[j];
  else
    return tactile_offset_f3[j];
}


// Takes raw Dynamixel values (0-4095) and writes them directly to the motors
void set_raw_positions_cb(reflex_hand::ReflexHand *rh,
                          const reflex_msgs::RawServoPositions::ConstPtr &msg) {
  uint16_t targets[4];
  for (int i = 0; i < reflex_hand::ReflexHand::NUM_SERVOS; i++) {
    targets[i] = msg->raw_positions[i];
  }
  rh->setServoTargets(targets);
}


// Commands the motors from radians, using the zero references from
// yaml/finger_calibrate.yaml to translate into the raw Dynamixel values
void set_radian_positions_cb(reflex_hand::ReflexHand *rh,
                             const reflex_msgs::RadianServoPositions::ConstPtr &msg) {
  uint16_t targets[4];
  for (int i = 0; i < reflex_hand::ReflexHand::NUM_SERVOS; i++) {
    targets[i] = (motor_inversion[i]*msg->radian_positions[i] + dyn_zero[i]) *
                 (dyn_ratio[i]/reflex_hand::ReflexHand::DYN_SCALE);
    if (targets[i] > reflex_hand::ReflexHand::DYN_MIN_RAW_WRAPPED) {
      ROS_WARN("Finger %d set out of range (%d), reset to %d", i+1,
               (int16_t) targets[i], reflex_hand::ReflexHand::DYN_MIN_RAW);
      targets[i] = reflex_hand::ReflexHand::DYN_MIN_RAW;
    }
  }
  rh->setServoTargets(targets);
}


// Sets the procedure to calibrate the tactile values in motion
bool zero_tactile(std_srvs::Empty::Request &req,
                  std_srvs::Empty::Response &res) {
  aqcuire_tactile = true;
  ROS_INFO("Zeroing tactile data at current values...");
  ROS_INFO("tactile_file_address: %s", tactile_file_address.c_str());
  return true;
}


// Sets the procedure to calibrate the fingers in motion
bool zero_fingers(std_srvs::Empty::Request &req,
                  std_srvs::Empty::Response &res) {
  ROS_INFO("Beginning finger zero sequence...");
  ROS_INFO("finger_file_address: %s", finger_file_address.c_str());
  printf("This process assumes the fingers zeroed (opened as much as\n");
  printf("and that the hand in aligned in a cylindrical grasp. The\n");
  printf("preshape jointwill be set to zero just as it is now. If\n");
  printf("reset and rerun the calibration after this is finished...\n");
  aqcuire_fingers = true;
  first_capture = true;
  last_capture = false;
  return true;
}


void reflex_hand_state_cb(const reflex_hand::ReflexHandState * const state) {
  reflex_msgs::Hand hand_msg;

  for (int i = 0; i < reflex_hand::ReflexHandState::NUM_FINGERS; i++) {
    // Checks whether the encoder value has wrapped around and corrects for it
    if (encoder_offset[i] == -1) {
      encoder_offset[i] = 0;
    } else {
// TODO(Eric): Can we get rid of 2-i?
      if (encoder_last_value[i] - state->encoders_[2-i] > 5000)
      // if (encoder_last_value[i] - state->encoders_[i] > 5000)
        encoder_offset[i] = encoder_offset[i] + 16383;
      else if (encoder_last_value[i] - state->encoders_[2-i] < -5000)
      // else if (encoder_last_value[i] - state->encoders_[i] < -5000)
        encoder_offset[i] = encoder_offset[i] - 16383;
    }
    encoder_last_value[i] = state->encoders_[2-i];
    // encoder_last_value[i] = state->encoders_[i];

    hand_msg.finger[i].proximal =
      // (state->encoders_[2-i] + encoder_offset[i]) *
      (state->encoders_[i] + encoder_offset[i]) *
      reflex_hand::ReflexHand::ENC_SCALE - enc_zero[i];
    hand_msg.finger[i].spool =
      motor_inversion[i] *
      ((state->dynamixel_angles_[i] *
        reflex_hand::ReflexHand::DYN_SCALE / dyn_ratio[i]) - dyn_zero[i]);
    double diff = hand_msg.finger[i].spool - hand_msg.finger[i].proximal;
    hand_msg.finger[i].distal = (diff < 0) ? 0 : diff;

    for (int j=0; j < 9; j++) {
      hand_msg.finger[i].pressure[j] =
        state->tactile_pressures_[tactile_base_idx[i] + j] - pressure_offset(i, j);
      hand_msg.finger[i].contact[j] =
        abs(hand_msg.finger[i].pressure[j]) > contact_threshold;
    }
  }
  hand_msg.palm.preshape =
    motor_inversion[3] *
    ((state->dynamixel_angles_[3] *
      reflex_hand::ReflexHand::DYN_SCALE / dyn_ratio[3]) - dyn_zero[3]);
  hand_msg.joints_publishing = true;
  hand_msg.tactile_publishing = true;

  hand_pub.publish(hand_msg);

  // Capture the current tactile data and save it as a zero reference
  if (aqcuire_tactile) {
    tactile_file.open(tactile_file_address.c_str(), ios::out|ios::trunc);
    tactile_file << "# Captured sensor values from unloaded state\n";
    for (int i = 0; i < reflex_hand::ReflexHandState::NUM_FINGERS; i++) {
      for (int j = 0; j < 9; j++) {
        if (i == 0)
          tactile_offset_f1[j] = state->tactile_pressures_[tactile_base_idx[i] + j];
        else if (i == 1)
          tactile_offset_f2[j] = state->tactile_pressures_[tactile_base_idx[i] + j];
        else
          tactile_offset_f3[j] = state->tactile_pressures_[tactile_base_idx[i] + j];
      }

      // Write to file
      tactile_file << "tactile_offset_f" << i+1 << ": ["
                   << state->tactile_pressures_[tactile_base_idx[i] + 0] << ", "
                   << state->tactile_pressures_[tactile_base_idx[i] + 1] << ", "
                   << state->tactile_pressures_[tactile_base_idx[i] + 2] << ", "
                   << state->tactile_pressures_[tactile_base_idx[i] + 3] << ", "
                   << state->tactile_pressures_[tactile_base_idx[i] + 4] << ", "
                   << state->tactile_pressures_[tactile_base_idx[i] + 5] << ", "
                   << state->tactile_pressures_[tactile_base_idx[i] + 6] << ", "
                   << state->tactile_pressures_[tactile_base_idx[i] + 7] << ", "
                   << state->tactile_pressures_[tactile_base_idx[i] + 8] << "]\n";
    }

    aqcuire_tactile = false;
    tactile_file.close();
  }

  if (aqcuire_fingers) {
    // Open the parameter file and capture the current encoder position
    if (first_capture) {
      finger_file.open(finger_file_address.c_str(), ios::out|ios::trunc);
      ROS_INFO("Capturing starter encoder positions");
      // Write to variable
      for (int i = 0; i < reflex_hand::ReflexHandState::NUM_FINGERS; i++)
        // TODO(Eric): Remove the (2-i) statement when firmware order is fixed
        enc_zero[i] = state->encoders_[2-i] *
                      reflex_hand::ReflexHand::ENC_SCALE;

      // Write to file
      finger_file << "# Calbration constants for [f1, f2, f3, preshape]\n";
      finger_file << "encoder_zero_reference: ["
                            << enc_zero[0] << ", "
                            << enc_zero[1] << ", "
                            << enc_zero[2] << "]\n";
      first_capture = false;
    }

    // Check whether the fingers have moved, set the next movement if not
    uint16_t increase[] = {5, 5, 5, 0};  // Dynamixel step
    last_capture = true;
    for (int i = 0; i < reflex_hand::ReflexHandState::NUM_FINGERS; i++) {
      // TODO(Eric): Remove the (2-i) statement when the firmware order is fixed
      ROS_INFO("Finger %d\tenc_zero: %4f\tEncoder:%4f\tSpool: %4f",
              i+1, enc_zero[i],
              state->encoders_[2-i]*reflex_hand::ReflexHand::ENC_SCALE,
              state->dynamixel_angles_[i]*reflex_hand::ReflexHand::DYN_SCALE);
      if (abs(enc_zero[i] - state->encoders_[2-i]*reflex_hand::ReflexHand::ENC_SCALE) > CAL_ERROR)
        increase[i] = 0;
      else
        last_capture = false;
    }
    ROS_INFO("Palm Spool: %4f",
              state->dynamixel_angles_[3]*reflex_hand::ReflexHand::DYN_SCALE);

    // Move the fingers in
    ROS_INFO("Stepping the fingers inwards:\t%d+%d\t%d+%d\t%d+%d\t%d+%d",
          state->dynamixel_angles_[0], increase[0],
          state->dynamixel_angles_[1], increase[1],
          state->dynamixel_angles_[2], increase[2],
          state->dynamixel_angles_[3], increase[3]);
    reflex_msgs::RawServoPositions servo_pos;
    for (int i=0; i < 4; i++) {
      servo_pos.raw_positions[i] = state->dynamixel_angles_[i] +
                                   motor_inversion[i]*increase[i];
      ROS_INFO("Published position: %d", servo_pos.raw_positions[i]);
    }
    raw_pub.publish(servo_pos);

    // If all fingers have moved, capture their position and write it to file
    if (last_capture) {
      ROS_INFO("FINISHED ZEROING: Finger movement detected, zeroing motors");
      // Write to variable
      int offset = 50;
      for (int i = 0; i < 4; i++) {
        if (i == 3) offset = 0;
        dyn_zero[i] = (state->dynamixel_angles_[i] - (motor_inversion[i]*offset))*
                       reflex_hand::ReflexHand::DYN_SCALE / dyn_ratio[i];
        if (dyn_zero[i] > 14 * 3.1415) {
          ROS_WARN("Something went wrong in calibration - motor %d was", i+1);
          ROS_WARN("  set anomalously high. Motor zero reference value:");
          ROS_WARN("  %4f radians (nothing should be over 2*pi)", dyn_zero[i]);
          ROS_WARN("  Try redoing the calibration, depowering/repowering");
          ROS_WARN("  if it repeats");
        }

        servo_pos.raw_positions[i] = state->dynamixel_angles_[i] -
                                     (motor_inversion[i]*offset);
      }

      // Write to file
      finger_file << "motor_zero_reference: ["
                            << dyn_zero[0] << ", "
                            << dyn_zero[1] << ", "
                            << dyn_zero[2] << ", "
                            << dyn_zero[3] << "]\n";
      aqcuire_fingers = false;
      finger_file.close();
      raw_pub.publish(servo_pos);
    }
  }

  char buffer[10];
  reflex_msgs::MotorDebug debug_msg;
  for (int i = 0; i < reflex_hand::ReflexHandState::NUM_FINGERS; i++) {
    debug_msg.encoder[i] = state->encoders_[i];
  }
  for (int i = 0; i < reflex_hand::ReflexHand::NUM_SERVOS; i++) {
    debug_msg.motor[i].raw_angle = state->dynamixel_angles_[i];
    debug_msg.motor[i].speed = state->dynamixel_speeds_[i];
    debug_msg.motor[i].load = state->dynamixel_loads_[i];
    debug_msg.motor[i].voltage = state->dynamixel_voltages_[i];
    debug_msg.motor[i].temperature = state->dynamixel_temperatures_[i];
    sprintf(buffer, "0x%02x", state->dynamixel_error_states_[i]);
    debug_msg.motor[i].error_state = buffer;
  }
  debug_pub.publish(debug_msg);

  return;
}


int main(int argc, char **argv) {
  // Initialize ROS node
  ros::init(argc, argv, "reflex_hand_driver");
  ros::NodeHandle nh, nh_private("~");
  load_params(nh);

  // Advertise necessary topics
  hand_pub = nh.advertise<reflex_msgs::Hand>("/reflex_hand", 10);
  ROS_INFO("Advertising the /reflex_hand topic");
  debug_pub = nh.advertise<reflex_msgs::MotorDebug>("/reflex/motor_debug", 10);
  ROS_INFO("Advertising the /reflex/motor_debug topic");
  raw_pub = nh.advertise<reflex_msgs::RawServoPositions>("/set_reflex_raw", 1);

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
    nh.subscribe<reflex_msgs::RawServoPositions>("set_reflex_raw",
                 1, boost::bind(set_raw_positions_cb, &rh, _1));
  ros::Subscriber radian_positions_sub =
    nh.subscribe<reflex_msgs::RadianServoPositions>("set_reflex_hand",
                 1, boost::bind(set_radian_positions_cb, &rh, _1));

  // Initialize the /zero_tactile and /zero_finger services
  string buffer;
  nh.getParam("yaml_dir", buffer);
  tactile_file_address = buffer + "/tactile_calibrate.yaml";
  finger_file_address = buffer + "/finger_calibrate.yaml";
  ros::ServiceServer zero_tactile_srv =
    nh.advertiseService("/zero_tactile", zero_tactile);
  ROS_INFO("Advertising the /zero_tactile service");
  ros::ServiceServer zero_fingers_srv =
    nh.advertiseService("/zero_fingers", zero_fingers);
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
