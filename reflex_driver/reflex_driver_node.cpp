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

#include <ros/ros.h>
#include <algorithm>
#include <fstream>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <std_srvs/Empty.h>
#include "reflex_hand.h"
#include <reflex_msgs/RawServoPositions.h>
#include <reflex_msgs/RadianServoPositions.h>
#include <reflex_msgs/Hand.h>
#include <reflex_msgs/MotorDebug.h>
#include <reflex_msgs/StateDebug.h>
using namespace std;


ros::Publisher hand_pub;
ros::Publisher debug_pub;
ros::Publisher raw_pub;
ofstream tactile_file;
string tactile_file_address;
ofstream finger_file;
string finger_file_address;
bool aqcuire_tactile, aqcuire_fingers, first_capture, last_capture = false;
bool g_done = false;
vector<int> motor_inversion;
vector<double> dyn_zero;
vector<double> enc_zero;
vector<double> dyn_ratio;
int contact_threshold;
vector<int> tactile_offset_f1;
vector<int> tactile_offset_f2;
vector<int> tactile_offset_f3;
vector<int> tactile_offset_palm;
int encoder_last_value[] = {0, 0, 0};
// The encoders wrap around at 0-16383. This value stores the offset at each wrap
int encoder_offset[] = {-1, -1, -1};
// The encoder error at which the fingers consider themself zeroed during calibration
float ZERO_ERROR = 0.05;
int MOTOR_CAL_MIN = 1200;
int REVERSE_MOTOR_CAL_MIN = 2900;
double calibration_error[] = {0.0, 0.0, 0.0};

void signal_handler(int signum)
{
  if (signum == SIGINT || signum == SIGTERM)
    g_done = true;
}


// Loads necessary parameters
void load_params(ros::NodeHandle nh)
{
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
  if (topic != "no error")
  {
    ROS_FATAL("Failed to load %s parameter", topic.c_str());
    ROS_FATAL("This is likely because the corresponding yaml file in reflex_driver/yaml");
    ROS_FATAL("were never loaded, or it's because they have been corrupted. If they were");
    ROS_FATAL("corrupted, they can be repaired by pasting the *_backup.yaml file text in");
  }
  ROS_INFO("Loaded all parameters");
}


// Takes raw Dynamixel values (0-4095) and writes them directly to the motors
void set_raw_positions_cb(reflex_hand::ReflexHand *rh, const reflex_msgs::RawServoPositions::ConstPtr &msg)
{
  uint16_t targets[4];
  for (int i = 0; i < reflex_hand::ReflexHand::NUM_SERVOS; i++) {
    targets[i] = msg->raw_positions[i];
  }
  rh->setServoTargets(targets);
}


// Commands the motors from radians, using the zero references from
// yaml/finger_calibrate.yaml to translate into the raw Dyanmixel values
void set_radian_positions_cb(reflex_hand::ReflexHand *rh, const reflex_msgs::RadianServoPositions::ConstPtr &msg)
{
  uint16_t targets[4];
  for (int i = 0; i < reflex_hand::ReflexHand::NUM_SERVOS; i++) {
    targets[i] = (motor_inversion[i]*msg->radian_positions[i] + dyn_zero[i]) * (dyn_ratio[i]/reflex_hand::ReflexHand::DYN_SCALE);
    if (targets[i] > reflex_hand::ReflexHand::DYN_MIN_RAW_WRAPPED) {
      ROS_WARN("Finger %d set out of range (%d), reset to %d", i+1,
               (int16_t) targets[i], reflex_hand::ReflexHand::DYN_MIN_RAW);
      targets[i] = reflex_hand::ReflexHand::DYN_MIN_RAW;
    }
    else if (targets[i] > reflex_hand::ReflexHand::DYN_MAX_RAW) {
      ROS_WARN("Finger %d set out of range (%d), reset to %d", i+1,
               targets[i], reflex_hand::ReflexHand::DYN_MAX_RAW);
      targets[i] = reflex_hand::ReflexHand::DYN_MAX_RAW;
    }
  }
  rh->setServoTargets(targets);
}


void reflex_hand_state_cb(const reflex_hand::ReflexHandState * const state)
{
  int val = 0;

  // Sets and publishes the reflex_msgs/Hand message
  reflex_msgs::Hand hand_msg;
  int pressure_offset;
  // The dynamixel for Finger 1 has index 0, Finger 2 has index 1, Finger 3 has index 3
  for (int i = 0; i < reflex_hand::ReflexHandState::NUM_FINGERS; i++)
  {
    // TODO: Remove these if statements when the firmware order is fixed
    if (i == 0) val = 0;
    else if (i == 1) val = 2*9;
    else val = 1*9;
    const int tactile_base_idx = val;  
    // const int tactile_base_idx = i * 9;
    // TODO: Remove the (2-i) statement when the firmware order is fixed
    // Checks whether the encoder value has wrapped around and corrects for it
    if (encoder_offset[i] == -1)
      encoder_offset[i] = 0;
    else
    {
      if (encoder_last_value[i] - state->encoders_[2-i] > 5000)
        encoder_offset[i] = encoder_offset[i] + 16383;
      else if (encoder_last_value[i] - state->encoders_[2-i] < -5000)
        encoder_offset[i] = encoder_offset[i] - 16383;
    }
    encoder_last_value[i] = state->encoders_[2-i];
    hand_msg.finger[i].proximal = ((state->encoders_[2-i] + encoder_offset[i])*reflex_hand::ReflexHand::ENC_SCALE - enc_zero[i]);
    hand_msg.finger[i].spool = motor_inversion[i]*((state->dynamixel_angles_[i] * reflex_hand::ReflexHand::DYN_SCALE / dyn_ratio[i]) - dyn_zero[i]);
    double diff = hand_msg.finger[i].spool - hand_msg.finger[i].proximal;
    hand_msg.finger[i].distal = (diff<0)?0:diff;

    for (int j=0; j < 9; j++)
    {
      // This if statement is really messy, but I couldn't find a way to read the pressure
      // offset values in as a 3D vector. Probably possible, skipped for now
      if (i == 0)
        pressure_offset = tactile_offset_f1[j];
      else if (i == 1)
        pressure_offset = tactile_offset_f2[j];
      else
        pressure_offset = tactile_offset_f3[j];
      hand_msg.finger[i].pressure[j] = state->tactile_pressures_[tactile_base_idx + j] - pressure_offset;
      hand_msg.finger[i].contact[j] = false;
      if (abs(hand_msg.finger[i].pressure[j]) > contact_threshold)
        hand_msg.finger[i].contact[j] = true;
    }
  }
  hand_msg.palm.preshape = motor_inversion[3]*((state->dynamixel_angles_[3] * reflex_hand::ReflexHand::DYN_SCALE / dyn_ratio[3]) - dyn_zero[3]);
  const int palm_tactile_base_idx = reflex_hand::ReflexHandState::NUM_FINGERS * 9;
  for (int i=0; i < 11; i++)
  {
      hand_msg.palm.pressure[i] = state->tactile_pressures_[palm_tactile_base_idx + i] - tactile_offset_palm[i];
      hand_msg.palm.contact[i] = false;
      if (abs(hand_msg.palm.pressure[i]) > contact_threshold)
        hand_msg.palm.contact[i] = true;
  }
  hand_msg.joints_publishing = true;
  hand_msg.tactile_publishing = true;
  
  hand_pub.publish(hand_msg);

  // Capture the current tactile data and save it as a zero reference
  if (aqcuire_tactile)
  {
    tactile_file.open(tactile_file_address.c_str(), ios::out|ios::trunc);
    tactile_file << "# Captured sensor values from unloaded state, used for calibration \n";
    for (int i = 0; i < reflex_hand::ReflexHandState::NUM_FINGERS; i++)
    {
      if (i == 0) val = 0;
      else if (i == 1) val = 2*9;
      else val = 1*9;
      const int tactile_base_idx = val;  
      // const int tactile_base_idx = i * 9;
      // Write to variable
      for (int j = 0; j < 9; j++)
      {
        if (i == 0)
          tactile_offset_f1[j] = state->tactile_pressures_[tactile_base_idx + j];
        else if (i == 1)
          tactile_offset_f2[j] = state->tactile_pressures_[tactile_base_idx + j];
        else
          tactile_offset_f3[j] = state->tactile_pressures_[tactile_base_idx + j];
      }

      // Write to file
      tactile_file << "tactile_offset_f" << i+1 << ": ["
                          << state->tactile_pressures_[tactile_base_idx + 0] << ", "
                          << state->tactile_pressures_[tactile_base_idx + 1] << ", "
                          << state->tactile_pressures_[tactile_base_idx + 2] << ", "
                          << state->tactile_pressures_[tactile_base_idx + 3] << ", "
                          << state->tactile_pressures_[tactile_base_idx + 4] << ", "
                          << state->tactile_pressures_[tactile_base_idx + 5] << ", "
                          << state->tactile_pressures_[tactile_base_idx + 6] << ", "
                          << state->tactile_pressures_[tactile_base_idx + 7] << ", "
                          << state->tactile_pressures_[tactile_base_idx + 8] << "]\n";
    }
    // Write to variable
    for (int j = 0; j < 11; j++)
      tactile_offset_palm[j] = state->tactile_pressures_[palm_tactile_base_idx + j];

    // Write to file
    tactile_file << "tactile_offset_palm: ["
                          << state->tactile_pressures_[palm_tactile_base_idx + 0]<< ", "
                          << state->tactile_pressures_[palm_tactile_base_idx + 1]<< ", "
                          << state->tactile_pressures_[palm_tactile_base_idx + 2]<< ", "
                          << state->tactile_pressures_[palm_tactile_base_idx + 3]<< ", "
                          << state->tactile_pressures_[palm_tactile_base_idx + 4]<< ", "
                          << state->tactile_pressures_[palm_tactile_base_idx + 5]<< ", "
                          << state->tactile_pressures_[palm_tactile_base_idx + 6]<< ", "
                          << state->tactile_pressures_[palm_tactile_base_idx + 7]<< ", "
                          << state->tactile_pressures_[palm_tactile_base_idx + 8]<< ", "
                          << state->tactile_pressures_[palm_tactile_base_idx + 9]<< ", "
                          << state->tactile_pressures_[palm_tactile_base_idx + 10] << "]\n";

    aqcuire_tactile = false;
    tactile_file.close();
  }

  if (aqcuire_fingers)
  {
    // Open the parameter file and capture the current encoder position
    if (first_capture)
    {
      finger_file.open(finger_file_address.c_str(), ios::out|ios::trunc);
      ROS_INFO("Capturing starter encoder positions");
      // Write to variable
      for (int i = 0; i < reflex_hand::ReflexHandState::NUM_FINGERS; i++)
        // TODO: Remove the (2-i) statement when the firmware order is fixed
        enc_zero[i] = state->encoders_[2-i] * reflex_hand::ReflexHand::ENC_SCALE;

      // Write to file
      finger_file << "# Calbration constants for joints [f1, f2, f3, preshape]\n";
      finger_file << "encoder_zero_reference: ["
                            << enc_zero[0] << ", "
                            << enc_zero[1] << ", "
                            << enc_zero[2] << "]\n";
      first_capture = false;
    }

    // Check whether the fingers have moved and set the next movement if they haven't
    uint16_t increase[] = {20, 20, 50, 0}; // dynamixel step in counts out of 4095
    last_capture = true;
    for (int i = 0; i < reflex_hand::ReflexHandState::NUM_FINGERS; i++)
    {
      // TODO: Remove the (2-i) statement when the firmware order is fixed
      ROS_INFO("Finger %d\tenc_zero: %4f\tEncoder:%4f\tSpool: %4f",
              i+1, enc_zero[i], state->encoders_[2-i]*reflex_hand::ReflexHand::ENC_SCALE,
              state->dynamixel_angles_[i]*reflex_hand::ReflexHand::DYN_SCALE);
      if (abs(enc_zero[i] - state->encoders_[2-i]*reflex_hand::ReflexHand::ENC_SCALE) > ZERO_ERROR)
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
    for (int i=0; i<4; i++)
    {
      servo_pos.raw_positions[i] = state->dynamixel_angles_[i] + motor_inversion[i]*increase[i];
      ROS_INFO("Published position: %d", servo_pos.raw_positions[i]);
    }
    raw_pub.publish(servo_pos);

    // If all fingers have moved, capture their position and write it to file
    if (last_capture)
    {
      ROS_INFO("FINISHED ZEROING: Finger movement detected, zeroing motors");
      // Write to variable
      int offset = 300;
      for (int i = 0; i<4; i++) {
        if (i == 3) offset = 0;
        dyn_zero[i] = (state->dynamixel_angles_[i] - (motor_inversion[i]*offset))*
                       reflex_hand::ReflexHand::DYN_SCALE / dyn_ratio[i];
        if (dyn_zero[i] > 4*3.1415)
        {
          ROS_WARN("Something went wrong in calibration - motor %d was set anomalously high", i+1);
          ROS_WARN("  Motor zero reference value: %4f radians (nothing should be over 2*pi)", dyn_zero[i]);
          ROS_WARN("  Try redoing the calibration, depowering/repowering if it repeats");
        }

        servo_pos.raw_positions[i] = state->dynamixel_angles_[i] - (motor_inversion[i]*offset);
      }

      for (int i=0; i<3; i++) {
        if ((motor_inversion[i] > 0 && servo_pos.raw_positions[i] > MOTOR_CAL_MIN) || 
            (motor_inversion[i] < 0 && servo_pos.raw_positions[i] < REVERSE_MOTOR_CAL_MIN)) {
          if (motor_inversion[i] > 0 && servo_pos.raw_positions[i]) {
            calibration_error[i] = (servo_pos.raw_positions[i] - MOTOR_CAL_MIN) *
                                    reflex_hand::ReflexHand::DYN_SCALE / dyn_ratio[i];
          } else {
            calibration_error[i] = (REVERSE_MOTOR_CAL_MIN - servo_pos.raw_positions[i]) *
                                    reflex_hand::ReflexHand::DYN_SCALE / dyn_ratio[i];
          }
          ROS_WARN("Finger %d has been calibrated so that it won't", i+1);
          ROS_WARN("  be able to fully close. Check - is the tendon");
          ROS_WARN("  out of the groove? If not, the finger will");
          ROS_WARN("  probably need to have the tendon re-strung.");
          ROS_WARN("  Finger %d has been calibrated %4f radians", i+1, calibration_error[i]);
          ROS_WARN("  out of bounds");
        }
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

  char buffer [10];
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

  // for (int i = 0; i < reflex_hand::ReflexHandState::NUM_FINGERS; i++)
  // {
  //   const int tactile_base_idx = i * 9;
  //   ROS_INFO("finger %d tactile pressures: "
  //            "%4u %4u %4u %4u %4u %4u %4u %4u %4u",
  //            i,
  //            state->tactile_pressures_[tactile_base_idx + 0],
  //            state->tactile_pressures_[tactile_base_idx + 1],
  //            state->tactile_pressures_[tactile_base_idx + 2],
  //            state->tactile_pressures_[tactile_base_idx + 3],
  //            state->tactile_pressures_[tactile_base_idx + 4],
  //            state->tactile_pressures_[tactile_base_idx + 5],
  //            state->tactile_pressures_[tactile_base_idx + 6],
  //            state->tactile_pressures_[tactile_base_idx + 7],
  //            state->tactile_pressures_[tactile_base_idx + 8]);
  // }
  // ROS_INFO("palm tactile pressures: "
  //          "%4u %4u %4u %4u %4u %4u %4u %4u %4u %4u %4u",
  //          state->tactile_pressures_[palm_tactile_base_idx + 0],
  //          state->tactile_pressures_[palm_tactile_base_idx + 1],
  //          state->tactile_pressures_[palm_tactile_base_idx + 2],
  //          state->tactile_pressures_[palm_tactile_base_idx + 3],
  //          state->tactile_pressures_[palm_tactile_base_idx + 4],
  //          state->tactile_pressures_[palm_tactile_base_idx + 5],
  //          state->tactile_pressures_[palm_tactile_base_idx + 6],
  //          state->tactile_pressures_[palm_tactile_base_idx + 7],
  //          state->tactile_pressures_[palm_tactile_base_idx + 8],
  //          state->tactile_pressures_[palm_tactile_base_idx + 9],
  //          state->tactile_pressures_[palm_tactile_base_idx + 10]);

  return;
}


// Sets the procedure to calibrate the tactile values in motion
// Actual capturing is done in reflex_hand_state_cb
bool zero_tactile(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  aqcuire_tactile = true;
  ROS_INFO("Zeroing tactile data at current values...");
  ROS_INFO("tactile_file_address: %s", tactile_file_address.c_str());
  return true;
}


// Sets the procedure to calibrate the fingers in motion
// Actual capturing is done in reflex_hand_state_cb
bool zero_fingers(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  aqcuire_fingers = true;
  first_capture = true;
  last_capture = false;
  ROS_INFO("Beginning finger zero sequence...");
  ROS_INFO("finger_file_address: %s", finger_file_address.c_str());
  printf("This process assumes the fingers zeroed (opened as much as possible)\n");
  printf("and that the hand in aligned in a cylindrical grasp. The preshape joint\n");
  printf("will be set to zero just as it is now. If you don't like this position,\n");
  printf("reset and rer;un the calibration after this is finished...\n");
  return true;
}


int main(int argc, char **argv)
{
  // Initialize ROS node 
  ros::init(argc, argv, "reflex_hand_driver");
  ros::NodeHandle nh, nh_private("~");
  load_params(nh);

  // Advertising necessary topics
  hand_pub = nh.advertise<reflex_msgs::Hand>("/reflex_hand", 10);
  ROS_INFO("Advertising the /reflex_hand topic");
  debug_pub = nh.advertise<reflex_msgs::MotorDebug>("/reflex/motor_debug", 10);
  ROS_INFO("Advertising the /reflex/motor_debug topic");
  raw_pub = nh.advertise<reflex_msgs::RawServoPositions>("/set_reflex_raw", 1);

  // Intializes the reflex_hand object
  string network_interface;
  nh_private.param<string>("network_interface", network_interface, "eth0");
  ROS_INFO("starting reflex_hand_driver on network interface %s", network_interface.c_str());
  reflex_hand::ReflexHand rh(network_interface);
  if (!rh.happy())
  {
    ROS_FATAL("error during initialization. bailing now. have a nice day.");
    return 1;
  }
  rh.setStateCallback(reflex_hand_state_cb);
  rh.setServoControlModes(reflex_hand::ReflexHand::CM_POSITION);
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);
  
  // Subscribing to the hand command topics
  ros::Subscriber raw_positions_sub = nh.subscribe<reflex_msgs::RawServoPositions>("set_reflex_raw",
                    1, boost::bind(set_raw_positions_cb, &rh, _1));
  ros::Subscriber radian_positions_sub = nh.subscribe<reflex_msgs::RadianServoPositions>("set_reflex_hand",
                    1, boost::bind(set_radian_positions_cb, &rh, _1));
  
  // Initializing the /zero_tactile and /zero_finger services
  string buffer;
  nh.getParam("yaml_dir", buffer);
  tactile_file_address = buffer + "/tactile_calibrate.yaml";
  finger_file_address = buffer + "/finger_calibrate.yaml";
  ros::ServiceServer zero_tactile_srv = nh.advertiseService("/zero_tactile", zero_tactile);
  ROS_INFO("Advertising the /zero_tactile service");
  ros::ServiceServer zero_fingers_srv = nh.advertiseService("/zero_fingers", zero_fingers);
  ROS_INFO("Advertising the /zero_fingers service");

  ROS_INFO("entering main loop...");
  while (!g_done)
  {
    ros::spinOnce();
    if (!rh.listen(0.001))
      ROS_ERROR("error in listen");
  }
  rh.setServoControlModes(reflex_hand::ReflexHand::CM_IDLE);
  ros::Duration(0.01).sleep();
  ROS_INFO("have a nice day");
  return 0;
}
