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
#include <fstream>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <std_srvs/Empty.h>
#include "reflex_hand.h"
#include "reflex_hand/RawServoPositions.h"
#include "reflex_hand/RadianServoPositions.h"
#include <reflex_msgs/Hand.h>
using namespace std;


ros::Publisher hand_pub;
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


void signal_handler(int signum)
{
  if (signum == SIGINT || signum == SIGTERM)
    g_done = true;
}


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
  nh.param("contact_threshold", contact_threshold, 400);
  if (!nh.getParam("tactile_offset_f1", tactile_offset_f1))
    topic = "tactile_offset_f1";
  if (!nh.getParam("tactile_offset_f2", tactile_offset_f2))
    topic = "tactile_offset_f2";
  if (!nh.getParam("tactile_offset_f3", tactile_offset_f3))
    topic = "tactile_offset_f3";
  if (!nh.getParam("tactile_offset_palm", tactile_offset_palm))
    topic = "tactile_offset_palm";
  if (topic != "no error")
    ROS_FATAL("Failed to load %s parameter", topic.c_str());
  ROS_INFO("Loaded all parameters");
}


void set_raw_positions_cb(reflex_hand::ReflexHand *rh, const reflex_hand::RawServoPositions::ConstPtr &msg)
{
  uint16_t targets[4];
  for (int i = 0; i < 4; i++)
    targets[i] = msg->raw_positions[i];
  rh->setServoTargets(targets);
}


void set_radian_positions_cb(reflex_hand::ReflexHand *rh, const reflex_hand::RadianServoPositions::ConstPtr &msg)
{
  uint16_t targets[4];
  for (int i = 0; i < 4; i++) {
    targets[i] = (motor_inversion[i]*msg->radian_positions[i] + dyn_zero[i]) * (dyn_ratio[i]/reflex_hand::ReflexHand::DYN_SCALE);
  }
  rh->setServoTargets(targets);
}


void reflex_hand_state_cb(const reflex_hand::ReflexHandState * const state)
{
  // ROS_INFO("rx time: %d", state->systime_us_);
  // ROS_INFO("encoders: %6u %6u %6u", 
  //          state->encoders_[0],
  //          state->encoders_[1],
  //          state->encoders_[2]);
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
  // const int palm_tactile_base_idx = 
  //   reflex_hand::ReflexHandState::NUM_FINGERS * 9;
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
  // ROS_INFO("dynamixel error states: 0x%02x 0x%02x 0x%02x 0x%02x",
  //          state->dynamixel_error_states_[0],
  //          state->dynamixel_error_states_[1],
  //          state->dynamixel_error_states_[2],
  //          state->dynamixel_error_states_[3]);
    // ROS_INFO("dynamixel angles: %6u %6u %6u %6u",
    //          state->dynamixel_angles_[0],
    //          state->dynamixel_angles_[1],
    //          state->dynamixel_angles_[2],
    //          state->dynamixel_angles_[3]);
  // ROS_INFO("dynamixel speeds: %6u %6u %6u %6u",
  //          state->dynamixel_speeds_[0],
  //          state->dynamixel_speeds_[1],
  //          state->dynamixel_speeds_[2],
  //          state->dynamixel_speeds_[3]);
  // ROS_INFO("dynamixel loads: %6u %6u %6u %6u",
  //          state->dynamixel_loads_[0],
  //          state->dynamixel_loads_[1],
  //          state->dynamixel_loads_[2],
  //          state->dynamixel_loads_[3]);
  // ROS_INFO("dynamixel voltages: %6u %6u %6u %6u",
  //          state->dynamixel_voltages_[0],
  //          state->dynamixel_voltages_[1],
  //          state->dynamixel_voltages_[2],
  //          state->dynamixel_voltages_[3]);
  // ROS_INFO("dynamixel temperatures: %6u %6u %6u %6u",
  //          state->dynamixel_temperatures_[0],
  //          state->dynamixel_temperatures_[1],
  //          state->dynamixel_temperatures_[2],
  //          state->dynamixel_temperatures_[3]);


  // Sets and publishes the reflex_msgs/Hand message
  reflex_msgs::Hand hand_msg;
  int pressure_offset;
  // The dynamixel for Finger 1 has index 0, Finger 2 has index 1, Finger 3 has index 3
  for (int i = 0; i < reflex_hand::ReflexHandState::NUM_FINGERS; i++)
  {
    const int tactile_base_idx = i * 9;
    hand_msg.finger[i].proximal = ((state->encoders_[i] * reflex_hand::ReflexHand::ENC_SCALE) - enc_zero[i]);
// TODO: take this out when the motor order is switched
    // if (i == 2)
    //   hand_msg.finger[i].spool = motor_inversion[i]*((state->dynamixel_angles_[i+1] * reflex_hand::ReflexHand::DYN_SCALE / dyn_ratio[i]) - dyn_zero[i]);
    // ENC_SCALE
    hand_msg.finger[i].spool = motor_inversion[i]*((state->dynamixel_angles_[i] * reflex_hand::ReflexHand::DYN_SCALE / dyn_ratio[i]) - dyn_zero[i]);
    hand_msg.finger[i].distal = hand_msg.finger[i].spool - hand_msg.finger[i].proximal;

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
      if (hand_msg.finger[i].pressure[j] > contact_threshold)
        hand_msg.finger[i].contact[j] = true;
    }
  }
// TODO: Switch this 2 to a 3 when the dynamixel order is corrected
  // hand_msg.palm.preshape = motor_inversion[3] * ((state->dynamixel_angles_[2] * reflex_hand::ReflexHand::DYN_SCALE / dyn_ratio[3]) - dyn_zero[3]);
  hand_msg.palm.preshape = motor_inversion[3] * ((state->dynamixel_angles_[3] * reflex_hand::ReflexHand::DYN_SCALE / dyn_ratio[3]) - dyn_zero[3]);
  const int palm_tactile_base_idx = reflex_hand::ReflexHandState::NUM_FINGERS * 9;
  for (int i=0; i < 11; i++)
  {
      hand_msg.palm.pressure[i] = state->tactile_pressures_[palm_tactile_base_idx + i] - tactile_offset_palm[i];
      hand_msg.palm.contact[i] = false;
      if (state->tactile_pressures_[palm_tactile_base_idx + i] > contact_threshold)
        hand_msg.palm.contact[i] = true;
  }
  hand_msg.joints_publishing = true;
  hand_msg.tactile_publishing = true;
  
  hand_pub.publish(hand_msg);

  if (aqcuire_tactile)
  {
    tactile_file.open(tactile_file_address.c_str(), ios::out|ios::trunc);
    tactile_file << "# Captured sensor values from unloaded state, used for calibration \n";
    for (int i = 0; i < reflex_hand::ReflexHandState::NUM_FINGERS; i++)
    {
      const int tactile_base_idx = i * 9;
      // Write to variable
      for (int j = 0; j < 9; j++)
      {
        // This if statement is really messy, but I couldn't find a way to read the pressure
        // offset values in as a 3D vector. Probably possible, skipped for now
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
    for (int j = 0; j < 11; j++)
      tactile_offset_palm[j] = state->tactile_pressures_[palm_tactile_base_idx + j];
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
    if (first_capture)
    {
      finger_file.open(finger_file_address.c_str(), ios::out|ios::trunc);
      ROS_INFO("Capturing starter encoder positions");
      // Write to variable
      for (int i = 0; i < reflex_hand::ReflexHandState::NUM_FINGERS; i++)
        enc_zero[i] = state->encoders_[i] * reflex_hand::ReflexHand::ENC_SCALE;

      // Write to file
      finger_file << "# Calbration constants for joints [f1, f2, f3, preshape]\n";
      finger_file << "encoder_zero_reference: ["
                            << enc_zero[0] << ", "
                            << enc_zero[1] << ", "
                            << enc_zero[2] << "]\n";
      first_capture = false;
    }

    uint16_t increase[] = {10, 10, 10, 0};
    last_capture = true;
    for (int i = 0; i < reflex_hand::ReflexHandState::NUM_FINGERS; i++)
    {
      if (abs(enc_zero[i] - state->encoders_[i]*reflex_hand::ReflexHand::ENC_SCALE) > 0.1)
        increase[i] = 0;
      else
        last_capture = false;
    }

    ROS_INFO("Stepping the fingers inwards:\t%d\t%d\t%d\t%d", increase[0], increase[1], increase[2], increase[3]);
    reflex_hand::RawServoPositions servo_pos;
    for (int i=0; i<4; i++)
      servo_pos.raw_positions[i] = state->dynamixel_angles_[i] + motor_inversion[i]*increase[i];
    raw_pub.publish(servo_pos);

    if (last_capture)
    {
      ROS_INFO("Finger movement detected, zeroing motors");
      // Write to variable
      for (int i = 0; i<4; i++)
        dyn_zero[i] = state->dynamixel_angles_[i] * reflex_hand::ReflexHand::DYN_SCALE / dyn_ratio[i];
      // Write to file
      finger_file << "motor_zero_reference: ["
                            << state->dynamixel_angles_[0] * reflex_hand::ReflexHand::DYN_SCALE / dyn_ratio[0] << ", "
                            << state->dynamixel_angles_[1] * reflex_hand::ReflexHand::DYN_SCALE / dyn_ratio[1] << ", "
                            << state->dynamixel_angles_[2] * reflex_hand::ReflexHand::DYN_SCALE / dyn_ratio[2] << ", "
                            << state->dynamixel_angles_[3] * reflex_hand::ReflexHand::DYN_SCALE / dyn_ratio[3] << "]\n";
      aqcuire_fingers = false;
      finger_file.close();
    }
  }

  return;
}


bool zero_tactile(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  aqcuire_tactile = true;
  ROS_INFO("Zeroing tactile data at current values...");
  ROS_INFO("tactile_file_address: %s", tactile_file_address.c_str());
  return true;
}


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
  ros::init(argc, argv, "reflex_hand_driver");
  ros::NodeHandle nh, nh_private("~");
  load_params(nh);
  hand_pub = nh.advertise<reflex_msgs::Hand>("/reflex_hand", 10);
  ROS_INFO("Advertising the /reflex_hand topic");
  raw_pub = nh.advertise<reflex_hand::RawServoPositions>("set_reflex_raw", 1);

  string network_interface;
  nh_private.param<string>("network_interface", 
                                network_interface, "eth0");
  ROS_INFO("starting reflex_hand_driver on network interface %s", 
           network_interface.c_str());
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
  
  ros::Subscriber raw_positions_sub = nh.subscribe<reflex_hand::RawServoPositions>("set_reflex_raw", 1, boost::bind(set_raw_positions_cb, &rh, _1));
  ros::Subscriber radian_positions_sub = nh.subscribe<reflex_hand::RadianServoPositions>("set_reflex_hand", 1, boost::bind(set_radian_positions_cb, &rh, _1));
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
