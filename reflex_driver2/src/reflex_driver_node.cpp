/////////////////////////////////////////////////////////////////////////////
//
//   Copyright 2017 Open Source Robotics Foundation, Inc.
//   Copyright 2017-2018 Right Hand Robotics
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
/*

PUBLIC EXPOSURE BY THIS DRIVER

  Published topics
        /hand_state             Current state of hand

  subscribed topics
        /radian_hand_command    Execute radian commands, assuming hand is calibrated

  Advertised services
        /calibrate_tactile      Calibrate tactile sensors at current level
        /calibrate_fingers      Calibrate encoder and motor values
        /disable_torque         Disable finger torque
        /set_tactile_threshold  Set threshold for "contact" for each sensor individually
        /initIMUCal             Start IMU calibration
        /loadIMUCalData         Load data from yaml to firmware
        /saveIMUCalData         Save data from firmware to yaml
        /refreshIMUCalData      ......................==============================================================? Clarify with john

*/

#include "reflex_driver_node.h"
using namespace std;

ros::Publisher hand_pub;
ros::Publisher raw_pub;

ofstream tactile_file;                        // Accesses tactile calibration file
ofstream imu_calibration_file;                // Accesses IMU calibration file
string tactile_file_address;                  // Set in main()
string imu_file_address;                      // Set in main()
ofstream finger_file;                         // Accesses finger calibration file
string finger_file_address;                   // Set in main()

vector<int> MOTOR_TO_JOINT_INVERTED;          // Loaded from yaml 
vector<double> MOTOR_TO_JOINT_GEAR_RATIO;     // Loaded from yaml
int default_contact_threshold;                // Loaded from yaml 

// Set by /set_tactile_threshold ROS service
reflex_msgs2::SetTactileThreshold::Request contact_thresholds;   
vector<int> tactile_offset_f1;                // Loaded from yaml and reset during calibration
vector<int> tactile_offset_f2;                // Loaded from yaml and reset during calibration
vector<int> tactile_offset_f3;                // Loaded from yaml and reset during calibration
const int TACTILE_BASE_IDX[] = {0, 28, 14}; 

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Pointer
const int IMU_BASE_IDX[] = {0, 11, 22, 33};


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int encoder_last_value[] = {0, 0, 0};         // Updated constantly in reflex_hand_state_cb()
int encoder_offset[] = {-1, -1, -1};          // Updated constantly in reflex_hand_state_cb()
float load_last_value[] = {0, 0, 0, 0};       // Updated constantly in reflex_hand_state_cb()
int raw_cmd_last_value[] = {0, 0, 0, 0};      // Updated in receive_raw_cmd_cb and receive_angle_cmd_cb

bool acquire_tactile = false;                 // Updated by /calibrate_tactile ROS service and in reflex_hand_state_cb()
bool acquire_fingers = false;                 // Updated by /calibrate_fingers ROS service and in reflex_hand_state_cb()
bool first_capture = false;                   // Updated by /calibrate_fingers ROS service and in reflex_hand_state_cb()
bool all_fingers_moved = false;               // Updated in reflex_hand_state_cb()

vector<double> dynamixel_zero_point;          // Loaded from yaml and reset during calibration
vector<double> encoder_zero_point;            // Loaded from yaml and reset during calibration

vector<int> imu_calibration_data_f1;
vector<int> imu_calibration_data_f2; 
vector<int> imu_calibration_data_f3;
vector<int> imu_calibration_data_palm;  
bool acquire_imus = false;  
//////////////////////////////////////////////////////////.. should I make a flag? //bool set_imus = false;

uint16_t calibration_dyn_increase[] = {6, 6, 6, 0};         // Updated itxn reflex_hand_state_cb() during calibration
const uint16_t CALIBRATION_DYN_OFFSET[] = {50, 50, 50, 0};  
ros::Time latest_calibration_time;                          // Updated in reflex_hand_state_cb() during calibration

bool g_done = false;    // Updated by signum handler below

typedef struct 
{
  float w;
  float x;
  float y;
  float z;
} __attribute__((packed)) Quaternion;

Quaternion getQuaternion(float w, float x, float y, float z)
{
  Quaternion quat;
  quat.w = w;
  quat.x = x;
  quat.y = y;
  quat.z = z;
  return quat; 
}

Quaternion invertQuaternion(Quaternion quat)
{
  Quaternion inversedQuat;
  float dividend = quat.w * quat.w + quat.x * quat.x + 
    quat.y * quat.y + quat.z * quat.z;
  inversedQuat.w =  quat.w / dividend;
  inversedQuat.x = (-1) * (quat.x / dividend);
  inversedQuat.y = (-1) * (quat.y / dividend);
  inversedQuat.z = (-1) * (quat.z / dividend);
  return inversedQuat;           
}

// source https://www.youtube.com/watch?v=jlskQDR8-bY
// quaternion multiplication is not commutative q1*q2 != q2*q1
Quaternion multiplyQuaternion(Quaternion q1, Quaternion q2)
{
  Quaternion product;
  product.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
  product.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
  product.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
  product.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
  return product;
}
/*
// Calculate Euler angles (roll, pitch, and yaw) from the unit quaternion.

source: https://developer.thalmic.com/docs/api_reference/platform/hello-myo_8cpp-example.html
        float roll = atan2(2.0f * (quat.w() * quat.x() + quat.y() * quat.z()),
                           1.0f - 2.0f * (quat.x() * quat.x() + quat.y() * quat.y()));
        float pitch = asin(max(-1.0f, min(1.0f, 2.0f * (quat.w() * quat.y() - quat.z() * quat.x()))));
        float yaw = atan2(2.0f * (quat.w() * quat.z() + quat.x() * quat.y()),
                        1.0f - 2.0f * (quat.y() * quat.y() + quat.z() * quat.z()));

*/

// Outputs are in radians
static void toEulerAngle(const Quaternion q, double& roll, double& pitch, double& yaw)
{
  // roll (x-axis rotation)
  double sinr = +2.0 * (q.w * q.x + q.y * q.z);
  double cosr = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
  roll = atan2(sinr, cosr);

  // pitch (y-axis rotation)
  double sinp = +2.0 * (q.w * q.y - q.z * q.x);
  if (fabs(sinp) >= 1)
    pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
    pitch = asin(sinp);

  // yaw (z-axis rotation)
  double siny = +2.0 * (q.w * q.z + q.x * q.y);
  double cosy = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);  
  yaw = atan2(siny, cosy);
}


void signal_handler(int signum) {
  if (signum == SIGINT || signum == SIGTERM)
    g_done = true;
}

/*
TODO(LANCE): How does this logic work? when do we use topic?

getParam()
  Returns a bool: provides the ability to check if retrieving the parameter succeeded or not: 
  Takes in a string "key"
  Takes in a parameter type

setParam()
deleteParam()
searchParam()
*/
void load_params(ros::NodeHandle nh) {
  string topic = "No error";
  if (!nh.getParam("motor_zero_reference", dynamixel_zero_point))
    topic = "motor_zero_reference";
  if (!nh.getParam("encoder_zero_reference", encoder_zero_point))
    topic = "encoder_zero_reference";
  if (!nh.getParam("motor_to_joint_inverted", MOTOR_TO_JOINT_INVERTED))
    topic = "motor_to_joint_inverted";
  if (!nh.getParam("motor_to_joint_gear_ratio", MOTOR_TO_JOINT_GEAR_RATIO))
    topic = "motor_to_joint_gear_ratio";
  if (!nh.getParam("default_contact_threshold", default_contact_threshold))
    topic = "default_contact_threshold";
  if (!nh.getParam("tactile_offset_f1", tactile_offset_f1))
    topic = "tactile_offset_f1";
  if (!nh.getParam("tactile_offset_f2", tactile_offset_f2))
    topic = "tactile_offset_f2";
  if (!nh.getParam("tactile_offset_f3", tactile_offset_f3))
    topic = "tactile_offset_f3";
  if (!nh.getParam("imu_calibration_data_f1", imu_calibration_data_f1))
    topic = "imu_calibration_data_f1";
  if (!nh.getParam("imu_calibration_data_f2", imu_calibration_data_f2))
    topic = "imu_calibration_data_f2";
  if (!nh.getParam("imu_calibration_data_f3", imu_calibration_data_f3))
    topic = "imu_calibration_data_f3";
  if (!nh.getParam("imu_calibration_data_palm", imu_calibration_data_palm))
    topic = "imu_calibration_data_palm";

  if (topic != "No error") {
    ROS_FATAL("Failed to load %s parameter", topic.c_str());
    ROS_FATAL("This is likely because the corresponding yaml file in");
    ROS_FATAL("reflex_driver/yaml were never loaded, or because they");
    ROS_FATAL("have been corrupted. If they were corrupted, they can");
    ROS_FATAL("be repaired by pasting the *_backup.yaml file text in");
    ros::shutdown();
  }

  ROS_INFO("Succesfully loaded all parameters");
} 


/*
Takes raw Dynamixel values (0-4095) and writes them directly to motors
Note: The Dynamixels have a resolution divider of 4. 
    http://support.robotis.com/en/product/dynamixel/mx_series/mx-28.htm#Actuator_Address_0B1
*/
void receive_raw_cmd_cb(reflex_hand::ReflexHand *rh,
                          const reflex_msgs2::RawServoCommands::ConstPtr &msg) {
  uint16_t targets[reflex_hand::ReflexHand::NUM_SERVOS];

  for (int i = 0; i < reflex_hand::ReflexHand::NUM_SERVOS; i++) {
    targets[i] = msg->raw_positions[i];
    raw_cmd_last_value[i] = targets[i];
  }

  //..................................====================================================================== What is targets?
  rh->setServoTargets(targets);
}


/*
Commands motors from radians
Using zero references in "yaml/finger_calibrate.yaml" to translate into 
the raw Dynamixel values
*/
void receive_angle_cmd_cb(reflex_hand::ReflexHand *rh,
                          const reflex_msgs2::RadianServoCommands::ConstPtr &msg) {
  uint16_t targets[reflex_hand::ReflexHand::NUM_SERVOS];

  for (int i = 0; i < reflex_hand::ReflexHand::NUM_SERVOS; i++) {
    targets[i] = pos_rad_to_raw(msg->radian_commands[i], i);
    raw_cmd_last_value[i] = targets[i];
  }

  rh->setServoTargets(targets);
}


/*
Takes in: rad/s command 
Returns: Dynamixel command
*/
uint16_t pos_rad_to_raw(float rad_command, int motor_idx) {
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


// Changes travel speed of motor
bool set_motor_speed(reflex_hand::ReflexHand *rh,
                     reflex_msgs2::SetSpeed::Request &req, reflex_msgs2::SetSpeed::Response &res) {
  rh->setServoControlModes(reflex_hand::ReflexHand::CM_VELOCITY);
  ros::Duration(0.02).sleep();

  uint16_t targets[reflex_hand::ReflexHand::NUM_SERVOS];

  for (int i = 0; i < reflex_hand::ReflexHand::NUM_SERVOS; i++)
    targets[i] = speed_rad_to_raw(req.motor[i], i);

  rh->setServoTargets(targets);
  ros::Duration(0.035).sleep();  // Sleep necessary to prevent Brain Board freezing
  rh->setServoControlModes(reflex_hand::ReflexHand::CM_POSITION);
  check_for_potential_motor_wraps_and_rezero();
  ros::Duration(0.035).sleep();  // Sleep necessary to prevent Brain Board freezing
  return true;
}


// Takes rad/s command and returns Dynamixel command
// http://support.robotis.com/en/product/dynamixel/mx_series/mx-28.htm#Actuator_Address_20
uint16_t speed_rad_to_raw(float rad_per_s_command, int motor_idx) {
  uint16_t command = abs(rad_per_s_command) *
                     (MOTOR_TO_JOINT_GEAR_RATIO[motor_idx] / reflex_hand::ReflexHand::DYN_VEL_SCALE);
  if (command > 1023) 
    command = 1023;
  
  if (MOTOR_TO_JOINT_INVERTED[motor_idx] * rad_per_s_command < 0) 
    command += 1024;
  
  if (command == 0)
    command = 1;  // 0 does not actually stop the motor

  return command;
}


// Checks whether motor positions will reset after mode switch, 
// and corrects zero point.
// When switching modes (VELOCITY and POSITION),  
// motor will wrap values if above 14024 or below 13000
void check_for_potential_motor_wraps_and_rezero() {
  double motor_wrap;
  for (int i = 0; i < reflex_hand::ReflexHand::NUM_SERVOS; i++) 
    motor_wrap = 1025 * (reflex_hand::ReflexHand::DYN_POS_SCALE / MOTOR_TO_JOINT_GEAR_RATIO[i]);
}


bool disable_torque(reflex_hand::ReflexHand *rh, std_srvs::Empty::Request &req, 
                    std_srvs::Empty::Response &res) {
  rh->setServoControlModes(reflex_hand::ReflexHand::CM_IDLE);
  return true;
}


// Sets procedure to calibrate the tactile values in motion
bool calibrate_tactile(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  acquire_tactile = true;
  ROS_INFO("Zeroing tactile data at current values...");
  return true;
}


// Sets procedure to calibrate the fingers in motion
bool calibrate_fingers(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  ROS_INFO("Beginning finger calibration sequence...");
  acquire_fingers = true;
  first_capture = true;
  all_fingers_moved = false;
  return true;
}


// Sets tactile threshold levels to be all one value
void populate_tactile_threshold(int threshold) {
  for (int i = 0; i < reflex_hand::ReflexHandState::NUM_FINGERS; i++) {
    for (int j = 0; j < reflex_hand::ReflexHand::NUM_SENSORS_PER_FINGER; j++)
      contact_thresholds.finger[i].sensor[j] = threshold;
  }
}


// Sets threshold levels on tactile sensors for reporting contact
bool set_tactile_threshold(reflex_msgs2::SetTactileThreshold::Request &req,
                           reflex_msgs2::SetTactileThreshold::Response &res) {
  contact_thresholds = req;
  return true;
}


// Returns correct pressure calibration offset for finger[sensor]
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

  if (enc_offset == -1)
    enc_offset = 0; // This case happens upon startup
  
  else {
    // If encoder value jumps, it has wrapped a revolution
    if ((last_value - raw_value) > 5000)
      enc_offset = enc_offset + 16383;
    else if ((last_value - raw_value) < -5000)
      enc_offset = enc_offset - 16383;
  }

  return enc_offset;
}


// Calculates actual proximal angle using raw sensor value, wrap offset
// for that finger, and calibrated "zero" point for that encoder
float calc_proximal_angle(int raw_enc_value, int offset, double zero) {
  int wrapped_enc_value = raw_enc_value + offset;
  float rad_value = (float) wrapped_enc_value * reflex_hand::ReflexHand::ENC_SCALE;
  return (zero - rad_value);
}


// Calculates joint angle from raw sensor value, motor gear ratio,
// and calibrated "zero" point for the joint
float calc_motor_angle(int inversion, int raw_dyn_value, double ratio, double zero) {
  float rad_value = raw_dyn_value * reflex_hand::ReflexHand::DYN_POS_SCALE / ratio;
  float zeroed_value = rad_value - zero;
  return inversion * zeroed_value;
}


// Calculates distal angle, "tendon spooled out" - "proximal encoder" angles
// Could be improved. HOW SWAY?
float calc_distal_angle(float joint_angle, float proximal) {
  float diff = joint_angle - proximal;
  return (diff < 0) ? 0 : diff;
}


// Takes hand state and returns calibrated tactile data for given finger
int calc_pressure(const reflex_hand::ReflexHandState* const state, int finger, int sensor) {
  int raw_value = state->tactile_pressures_[TACTILE_BASE_IDX[finger] + sensor];
  return raw_value - pressure_offset(finger, sensor);
}


// Checks given finger/sensor for contact threshold
int calc_contact(reflex_msgs2::Hand hand_msg, int finger, int sensor) {
  int pressure_value = hand_msg.finger[finger].pressure[sensor];
  return pressure_value > contact_thresholds.finger[finger].sensor[sensor];
}


/* 
  Takes in hand state data and publishes to reflex_hand topic
  Also performs calibration when certain booleans are enabled
*/
void reflex_hand_state_cb(const reflex_hand::ReflexHandState * const state) {
  reflex_msgs2::Hand hand_msg; 
  Quaternion p;

  // 1 Quaternion (unit less) = 2^14 LSB
  const float scale = (1.0 / (1 << 14)); // TODO(LANCE): verify if correct

  // PALM
  for (int i = 0; i < 4; i++)
    hand_msg.palmImu.quat[i] = float (scale * state->imus[12 + i]);

  for (int i = 0; i < 3; i++)
    hand_msg.palmImu.euler_angles[i] = 0;

  p = getQuaternion(hand_msg.palmImu.quat[0], hand_msg.palmImu.quat[1], 
    hand_msg.palmImu.quat[2], hand_msg.palmImu.quat[3]);

  hand_msg.palmImu.calibration_status = state->imu_calibration_status[3];

  for (int i = 0; i < 11; i++)
    hand_msg.palmImu.calibration_data[i] 
      = state->imu_calibration_data[reflex_hand::ReflexHandState::NUM_FINGERS * 11 + i]; 

  // FINGER
  for (int i = 0; i < reflex_hand::ReflexHandState::NUM_FINGERS; i++) {
    double roll, pitch, yaw;
    Quaternion f, inv, prod;

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

    for (int j = 0; j < 5; j++) {
      hand_msg.finger[i].pressure[j] = calc_pressure(state, i, j);
      hand_msg.finger[i].contact[j] = calc_contact(hand_msg, i, j);
    }
    hand_msg.finger[i].pressure[5] = calc_pressure(state, i, 10);
    hand_msg.finger[i].contact[5] = calc_contact(hand_msg, i, 5);
    hand_msg.finger[i].pressure[6] = calc_pressure(state, i, 11);
    hand_msg.finger[i].contact[6] = calc_contact(hand_msg, i, 6);
    hand_msg.finger[i].pressure[7] = calc_pressure(state, i, 7);
    hand_msg.finger[i].contact[7] = calc_contact(hand_msg, i, 7);
    hand_msg.finger[i].pressure[8] = calc_pressure(state, i, 8);
    hand_msg.finger[i].contact[8] = calc_contact(hand_msg, i, 8);
    hand_msg.finger[i].pressure[9] = calc_pressure(state, i, 9);
    hand_msg.finger[i].contact[9] = calc_contact(hand_msg, i, 9);
    hand_msg.finger[i].pressure[10] = calc_pressure(state, i, 13);
    hand_msg.finger[i].contact[10] = calc_contact(hand_msg, i, 10);
    hand_msg.finger[i].pressure[11] = calc_pressure(state, i, 12);
    hand_msg.finger[i].contact[11] = calc_contact(hand_msg, i, 11);
    hand_msg.finger[i].pressure[12] = calc_pressure(state, i, 5);
    hand_msg.finger[i].contact[12] = calc_contact(hand_msg, i, 12);
    hand_msg.finger[i].pressure[13] = calc_pressure(state, i, 6);
    hand_msg.finger[i].contact[13] = calc_contact(hand_msg, i, 13);
    // IMU
    for (int j = 0; j < 4; j++){
      hand_msg.finger[i].imu.quat[j] = float (scale * state->imus[i * 4 + j]);
    }

    f = getQuaternion(hand_msg.finger[i].imu.quat[0], hand_msg.finger[i].imu.quat[1],
                      hand_msg.finger[i].imu.quat[2], hand_msg.finger[i].imu.quat[3]);
    inv = invertQuaternion(f);
    prod = multiplyQuaternion(p,f);
    toEulerAngle(prod, roll, pitch, yaw);

    // Convert radians to degrees
    hand_msg.finger[i].imu.euler_angles[0] = roll * 180 / 3.14159;
    hand_msg.finger[i].imu.euler_angles[1] = pitch * 180 / 3.14159;
    hand_msg.finger[i].imu.euler_angles[2] = yaw * 180 / 3.14159;


    hand_msg.finger[i].imu.calibration_status = state->imu_calibration_status[i];
    
    for (int j = 0; j < 11; j++)
      hand_msg.finger[i].imu.calibration_data[j] = state->imu_calibration_data[i * 11 + j];
    
  }

  // MOTOR
  hand_msg.motor[3].joint_angle = calc_motor_angle(MOTOR_TO_JOINT_INVERTED[3],
                                                   state->dynamixel_angles_[3],
                                                   MOTOR_TO_JOINT_GEAR_RATIO[3],
                                                   dynamixel_zero_point[3]);
  populate_motor_state(&hand_msg, state);

  hand_pub.publish(hand_msg);

  // REPEATED
  // for (int i = 0; i < 4; i++)
  //   hand_msg.palmImu.quat[i] = float (scale * state->imus[12 + i]);

  hand_msg.palmImu.calibration_status = state->imu_calibration_status[3];

  if (acquire_imus) 
    log_imu_calibration_data(state);

  // Capture current tactile data and save it as a zero reference
  if (acquire_tactile) 
    calibrate_tactile_sensors(state, hand_msg);
  

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
    } 

    else 
      move_fingers_in(state);
    
    latest_calibration_time = ros::Time::now();
  }

  return;
}


/*
  Opens tactile calibration data file
  Changes local tactile_offset
  Saves current tactile values to file as the new calibrated "zero"
*/
void calibrate_tactile_sensors(const reflex_hand::ReflexHandState* const state,
                               reflex_msgs2::Hand hand_msg) {
  tactile_file.open(tactile_file_address.c_str(), ios::out|ios::trunc);
  tactile_file << "# Captured sensor values from unloaded state\n";
  log_current_tactile_locally(state);
  for (int i = 0; i < reflex_hand::ReflexHandState::NUM_FINGERS; i++) 
    log_current_tactile_to_file(state, i);
  
  acquire_tactile = false;
  tactile_file.close();
}


// Save local variables tactile_offset_f* with current tactile position
void log_current_tactile_locally(const reflex_hand::ReflexHandState* const state) {
  for (int j = 0; j < reflex_hand::ReflexHand::NUM_SENSORS_PER_FINGER; j++) {
    tactile_offset_f1[j] = state->tactile_pressures_[TACTILE_BASE_IDX[0] + j];
    tactile_offset_f2[j] = state->tactile_pressures_[TACTILE_BASE_IDX[1] + j];
    tactile_offset_f3[j] = state->tactile_pressures_[TACTILE_BASE_IDX[2] + j];
  }
}


void log_current_tactile_to_file(const reflex_hand::ReflexHandState* const state,
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
               << state->tactile_pressures_[TACTILE_BASE_IDX[finger] + 8] << ", "
               << state->tactile_pressures_[TACTILE_BASE_IDX[finger] + 9] << ", "
               << state->tactile_pressures_[TACTILE_BASE_IDX[finger] + 10] << ", "
               << state->tactile_pressures_[TACTILE_BASE_IDX[finger] + 11] << ", "
               << state->tactile_pressures_[TACTILE_BASE_IDX[finger] + 12] << ", "
               << state->tactile_pressures_[TACTILE_BASE_IDX[finger] + 13] << "]\n";
}


// Capture current encoder position locally as "zero" and save to file
void calibrate_encoders_locally(const reflex_hand::ReflexHandState* const state) {
  for (int i = 0; i < reflex_hand::ReflexHandState::NUM_FINGERS; i++) {
    encoder_zero_point[i] = state->encoders_[i] * reflex_hand::ReflexHand::ENC_SCALE;
    encoder_offset[i] = 0;
  }
}


/*
  Save current dynamixel location (plus an offset) as "zero" and then
  Write dynamixels to spot
*/
void calibrate_motors_locally(const reflex_hand::ReflexHandState* const state) {
  reflex_msgs2::RawServoCommands servo_pos;
  int motor_offset;
  float motor_scalar;

  for (int i = 0; i < reflex_hand::ReflexHand::NUM_SERVOS; i++) {
    motor_offset = MOTOR_TO_JOINT_INVERTED[i] * CALIBRATION_DYN_OFFSET[i];
    motor_scalar = reflex_hand::ReflexHand::DYN_POS_SCALE / MOTOR_TO_JOINT_GEAR_RATIO[i];
    dynamixel_zero_point[i] = (state->dynamixel_angles_[i] - motor_offset) * motor_scalar;
    servo_pos.raw_positions[i] = state->dynamixel_angles_[i] - motor_offset;
  }

  raw_pub.publish(servo_pos);
}


void log_encoder_zero_to_file() {
  finger_file.open(finger_file_address.c_str(), ios::out|ios::trunc);
  finger_file << "# Calibration constants for [f1, f2, f3, preshape]\n";
  finger_file << "encoder_zero_reference: ["
              << encoder_zero_point[0] << ", "
              << encoder_zero_point[1] << ", "
              << encoder_zero_point[2] << "]\n";
}


void log_motor_zero_to_file_and_close() {
  finger_file << "motor_zero_reference: ["
              << dynamixel_zero_point[0] << ", "
              << dynamixel_zero_point[1] << ", "
              << dynamixel_zero_point[2] << ", "
              << dynamixel_zero_point[3] << "]\n";
  finger_file.close();
}


bool initIMUCal(reflex_hand::ReflexHand *rh, std_srvs::Empty::Request &req, 
                std_srvs::Empty::Response &res) {
  ROS_INFO("Initializing IMU...");
  rh->initIMUCal();
  return true;
}


bool saveIMUCalData(std_srvs::Empty::Request &req, 
                    std_srvs::Empty::Response &res) {
  ROS_INFO("Saving IMU calibration data...");
  acquire_imus = true;
  return true;
}


/* 
  Loads yaml file to firmware
  Takes in: uint16 array
  Returns: bool
*/
bool loadIMUCalData(reflex_hand::ReflexHand *rh, 
                    std_srvs::Empty::Request &req, 
                    std_srvs::Empty::Response &res) {
  ROS_INFO("Loading IMU calibration data...");
  
  uint16_t buffer[reflex_hand::ReflexHandState::NUM_IMUS * 11];

  //Collect calibration data into single array
  for (int i = 0; i < 11; i++){

    buffer[i] = imu_calibration_data_f1[i];
    buffer[i + 11] = imu_calibration_data_f2[i];
    buffer[i + 22] = imu_calibration_data_f3[i];
    buffer[i + 33] = imu_calibration_data_palm[i];

  }


  //Print Buffered Calibration Data Through Console
  ostringstream f1, f2, f3, palm;
  f1 << "imu_calibration_data_f1: ";
  f2 << "imu_calibration_data_f2: ";
  f3 << "imu_calibration_data_f3: ";
  palm << "imu_calibration_data_palm: ";

  for (int i = 0; i < 11; i++){
    f1 << "[" << buffer[i] << "] ";
    f2 << "[" << buffer[i + 11] << "] ";
    f3 << "[" << buffer[i + 22] << "] ";
    palm << "[" << buffer[i + 33] << "] ";
  }

  ROS_INFO_STREAM(f1.str());
  ROS_INFO_STREAM(f2.str());
  ROS_INFO_STREAM(f3.str());
  ROS_INFO_STREAM(palm.str());

  //Cast 16-bit int array to 8-bit int array to send over ethernet
  rh->loadIMUCalData((uint8_t *)buffer);

  return true;
}


/*
  Opens IMU calibration data file
  Saves current IMU calibration values to file as new calibrated "zero"
*/
void log_imu_calibration_data(const reflex_hand::ReflexHandState* const state) {
  imu_calibration_file.open(imu_file_address.c_str(), ios::out|ios::trunc); // What is the second argument?
  imu_calibration_file << "# Opened file successfully\n"; // This line clears the file
  
  ///ROS_INFO("Entered log_imu_calibration_data()");
  
  for (int finger = 0; finger < (reflex_hand::ReflexHandState::NUM_FINGERS + 1); finger++)  // There's an IMU on the palm too
    log_current_imu_offsets_to_file(state, finger);

  imu_calibration_file.close();
  acquire_imus = false;
}

/* 
  TODO(LANCE): 
    --- Ask John for code review
    --- Make plan for testing
*/
void log_current_imu_offsets_to_file(const reflex_hand::ReflexHandState* const state, int finger) {
  string label;
  //ROS_INFO("Entered log_current_imu_offsets_to_file()");
  if (finger == 3)
    imu_calibration_file << "imu_calibration_data_palm" << ": [";
  else
    imu_calibration_file << "imu_calibration_data_f" << (finger + 1) << ": [";

  for (int i = 0; i < 10; i++) { 
      // imu_calibration_file is? ofstream and accesses the imu calibration file
    imu_calibration_file << state->imu_calibration_data[IMU_BASE_IDX[finger] + i] << ", ";  
  }

  //imu_calibration_file << "10" << "]\n"; 
  imu_calibration_file << state->imu_calibration_data[IMU_BASE_IDX[finger] + 10] << "]\n";
}

/*
  TODO(LANCE)
    --- Ask John for requirements
    --- Understand if return type and arguments are correct
*/
bool refreshIMUCalData(reflex_hand::ReflexHand *rh, 
                       std_srvs::Empty::Request &req, 
                       std_srvs::Empty::Response &res) {
  ROS_INFO("Refreshing IMU calibration data...");
  rh->refreshIMUCalData();
  return true;
}

// Checks whether all fingers have moved more than CALIBRATION_ERROR, halts
// the motion of fingers past that point
bool check_for_finger_movement(const reflex_hand::ReflexHandState* const state) {
  bool all_fingers_moved = true;
  for (int i = 0; i < reflex_hand::ReflexHandState::NUM_FINGERS; i++) {
    float enc_pos = encoder_zero_point[i] -
                    state->encoders_[i] * reflex_hand::ReflexHand::ENC_SCALE;
    
    if (abs(enc_pos) > CALIBRATION_ERROR) 
      calibration_dyn_increase[i] = 0;
    else {
      calibration_dyn_increase[i] = 6;
      all_fingers_moved = false;
    }
  }
  return all_fingers_moved;
}


// Steps the fingers in by a calibration amount
void move_fingers_in(const reflex_hand::ReflexHandState* const state) {
  reflex_msgs2::RawServoCommands servo_pos;
  int motor_step;
  ROS_INFO("Step fingers inwards:\t%d+%d\t%d+%d\t%d+%d\t%d+%d",
           state->dynamixel_angles_[0], calibration_dyn_increase[0],
           state->dynamixel_angles_[1], calibration_dyn_increase[1],
           state->dynamixel_angles_[2], calibration_dyn_increase[2],
           state->dynamixel_angles_[3], calibration_dyn_increase[3]);
  
  for (int i=0; i < reflex_hand::ReflexHand::NUM_SERVOS; i++) {
    motor_step = MOTOR_TO_JOINT_INVERTED[i] * calibration_dyn_increase[i];
    servo_pos.raw_positions[i] = state->dynamixel_angles_[i] + motor_step;
  }

  if (state->dynamixel_angles_[0] == 0 || state->dynamixel_angles_[1] == 0 || 
      state->dynamixel_angles_[2] == 0 || state->dynamixel_angles_[3] == 0){

      ROS_FATAL("ERROR! Encoder malfunction, prevented motor catastrophe.\nPlease check finger connections!");
      g_done = true;

  }
  else{
    raw_pub.publish(servo_pos);
  }
}


// Captures current hand state and publishes to a debug message 
void populate_motor_state(reflex_msgs2::Hand* hand_msg, const reflex_hand::ReflexHandState* const state) {
  char buffer[10];

  for (int i = 0; i < reflex_hand::ReflexHand::NUM_SERVOS; i++) {
    hand_msg->motor[i].raw_angle = (float) state->dynamixel_angles_[i];
    hand_msg->motor[i].velocity = (float) state->dynamixel_speeds_[i];
    hand_msg->motor[i].load = load_raw_to_signed(state->dynamixel_loads_[i], i);
    hand_msg->motor[i].voltage = (float) state->dynamixel_voltages_[i];
    hand_msg->motor[i].temperature = state->dynamixel_temperatures_[i];
    sprintf(buffer, "0x%02x", state->dynamixel_error_states_[i]);
    hand_msg->motor[i].error_state = buffer;
  }
}


  /*
    Takes load
    Converts to float 
    Applies rolling filter  // TODO(LANCE): Understand what this is
  */
  float load_raw_to_signed(int load, int motor_idx) {
  if (load > 1023)
    load = (load - 1023);
  else
    load *= -1;
    //load = -1 * load;

  float signed_load = (float) (MOTOR_TO_JOINT_INVERTED[motor_idx] * load);
  float load_filter = 0.25;  // Rolling filter of noisy data
  float filter_load = load_filter * signed_load + (1 - load_filter) * load_last_value[motor_idx];
  load_last_value[motor_idx] = filter_load;
  return filter_load;
}

// Take in command line arguments specifying ethernet name, eth0 or eth1
int main(int argc, char **argv) { 
  std::vector<string> args;
  ros::removeROSArgs(argc, argv, args);
  string ethernet_name = args[1].data();

  // Initialize ROS node
  char nodeName[27];
  strcpy(nodeName, "reflex_takktile2_driver_");
  strcat(nodeName, ethernet_name.c_str());
  ros::init(argc, argv, nodeName);
  ros::NodeHandle nh, nh_private("~");
  load_params(nh);
  populate_tactile_threshold(default_contact_threshold);
  ROS_INFO("Populate_tactile_threshold");
  string ns = "/reflex_takktile2";

  // Advertise necessary topics
  hand_pub = nh.advertise<reflex_msgs2::Hand>(ns + "/hand_state", 10);
  ROS_INFO("Publishing the /hand_state topic");
  raw_pub = nh.advertise<reflex_msgs2::RawServoCommands>(ns + "/raw_hand_command", 1);

  // Intialize reflex_hand object
  string network_interface;
  nh_private.param<string>("Network_interface", network_interface, ethernet_name);
  ROS_INFO("Starting reflex_hand_driver on network interface %s",
           network_interface.c_str());
  reflex_hand::ReflexHand rh(network_interface);
  
  if (!rh.happy()) {
    ROS_FATAL("Error during initialization. Bailing now. Have a nice day.");
    return 1;
  }

  rh.setStateCallback(reflex_hand_state_cb);
  rh.setServoControlModes(reflex_hand::ReflexHand::CM_POSITION);
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  // Subscribe to hand command topics
  ros::Subscriber raw_positions_sub =
    nh.subscribe<reflex_msgs2::RawServoCommands>(ns + "/raw_hand_command", 10,
                                                boost::bind(receive_raw_cmd_cb, &rh, _1));
  ros::Subscriber radian_positions_sub =
    nh.subscribe<reflex_msgs2::RadianServoCommands>(ns + "/radian_hand_command", 10,
                                                   boost::bind(receive_angle_cmd_cb, &rh, _1));

  // Initialize hand command services
  ros::ServiceServer set_speed_service =
    nh.advertiseService<reflex_msgs2::SetSpeed::Request, reflex_msgs2::SetSpeed::Response>
      (ns + "/set_speed", boost::bind(set_motor_speed, &rh, _1, _2));
  ros::ServiceServer disable_service =
    nh.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>
      (ns + "/disable_torque", boost::bind(disable_torque, &rh, _1, _2));

  // Initialize /calibrate_tactile and /calibrate_fingers services
  string buffer;
  nh.getParam("yaml_dir", buffer);
  finger_file_address = buffer + "/finger_calibrate.yaml";
  tactile_file_address = buffer + "/tactile_calibrate.yaml";
  imu_file_address = buffer + "/imu_calibrate.yaml"; 
  
  ros::ServiceServer calibrate_fingers_service = nh.advertiseService(ns + "/calibrate_fingers", 
    calibrate_fingers);
  latest_calibration_time = ros::Time::now();
  ROS_INFO("Advertising the /calibrate_fingers service");
  
  // Initialize IMU services
  ros::ServiceServer initIMUCal_service =
    nh.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>
      (ns + "/initIMUCal", boost::bind(initIMUCal, &rh, _1, _2));
  ROS_INFO("Advertising the /initIMUCal service");
  
  ros::ServiceServer saveIMUCalData_service = nh.advertiseService(ns + "/saveIMUCalData", 
    saveIMUCalData);
  ROS_INFO("Advertising the /saveIMUCalData service");

  ros::ServiceServer loadIMUCalData_service = nh.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>
      (ns + "/loadIMUCalData", boost::bind(loadIMUCalData, &rh, _1, _2));
  ROS_INFO("Advertising the /loadIMUCalData service");

  ros::ServiceServer refreshIMUCalData_service = nh.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>
      (ns + "/refreshIMUCalData", boost::bind(refreshIMUCalData, &rh, _1, _2));
  ROS_INFO("Advertising the /refreshIMUCalData service");

  ros::ServiceServer calibrate_tactile_service = nh.advertiseService(ns + "/calibrate_tactile", 
    calibrate_tactile);
  ROS_INFO("Advertising the /calibrate_tactile service");
  
  ros::ServiceServer set_thresh_service = nh.advertiseService(ns + "/set_tactile_threshold", 
    set_tactile_threshold);
  ROS_INFO("Advertising the /set_tactile_threshold service");

  ROS_INFO("Entering main reflex_driver loop...");
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
