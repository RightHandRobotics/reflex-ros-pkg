#ifndef REFLEX_DRIVER_NODE_H
#define REFLEX_DRIVER_NODE_H

#define CALIBRATION_ERROR 0.05  // Encoder delta signifying movement in calibration

void signal_handler(int signum);
void load_params(ros::NodeHandle nh);

void receive_raw_cmd_cb(reflex_hand::ReflexHand *rh,
                        const reflex_msgs::RawServoPositions::ConstPtr &msg);
void receive_angle_cmd_cb(reflex_hand::ReflexHand *rh,
                          const reflex_msgs::RadianServoCommands::ConstPtr &msg);
uint16_t pos_rad_to_raw(float rad_command, int motor_idx);
bool set_motor_speed(reflex_hand::ReflexHand *rh,
					 reflex_msgs::SetSpeed::Request &req, reflex_msgs::SetSpeed::Response &res);
uint16_t speed_rad_to_raw(float rad_per_s_command, int motor_idx);
bool enable_torque(reflex_hand::ReflexHand *rh, std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
bool disable_torque(reflex_hand::ReflexHand *rh, std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
bool zero_tactile(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
bool zero_fingers(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
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
void calibrate_encoders(const reflex_hand::ReflexHandState* const state);
bool check_for_finger_movement(const reflex_hand::ReflexHandState* const state);
void move_fingers_in(const reflex_hand::ReflexHandState* const state);
void log_motor_zero_locally(const reflex_hand::ReflexHandState* const state);
void log_motor_zero_to_file_and_close();
void check_anomalous_motor_values();
void populate_motor_state(reflex_msgs::Hand* hand_msg, const reflex_hand::ReflexHandState* const state);

#endif
