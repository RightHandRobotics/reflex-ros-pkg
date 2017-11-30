# quaternion reading from IMU (w, x, y, z)
float32[4] quat	
float32[3] euler_angles 

# these are defined in reflex_hand.h driver
uint8 calibration_status 
uint16[11] calibration_data
