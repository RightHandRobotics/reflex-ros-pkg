# Acceleration offsets for each dimension in the format [x,y,z]
uint16[3] acc_offset_f1
uint16[3] acc_offset_f2
uint16[3] acc_offset_f3
uint16[3] acc_offset_palm

# Magnetometer offsets for each dimension in the format [x,y,z]
uint16[3] mag_offset_f1
uint16[3] mag_offset_f2
uint16[3] mag_offset_f3
uint16[3] mag_offset_palm

# Gyroscope offsets for each dimension in the format [x,y,z]
uint16[3] gyr_offset_f1
uint16[3] gyr_offset_f2
uint16[3] gyr_offset_f3
uint16[3] gyr_offset_palm

# Accelerometer and Gyroscope radius in the format [f1,f2,f3,palm]
uint16[4] acc_radius
uint16[4] gyr_radius