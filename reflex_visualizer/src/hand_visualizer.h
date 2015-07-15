#ifndef HAND_VISUALIZER_H
#define HAND_VISUALIZER_H

#define NUM_FIXED_STEPS     5
#define NUM_FLEX_STEPS      9
#define SENSORS_PER_FINGER  9

void publish_takktile_to_rviz(const reflex_msgs::HandConstPtr& hand);
void publish_sf_to_rviz(const reflex_msgs::HandConstPtr& hand);
void publish_finger_to_rviz(const reflex_msgs::HandConstPtr& hand, bool takktile);
void publish_sensors_to_rviz(const reflex_msgs::HandConstPtr& hand);
void finger_tactile_positions(int index, double* x, double* z);
visualization_msgs::Marker makeContactMarker(bool val, int id);
visualization_msgs::Marker makePressureMarker(float val, int id);
visualization_msgs::Marker makeFingerMarker(int id);

#endif