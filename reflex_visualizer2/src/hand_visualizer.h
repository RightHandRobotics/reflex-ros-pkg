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

#ifndef HAND_VISUALIZER_H
#define HAND_VISUALIZER_H

#define NUM_DOF             3
#define NUM_FINGERS         3
#define NUM_FIXED_STEPS     5
#define NUM_FLEX_STEPS      9
#define SENSORS_PER_FINGER  14

#define ROLL_OFFSET			0.0
#define PITCH_OFFSET		0.0
#define YAW_OFFSET			0.0

void publish_takktile_to_rviz(const reflex_msgs2::HandConstPtr& hand, ros::ServiceClient* client);
void publish_sf_to_rviz(const reflex_msgs2::HandConstPtr& hand);
void publish_finger_to_rviz(const reflex_msgs2::HandConstPtr& hand, ros::ServiceClient* client);
void publish_finger_to_rviz_sf(const reflex_msgs2::HandConstPtr& hand);
void publish_sensors_to_rviz(const reflex_msgs2::HandConstPtr& hand);
void finger_tactile_positions(int index, double* x, double* z, double* y, double* w);

visualization_msgs::Marker makeContactMarker(bool val, int id);
visualization_msgs::Marker makePressureMarker(float val, int id);
visualization_msgs::Marker makeFingerMarker(int id);

#endif
