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

#include <string>

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <reflex_msgs/Hand.h>
#include "./hand_visualizer.h"

using namespace std;


ros::Publisher joint_pub;
ros::Publisher sensor_pub;
sensor_msgs::JointState joint_state;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "rhr_hand_visualizer");
  ros::NodeHandle n;
  joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
  sensor_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);

  ROS_INFO("Number to resize: %d", NUM_FIXED_STEPS + 3 * (NUM_FLEX_STEPS + 1));
  joint_state.name.resize(NUM_FIXED_STEPS + 3 * (NUM_FLEX_STEPS + 1));
  joint_state.position.resize(NUM_FIXED_STEPS + 3 * (NUM_FLEX_STEPS + 1));
  joint_state.name[0] ="proximal_joint_1";
  joint_state.name[1] ="proximal_joint_2";
  joint_state.name[2] ="proximal_joint_3";
  joint_state.name[3] ="preshape_1";
  joint_state.name[4] ="preshape_2";

  char buffer[50];
  int index = NUM_FIXED_STEPS;
  for (int finger = 1; finger<4; finger++)
  {
    for (int i=1; i<(NUM_FLEX_STEPS+2); i++)
    {
      if (i == 1)
        sprintf(buffer, "finger[%d]/flex_joint_from_prox_to_1", finger);
      else if (i == (NUM_FLEX_STEPS+1))
        sprintf(buffer, "finger[%d]/flex_joint_from_%d_to_dist", finger, NUM_FLEX_STEPS);
      else
        sprintf(buffer, "finger[%d]/flex_joint_from_%d_to_%d", finger, i-1, i);
      joint_state.name[index] = buffer;
      index++;
    }
  }

  ros::Publisher pub = n.advertise<reflex_msgs::Hand>("/reflex_sf/hand_state", 10);
  ros::Subscriber takktile_sub = n.subscribe("/reflex_takktile/hand_state", 10, publish_takktile_to_rviz);
  ros::Subscriber sf_sub = n.subscribe("/reflex_sf/hand_state", 10, publish_sf_to_rviz);
  
  // Zero the hand and make it appear open. The sleeps are to let RVIZ start
  ros::Duration(2.0).sleep();
  reflex_msgs::Hand base_hand_state;
  for (int i=0; i<10; i++) {
    pub.publish(base_hand_state);
    ros::Duration(0.5).sleep();
  }

  ros::spin();
  return 0;
}


void publish_takktile_to_rviz(const reflex_msgs::HandConstPtr& hand) {
  publish_finger_to_rviz(hand, true);
  publish_sensors_to_rviz(hand);
}


void publish_sf_to_rviz(const reflex_msgs::HandConstPtr& hand) {
  publish_finger_to_rviz(hand, false);
}


void publish_finger_to_rviz(const reflex_msgs::HandConstPtr& hand, bool takktile) {
  joint_state.header.stamp = ros::Time::now();
  if (takktile) {
    joint_state.position[0] = hand->finger[0].proximal;
    joint_state.position[1] = hand->finger[1].proximal;
    joint_state.position[2] = hand->finger[2].proximal;
  } else {
    joint_state.position[0] = hand->motor[0].joint_angle;
    joint_state.position[1] = hand->motor[1].joint_angle;
    joint_state.position[2] = hand->motor[2].joint_angle;
  }
  joint_state.position[3] = hand->motor[3].joint_angle;
  joint_state.position[4] = -hand->motor[3].joint_angle;

  int index = NUM_FIXED_STEPS;
  for (int finger = 0; finger<3; finger++)
  {
    for (int i=0; i<(NUM_FLEX_STEPS+1); i++)
    {
      joint_state.position[index] = hand->finger[finger].distal_approx/((float) (NUM_FLEX_STEPS+1));
      index++;
    }
  }
  joint_pub.publish(joint_state);
}


void publish_sensors_to_rviz(const reflex_msgs::HandConstPtr& hand) {
  bool contact_val;
  float pressure_val;
  visualization_msgs::MarkerArray marker_array;

  for (int finger=0; finger<3; finger++)
  {
    char prox_fid[20];
    sprintf(prox_fid, "/proximal_%d_tactile", (finger+1));
    char dist_fid[20];
    sprintf(dist_fid, "/distal_%d_tactile", (finger+1));

    for (int i=0; i<SENSORS_PER_FINGER; i++)    // Loop through tactile sensors in the fingers
    {
      contact_val = hand->finger[finger].contact[i];
      pressure_val = hand->finger[finger].pressure[i];
      visualization_msgs::Marker contact_marker = makeContactMarker(contact_val, i);
      visualization_msgs::Marker pressure_marker = makePressureMarker(pressure_val, i);
      if (i < 5) {    // Proximal link
        contact_marker.header.frame_id = prox_fid;
        pressure_marker.header.frame_id = prox_fid;
      } else {      // Distal link
        contact_marker.header.frame_id = dist_fid;
        pressure_marker.header.frame_id = dist_fid;
      }
      contact_marker.id = i + (finger * SENSORS_PER_FINGER);
      pressure_marker.id = i + (finger * SENSORS_PER_FINGER);
      marker_array.markers.push_back(contact_marker);
      marker_array.markers.push_back(pressure_marker);
    }
  }

  sensor_pub.publish(marker_array);
}


visualization_msgs::Marker makeContactMarker(bool val, int id)
{
  visualization_msgs::Marker marker;
  marker = makeFingerMarker(id);
  marker.ns = "contact_markers";
  float radius = 0.004;
  float height = 0.005;

  if (val) {
    marker.scale.x = radius;
    marker.scale.y = radius;
    marker.scale.z = height;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
  }
  else {
    marker.scale.x = radius / 2;
    marker.scale.y = radius / 2;
    marker.scale.z = height - 0.001;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
  }
  return marker;
}


visualization_msgs::Marker makePressureMarker(float val, int id)
{
  visualization_msgs::Marker marker;
  marker = makeFingerMarker(id);
  marker.ns = "pressure_markers";
  float radius = 0.008;
  float height = 0.003;

  marker.scale.x = radius;
  marker.scale.y = radius * (4.0 / 5);
  marker.scale.z = height;

  // Assuming 80 is the max sensor value, that could be replaced with some MAX value
  val = max(min((val / -100.0) + 0.8, 1.0), 0.0);
  marker.color.r = val;
  marker.color.g = val;
  marker.color.b = val;
  return marker;
}


visualization_msgs::Marker makeFingerMarker(int id)
{
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time();
  marker.type = visualization_msgs::Marker::SPHERE;

  finger_tactile_positions(id, &marker.pose.position.x, &marker.pose.position.z);
  marker.pose.position.y = 0;
  marker.pose.orientation.w = 1.0;
  marker.color.a = 1.0;
  return marker;
}


void finger_tactile_positions(int index, double* x, double* z) {
  double x_gap = 0.008;
  if (index < 5) {  // Proximal link
    *x = x_gap*index + 0.019;
    *z = 0.015;
  }
  else {        // Distal link
    *x = x_gap*(index-5) + 0.0215;
    *z = 0.009;
  }
}
