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

#include <reflex_msgs2/Hand.h>
#include "./hand_visualizer.h"

#include <reflex_msgs2/DistalRotation.h>

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

  int total_joints = NUM_FIXED_STEPS + NUM_FINGERS * NUM_DOF * (NUM_FLEX_STEPS + 1);
  ROS_INFO("Number to resize: %d", total_joints);
  joint_state.name.resize(total_joints);
  joint_state.position.resize(total_joints);
  joint_state.name[0] ="proximal_joint_1";
  joint_state.name[1] ="proximal_joint_2";
  joint_state.name[2] ="proximal_joint_3";
  joint_state.name[3] ="preshape_1";
  joint_state.name[4] ="preshape_2";

  char buffer1[75], buffer2[75], buffer3[75];
  int index = NUM_FIXED_STEPS;
  for (int finger = 1; finger<4; finger++)
  {
    for (int i=1; i<(NUM_FLEX_STEPS+2); i++)
    {
      if (i == 1)
      {
        sprintf(buffer1, "finger[%d]/prox/flex_joint_from_prox_to_virtual_link[1]", finger);
        sprintf(buffer2, "finger[%d]/prox/flex_joint_from_virtual_link[1]_to_virtual_link[2]", finger);
        sprintf(buffer3, "finger[%d]/prox/flex_joint_from_virtual_link[2]_to_flex_link[1]", finger);
      }
      else if (i == (NUM_FLEX_STEPS+1))
      {
        sprintf(buffer1, "finger[%d]/flex_link[%d]/flex_joint_from_flex_link[%d]_to_virtual_link[1]", finger, NUM_FLEX_STEPS, NUM_FLEX_STEPS);
        sprintf(buffer2, "finger[%d]/flex_link[%d]/flex_joint_from_virtual_link[1]_to_virtual_link[2]", finger, NUM_FLEX_STEPS);
        sprintf(buffer3, "finger[%d]/flex_link[%d]/flex_joint_from_virtual_link[2]_to_dist", finger, NUM_FLEX_STEPS);
      }
      else
      {
        sprintf(buffer1, "finger[%d]/flex_link[%d]/flex_joint_from_flex_link[%d]_to_virtual_link[1]", finger, i-1, i-1);
        sprintf(buffer2, "finger[%d]/flex_link[%d]/flex_joint_from_virtual_link[1]_to_virtual_link[2]", finger, i-1);
        sprintf(buffer3, "finger[%d]/flex_link[%d]/flex_joint_from_virtual_link[2]_to_flex_link[%d]", finger, i-1, i);
      }
      joint_state.name[index] = buffer1;
      joint_state.name[index + 1] = buffer2;
      joint_state.name[index + 2] = buffer3;
      index += 3;
    }
  }

  ros::Publisher pub = n.advertise<reflex_msgs2::Hand>("/reflex_sf/hand_state", 10);
  ros::Duration(.5).sleep();
  
  ros::ServiceClient distal_rotation_client = n.serviceClient<reflex_msgs2::DistalRotation>("distal_rotation", true);
  while(!distal_rotation_client) {
    ROS_INFO("Failed to connect to client 'distal_rotation_client'");
    ros::Duration(.5).sleep();
    distal_rotation_client = n.serviceClient<reflex_msgs2::DistalRotation>("distal_rotation", true);
  }
  
  ros::Subscriber takktile_sub =
    n.subscribe<reflex_msgs2::Hand>(
      "/reflex_takktile2/hand_state", 10, boost::bind(publish_takktile_to_rviz, _1, &distal_rotation_client)
    );
  ros::Subscriber sf_sub = n.subscribe("/reflex_sf/hand_state", 10, publish_sf_to_rviz);
  
  // Zero the hand and make it appear open. The sleeps are to let RVIZ start
  ros::Duration(2.0).sleep();
  reflex_msgs2::Hand base_hand_state;
  for (int i=0; i<10; i++) {
    pub.publish(base_hand_state);
    ros::Duration(0.5).sleep();
  }

  ros::spin();
  return 0;
}


void publish_takktile_to_rviz(const reflex_msgs2::HandConstPtr& hand, ros::ServiceClient* client) {
  publish_finger_to_rviz(hand, client);
  publish_sensors_to_rviz(hand);
}


void publish_sf_to_rviz(const reflex_msgs2::HandConstPtr& hand) {
  publish_finger_to_rviz_sf(hand);
}

void publish_finger_to_rviz_sf(const reflex_msgs2::HandConstPtr& hand) {
  joint_state.header.stamp = ros::Time::now();
  joint_state.position[0] = hand->motor[0].joint_angle;
  joint_state.position[1] = hand->motor[1].joint_angle;
  joint_state.position[2] = hand->motor[2].joint_angle;
  joint_state.position[3] = hand->motor[3].joint_angle;
  joint_state.position[4] = -hand->motor[3].joint_angle;
  int index = NUM_FIXED_STEPS;
  for (int finger = 0; finger < 3; finger++)
  {
    for (int i = 0; i < (NUM_FLEX_STEPS + 1); i++)
    {
      joint_state.position[index] = hand->finger[finger].distal_approx/((float) (NUM_FLEX_STEPS+1));
      joint_state.position[index + 1] = 0;
      joint_state.position[index + 2] = 0;
      index += 3;
    }
  }
  joint_pub.publish(joint_state);
}

void publish_finger_to_rviz(const reflex_msgs2::HandConstPtr& hand, ros::ServiceClient* client) {
  joint_state.header.stamp = ros::Time::now();
  joint_state.position[0] = hand->finger[0].proximal;
  joint_state.position[1] = hand->finger[1].proximal;
  joint_state.position[2] = hand->finger[2].proximal;
  joint_state.position[3] = hand->motor[3].joint_angle;
  joint_state.position[4] = -hand->motor[3].joint_angle;

  reflex_msgs2::DistalRotation srv;

  int index = NUM_FIXED_STEPS;
  for (int finger = 0; finger<3; finger++)
  {
    float joint_angle = 0;
    if (finger == 0)
      joint_angle = hand->motor[3].joint_angle;
    else if (finger == 1)
      joint_angle = -hand->motor[3].joint_angle;
    else
      joint_angle = 3.14159265359;

    srv.request.palm_imu_quat = hand->palmImu.quat;
    srv.request.joint_angle = joint_angle;
    srv.request.proximal = hand->finger[finger].proximal;
    srv.request.finger_imu_quat = hand->finger[finger].imu.quat;
    if (client->call(srv))
    {
      for (int i = 0; i < (NUM_FLEX_STEPS+1); i++)
      {
        joint_state.position[index] = srv.response.rotation[0]/((float) (NUM_FLEX_STEPS+1)) + ROLL_OFFSET; //roll
        joint_state.position[index + 1] = srv.response.rotation[1]/((float) (NUM_FLEX_STEPS+1)) + PITCH_OFFSET; //pitch
        joint_state.position[index + 2] = srv.response.rotation[2]/((float) (NUM_FLEX_STEPS+1)) + YAW_OFFSET; //yaw
        index += 3;
      }
    }
    else
    {
      ROS_ERROR("Failed to call service distal_rotation");
    }
  }
  joint_pub.publish(joint_state);
}


void publish_sensors_to_rviz(const reflex_msgs2::HandConstPtr& hand) {
  bool contact_val;
  float pressure_val;
  visualization_msgs::MarkerArray marker_array;

  for (int finger=0; finger<3; finger++)
  {
    char prox_fid[20];
    sprintf(prox_fid, "/proximal_%d_tactile", (finger+1));
    char dist_fid[20];
    sprintf(dist_fid, "/distal_%d_tactile", (finger+1));

    for (int i = 0; i < SENSORS_PER_FINGER; i++)    // Loop through tactile sensors in the fingers
    {
      contact_val = hand->finger[finger].contact[i];
      pressure_val = hand->finger[finger].pressure[i];
      visualization_msgs::Marker contact_marker = makeContactMarker(contact_val, i);
      visualization_msgs::Marker pressure_marker = makePressureMarker(pressure_val, i);
      if (i < 5) {    // Proximal link
        contact_marker.header.frame_id = prox_fid;
        pressure_marker.header.frame_id = prox_fid;
      } 
      else {      // Distal link
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

  finger_tactile_positions(id, &marker.pose.position.x, &marker.pose.position.z, &marker.pose.position.y, &marker.pose.orientation.w);
  if (id == 8){
    marker.pose.orientation.y = 2.3;
  }
  else if (id == 9){
    marker.pose.orientation.y = -2.5;
  }
  else if (id > 9){
    marker.pose.orientation.x = 4.55;
  }
  marker.color.a = 1.0;
  return marker;
}


void finger_tactile_positions(int index, double* x, double* z, double* y, double* w) {
  //TODO: Make more dynamic/ easy to change position locations (sensors 0-8 are ok)
  double x_gap = 0.008;
  if (index < 5) {  // Proximal link
    *x = x_gap*index + 0.019;
    *z = 0.014;
    *y = 0;
    *w = 1.0;
  }
  else if ((index > 4) & (index < 8)) {        // Distal link
    *x = x_gap*(index-5) + 0.0245;
    *z = 0.010;
    *y = 0;
    *w = 1.0;
  }
  else if (index == 8){
    *x = x_gap*(index+1-5) + 0.019;
    *z = 0.007;
    *y = 0;
    *w = 5.0;
  }
  else if (index == 9){
    *x = x_gap*(index-5-1+1) + 0.0205;
    *z = -0.0025;
    *y = 0;
    *w = 5.0;
  }
  else if (index == 10){
    *x = x_gap*(6-5) + 0.0215;
    *z = 0.001;
    *y = -0.0075;
    *w = 5.0;
  }
  else if (index == 11){
    *x = x_gap*(7-5) + 0.0215;
    *z = 0.001;
    *y = -0.0075;
    *w = 5.0;
  }
  else if (index == 12){
    *x = x_gap*(6-5) + 0.0215;
    *z = 0.001;
    *y = 0.0075;
    *w = 5.0;
  }
  else if (index == 13){
    *x = x_gap*(7-5) + 0.0215;
    *z = 0.001;
    *y = 0.0075;
    *w = 5.0;
  }
}
