#include <ros/ros.h>
#include <reflex_msgs/Hand.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <string>
using namespace std;

//int palm_map[] = {5, 7, 6, 8, 2, 3, 8, 9, 9, 0, 1};
int palm_map[] = {5, 8, 6, 7, 2, 3, 8, 9, 9, 0, 1};

ros::Publisher joint_pub;
ros::Publisher sensor_pub;

// Joint variable instantiation
sensor_msgs::JointState joint_state;
int num_fixed_steps = 5;
int num_flex_steps = 9;
void publish_to_rviz(const reflex_msgs::HandConstPtr& hand);

// Pressure sensor instantiation
visualization_msgs::Marker makeContactMarker(bool val, int id, float radius, float height, bool finger);
visualization_msgs::Marker makePressureMarker(float val, int id, float radius, float height, bool finger);
visualization_msgs::Marker makeFingerMarker(int id);
visualization_msgs::Marker makePalmMarker(int id);
void finger_tactile_positions(int index, double* x, double* z);
const int sensors_per_finger = 9;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "rhr_hand_visualizer");
  ros::NodeHandle n;
  joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
  sensor_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);

  ROS_INFO("Number to resize: %d", num_fixed_steps + 3*(num_flex_steps+1));
  joint_state.name.resize(num_fixed_steps + 3*(num_flex_steps+1));
  joint_state.position.resize(num_fixed_steps + 3*(num_flex_steps+1));
  joint_state.name[0] ="proximal_joint_1";
  joint_state.name[1] ="proximal_joint_2";
  joint_state.name[2] ="proximal_joint_3";
  joint_state.name[3] ="preshape_1";
  joint_state.name[4] ="preshape_2";

  char buffer[50];
  int index = num_fixed_steps;
  for (int finger = 1; finger<4; finger++)
  {
    for (int i=1; i<(num_flex_steps+2); i++)
    {
      if (i == 1)
        sprintf(buffer, "finger[%d]/flex_joint_from_prox_to_1", finger);
      else if (i == (num_flex_steps+1))
        sprintf(buffer, "finger[%d]/flex_joint_from_%d_to_dist", finger, num_flex_steps);
      else
        sprintf(buffer, "finger[%d]/flex_joint_from_%d_to_%d", finger, i-1, i);
      joint_state.name[index] = buffer;
      index++;
    }
  }

  ros::Subscriber sub = n.subscribe("/reflex_hand", 1, publish_to_rviz);

  ros::spin();
  return 0;
}


/**
 * This function takes the Hand data published by the RightHandRobotics hand
 * and sends that information in joint format to an rviz visualizer
 */
 void publish_to_rviz(const reflex_msgs::HandConstPtr& hand)
 {
  // Pressure sensor setup
  bool contact_val;
  float pressure_val;
  visualization_msgs::MarkerArray marker_array;

  // Finger joint assignment
  joint_state.header.stamp = ros::Time::now();
  joint_state.position[0] = hand->finger[0].proximal;
  joint_state.position[1] = hand->finger[1].proximal;
  joint_state.position[2] = hand->finger[2].proximal;
  joint_state.position[3] = hand->palm.preshape;
  joint_state.position[4] = -hand->palm.preshape;

  int index = num_fixed_steps;
  for (int finger = 0; finger<3; finger++)
  {
    for (int i=0; i<(num_flex_steps+1); i++)
    {
      joint_state.position[index] = hand->finger[finger].distal/((float) (num_flex_steps+1));
      index++;
    }
  }

  // Pressure sensor assignment
  for (int finger=0; finger<3; finger++)
  {
    char prox_fid[20];
    sprintf(prox_fid, "/proximal_%d_tactile", (finger+1));
    char dist_fid[20];
    sprintf(dist_fid, "/distal_%d_tactile", (finger+1));

    for (int i=0; i<sensors_per_finger; i++)    // Loop through tactile sensors in the fingers
    {
      contact_val = hand->finger[finger].contact[i];
      pressure_val = hand->finger[finger].pressure[i];
      visualization_msgs::Marker contact_marker = makeContactMarker(contact_val, i, 0.004, 0.005, true);
      visualization_msgs::Marker pressure_marker = makePressureMarker(pressure_val, i, 0.008, 0.003, true);
      if (i < 5) {    // Proximal link
        contact_marker.header.frame_id = prox_fid;
        pressure_marker.header.frame_id = prox_fid;
      } else {      // Distal link
        contact_marker.header.frame_id = dist_fid;
        pressure_marker.header.frame_id = dist_fid;
      }
      contact_marker.id = i + (finger*sensors_per_finger);
      pressure_marker.id = i + (finger*sensors_per_finger);
      marker_array.markers.push_back(contact_marker);
      marker_array.markers.push_back(pressure_marker);
    }
  }

  // Loop through tactile sensors in the palm
  for (int i=0; i<11; i++)
  {
    contact_val = hand->palm.contact[palm_map[i]];
    pressure_val = hand->palm.pressure[palm_map[i]];
    visualization_msgs::Marker contact_marker = makeContactMarker(contact_val, i, 0.004, 0.01, false);
    visualization_msgs::Marker pressure_marker = makePressureMarker(pressure_val, i, 0.009, 0.008, false);
    marker_array.markers.push_back(contact_marker);
    marker_array.markers.push_back(pressure_marker);
  }

  // Publish joint data
  joint_pub.publish(joint_state);
  // Publish sensor markers
  sensor_pub.publish(marker_array);
}


visualization_msgs::Marker makeContactMarker(bool val, int id, float radius, float height, bool finger)
{
  visualization_msgs::Marker marker;
  if (finger)
    marker = makeFingerMarker(id);
  else
    marker = makePalmMarker(id);
  marker.ns = "contact_markers";

  if (val) {
    marker.scale.x = radius;
    marker.scale.y = radius;
    marker.scale.z = height;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
  }
  else {
    marker.scale.x = radius/2;
    marker.scale.y = radius/2;
    marker.scale.z = height-0.001;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
  }
  return marker;
}


visualization_msgs::Marker makePressureMarker(float val, int id, float radius, float height, bool finger)
{
  visualization_msgs::Marker marker;
  if (finger)
    marker = makeFingerMarker(id);
  else
    marker = makePalmMarker(id);
  marker.ns = "pressure_markers";

  marker.scale.x = radius;
  marker.scale.y = radius*(4.0/5);
  marker.scale.z = height;

  // Assuming 400 is the max sensor value, that could be replaced with some MAX value
  val = max(min((0.6*abs(val)/-100) + 0.6, 1.0), 0.0);
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


  visualization_msgs::Marker makePalmMarker(int id)
  {
    float pos1[3] = {-0.0235,  -0.0315,  0.082};
    float pos2[3] = {-0.0235 ,  0.0315,   0.082};
    float pos3[3] = {0.026,   0,      0.082};
    float x_gap = 0.028;
    float y_gap = 0.004;
    float x[11] = {pos1[0], pos1[0], pos1[0]+x_gap, pos1[0]+x_gap, pos2[0], pos2[0], pos2[0]+x_gap, pos2[0]+x_gap, pos3[0], pos3[0], pos3[0]+x_gap};
    float y[11] = {pos1[1], pos1[1]+y_gap, pos1[1], pos1[1]+y_gap, pos2[1], pos2[1]-y_gap, pos2[1], pos2[1]-y_gap, pos3[1]-0.5*y_gap, pos3[1]+0.5*y_gap, pos3[1]};
    float z[11] = {pos1[2], pos1[2], pos1[2], pos1[2], pos2[2], pos2[2], pos2[2], pos2[2], pos3[2], pos3[2], pos3[2]};

    visualization_msgs::Marker marker;
    marker.ns = "palm_markers";
    marker.header.stamp = ros::Time();
    marker.header.frame_id = "base_tactile";
    marker.id = id + 3*sensors_per_finger;
    marker.type = visualization_msgs::Marker::SPHERE;

    marker.pose.position.x = x[id];
    marker.pose.position.y = y[id];
    marker.pose.position.z = z[id];
    marker.pose.orientation.w = 1.0;
    marker.color.a = 1.0;

    return marker;
  }


  void finger_tactile_positions(int index, double* x, double* z)
  {
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
