#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <reflex_msgs/Hand.h>


void poseCallback(const reflex_msgs::HandConstPtr& msg) {
  static tf::TransformBroadcaster br;

  tf::Transform base_link_tf;
  tf::Transform swivel_tf[2];
  tf::Transform proximal_tf[3];
  tf::Transform distal_tf[3];
  tf::Transform proximal_sensor_tf[3][5];
  tf::Transform distal_sensor_tf[3][5];

  // Collect the geometry from reflex.yaml
  base_link_tf.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  base_link_tf.setRotation( tf::Quaternion(0.0, 0.0, 0.0) );

  std::vector<float> origin_param;
  std::vector<float> origin_param_2;
  std::vector<float> rotation_param;
  char s1[50]; char s2[50]; char s3[50];
  ros::NodeHandle node;

  for (int i=0; i<2; i++) {
    sprintf(s1, "tf_geometry/swivel_%d/origin", (i+1));
    node.getParam(s1, origin_param);
    swivel_tf[i].setOrigin(tf::Vector3(origin_param[0], origin_param[1], origin_param[2]));
    swivel_tf[i].setRotation(tf::Quaternion(0.0, 0.0, msg->motor[3].joint_angle * pow(-1, i)));
  }

  for (int i=0; i<3; i++) {
    sprintf(s1, "tf_geometry/proximal_%d/origin", (i+1));
    sprintf(s2, "tf_geometry/proximal_%d/rotation", (i+1));
    node.getParam(s1, origin_param);
    node.getParam(s2, rotation_param);
    proximal_tf[i].setOrigin(tf::Vector3(origin_param[0], origin_param[1], origin_param[2]));
    proximal_tf[i].setRotation(tf::Quaternion(msg->finger[i].proximal*pow(-1, floor(i/2.0)+1),
                                              rotation_param[1],
                                              rotation_param[2]));
  }

  node.getParam("tf_geometry/distal/origin", origin_param);
  for (int i=0; i<3; i++) {
    distal_tf[i].setOrigin(tf::Vector3(origin_param[0] + 0.008 * cos(msg->finger[i].distal_approx),
                                        origin_param[1],
                                        origin_param[2] + 0.008 * sin(msg->finger[i].distal_approx)));
    distal_tf[i].setRotation(tf::Quaternion(-msg->finger[i].distal_approx, 0.0, 0.0));
  }

  sprintf(s1, "tf_geometry/proximal_sensors/origin_x");
  sprintf(s2, "tf_geometry/proximal_sensors/origin_z");
  node.getParam(s1, origin_param);
  node.getParam(s2, origin_param_2);
  for (int i=0; i<3; i++) {
    for (int j=0; j<5; j++) {
      proximal_sensor_tf[i][j].setOrigin(tf::Vector3(origin_param[j], 0.0, origin_param_2[j]));
      proximal_sensor_tf[i][j].setRotation(tf::Quaternion(0.0, 0.0, 0.0));
    }
  }

  sprintf(s1, "tf_geometry/distal_sensors/origin_x");
  sprintf(s2, "tf_geometry/distal_sensors/origin_z");
  sprintf(s3, "tf_geometry/distal_sensors/rotation");
  node.getParam(s1, origin_param);
  node.getParam(s2, origin_param_2);
  node.getParam(s3, rotation_param);
  for (int i=0; i<3; i++) {
    for (int j=0; j<4; j++) {
      distal_sensor_tf[i][j].setOrigin( tf::Vector3(origin_param[j], 0.0, origin_param_2[j]) );
      distal_sensor_tf[i][j].setRotation( tf::Quaternion(rotation_param[j], 0.0, 0.0) );
    }
  }

  // Broadcast the transforms
  br.sendTransform(tf::StampedTransform(base_link_tf, ros::Time::now(), "base_link", "Base_Link"));

  for (int i=0; i<2; i++) {
    sprintf(s1, "Swivel_%d", (i+1));
    br.sendTransform(tf::StampedTransform(swivel_tf[i], ros::Time::now(), "Base_Link", s1));
  }

  for (int i=0; i<3; i++) {
    if (i==2)   {sprintf(s1, "Base_Link");}
    else        {sprintf(s1, "Swivel_%d", (i+1));}
    sprintf(s2, "Proximal_%d", (i+1));
    br.sendTransform(tf::StampedTransform(proximal_tf[i], ros::Time::now(), s1, s2));
  }

  for (int i=0; i<3; i++) {
    sprintf(s1, "Proximal_%d", (i+1));
    sprintf(s2, "Distal_%d", (i+1));
    br.sendTransform(tf::StampedTransform(distal_tf[i], ros::Time::now(), s1, s2));
  }

  for (int i=0; i<3; i++) {
    for (int j=0; j<5; j++) {
      sprintf(s1, "Proximal_%d", (i+1));
      sprintf(s2, "Proximal_%d/sensor_%d", (i+1), (j+1));
      br.sendTransform(tf::StampedTransform(proximal_sensor_tf[i][j], ros::Time::now(), s1, s2));
    }
  }

  for (int i=0; i<3; i++) {
    for (int j=0; j<4; j++) {
      sprintf(s1, "Distal_%d", (i+1));
      sprintf(s2, "Distal_%d/sensor_%d", (i+1), (j+1));
      br.sendTransform(tf::StampedTransform(distal_sensor_tf[i][j], ros::Time::now(), s1, s2));
    }
  }

}


int main(int argc, char** argv){
  ros::init(argc, argv, "reflex_tf_broadcaster");

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/reflex_hand", 10, &poseCallback);

  ros::spin();
  return 0;
};