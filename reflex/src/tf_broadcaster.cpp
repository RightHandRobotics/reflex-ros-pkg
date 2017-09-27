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
The Transform_broadcaster class provides an easy way to publish coordinate frame transform information. 
It will handle all the messaging and stuffing of messages. And the function prototypes
lay out all the necessary data needed for each message. 

The Quaternion class implements quaternion to perform linear algebra rotations in combination with 
Matrix3x3, Vector3 and Transform.
*/

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
  
  /* COMPILER WARNING
  
  warning: ‘tf::Quaternion::Quaternion(const tfScalar&, const tfScalar&, const tfScalar&)’ 
  is deprecated (declared at /opt/ros/jade/include/tf/LinearMath/Quaternion.h:51) [-Wdeprecated-declarations]
  base_link_tf.setRotation( tf::Quaternion(0.0, 0.0, 0.0) );

  ///////////// Line 51 in /opt/ros/jade/include/tf/LinearMath/Quaternion.h:51
  https://github.com/davheld/tf/blob/master/include/tf/LinearMath/Quaternion.h

  **@brief Constructor from Euler angles
   * @param yaw Angle around Y unless TF_EULER_DEFAULT_ZYX defined then Z
   * @param pitch Angle around X unless TF_EULER_DEFAULT_ZYX defined then Y
   * @param roll Angle around Z unless TF_EULER_DEFAULT_ZYX defined then X

  Quaternion(const tfScalar& yaw, const tfScalar& pitch, const tfScalar& roll) __attribute__((deprecated))
  { 
    #ifndef TF_EULER_DEFAULT_ZYX
    
    setEuler(yaw, pitch, roll); 
    
    #else
    
    setRPY(roll, pitch, yaw);
    
    #endif 
  }

  // What the heck does __attribute__((deprecated)) mean???????????????????????????
    http://www.keil.com/support/man/docs/armcc/armcc_chr1359124981701.htm
        The deprecated variable attribute enables the declaration of a deprecated variable without any warnings or errors 
        being issued by the compiler. However, any access to a deprecated variable creates a warning but still compiles.
        The warning gives the location where the variable is used and the location where it is defined. 
        This helps you to determine why a particular definition is deprecated.
  // What is a deprecated attribute?
        https://www.ibm.com/support/knowledgecenter/en/SSXVZZ_13.1.2/com.ibm.xlcpp1312.lelinux.doc/language_ref/var_attrib_deprecated.html
        If a variable that is specified with the deprecated attribute is used, the compiler issues a warning message 
        to indicate that the variable is not recommended to be used.
        Warning messages are issued only for uses but not declarations of deprecated variables. 
  // What is a deprecated variable? What is its purpose?

  Quaternion function returns a normalized version of this quaternion.  
  What is a tfScalar datatype?

  
  */


  base_link_tf.setRotation( tf::Quaternion(0.0, 0.0, 0.0) );

  std::vector<float> origin_param;
  std::vector<float> origin_param_2;

  // Getting compiler warning
  std::vector<float> rotation_param;

  char s1[50]; char s2[50]; char s3[50];
  ros::NodeHandle node;

  for (int i=0; i<2; i++) {
    sprintf(s1, "tf_geometry/swivel_%d/origin", (i+1));
    node.getParam(s1, origin_param);
    swivel_tf[i].setOrigin(tf::Vector3(origin_param[0], origin_param[1], origin_param[2]));
    
    /* Getting compiler warning
    
      What is swivel_tf[] array?
        - Local tf::Transform object defined above 
            tf::Transform swivel_tf[2]; line 38
      What is setRotation()?
        - Not defined in this file, only found in this file. Must be include
        - Something in tf probably
      What is msg->motor[] array?
        - const reflex_msgs::HandConstPtr& msg pass by reference
        - motor[] is only used in this function call below
      What is joint_angle attribute of above array?
        - There is some math done on 3rd argument
  
    */


    swivel_tf[i].setRotation(tf::Quaternion(0.0, 0.0, msg->motor[3].joint_angle * pow(-1, i)));
  }

  for (int i=0; i<3; i++) {
    sprintf(s1, "tf_geometry/proximal_%d/origin", (i+1));
    sprintf(s2, "tf_geometry/proximal_%d/rotation", (i+1));
    node.getParam(s1, origin_param);
    node.getParam(s2, rotation_param);
    proximal_tf[i].setOrigin(tf::Vector3(origin_param[0], origin_param[1], origin_param[2]));
    
    // Getting compiler warning
    proximal_tf[i].setRotation(tf::Quaternion(msg->finger[i].proximal*pow(-1, floor(i/2.0)+1),
                                              rotation_param[1],
                                              rotation_param[2]));
  }

  node.getParam("tf_geometry/distal/origin", origin_param);
  for (int i = 0; i < 3; i++) {
    distal_tf[i].setOrigin(tf::Vector3(origin_param[0] + 0.008 * cos(msg->finger[i].distal_approx),
                                        origin_param[1],
                                        origin_param[2] + 0.008 * sin(msg->finger[i].distal_approx)));
    
    // Getting compiler warning
    distal_tf[i].setRotation(tf::Quaternion(-msg->finger[i].distal_approx, 0.0, 0.0));
  }

  sprintf(s1, "tf_geometry/proximal_sensors/origin_x");
  sprintf(s2, "tf_geometry/proximal_sensors/origin_z");
  node.getParam(s1, origin_param);
  node.getParam(s2, origin_param_2);

  for (int i = 0; i < 3; i++) {
    for (int j =0 ; j < 5; j++) {
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

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 4; j++) {
      distal_sensor_tf[i][j].setOrigin( tf::Vector3(origin_param[j], 0.0, origin_param_2[j]) );
      
      // Getting compiler warning
      distal_sensor_tf[i][j].setRotation( tf::Quaternion(rotation_param[j], 0.0, 0.0) );
    }
  }

  // Broadcast transforms
  br.sendTransform(tf::StampedTransform(base_link_tf, ros::Time::now(), "base_link", "Base_Link"));

  for (int i = 0; i < 2; i++) {
    sprintf(s1, "Swivel_%d", (i+1));
    br.sendTransform(tf::StampedTransform(swivel_tf[i], ros::Time::now(), "Base_Link", s1));
  }

  for (int i = 0; i < 3; i++) {
    if (i == 2)   {sprintf(s1, "Base_Link");}
    else        {sprintf(s1, "Swivel_%d", (i+1));}
    sprintf(s2, "Proximal_%d", (i+1));
    br.sendTransform(tf::StampedTransform(proximal_tf[i], ros::Time::now(), s1, s2));
  }

  for (int i = 0; i < 3; i++) {
    sprintf(s1, "Proximal_%d", (i+1));
    sprintf(s2, "Distal_%d", (i+1));
    br.sendTransform(tf::StampedTransform(distal_tf[i], ros::Time::now(), s1, s2));
  }

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 5; j++) {
      sprintf(s1, "Proximal_%d", (i+1));
      sprintf(s2, "Proximal_%d/sensor_%d", (i+1), (j+1));
      br.sendTransform(tf::StampedTransform(proximal_sensor_tf[i][j], ros::Time::now(), s1, s2));
    }
  }

  for (int i = 0; i < 3 ; i++) {
    for (int j = 0; j < 4; j++) {
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