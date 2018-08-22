/*
 * visualizer_m545.cc
 *
 *  Created on: Aug 20, 2018
 *      Author: jelavice
 */

#include <iostream>
#include <map>
#include <memory>
#include <string>

#include <ros/init.h>

#include <xpp_msgs/topic_names.h>
#include <xpp_states/joints.h>
#include <xpp_states/endeffector_mappings.h>

#include <xpp_vis/cartesian_joint_converter.h>
#include <xpp_vis/urdf_visualizer.h>

#include <xpp_m545/M545StateConverter.h>


using namespace xpp;
using namespace quad;

int main(int argc, char *argv[])
{
  ::ros::init(argc, argv, "urdf_m545_visualizer");


  std::string xpp_state_topic = xpp_msgs::joint_desired;
  std::string m545_state_topic = "/m545_state";
  M545StateConverter m545_state_converter(xpp_state_topic, m545_state_topic);


  ::ros::spin();

  return 0;
}



