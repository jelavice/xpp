/*
 * M545JointConverter.h
 *
 *  Created on: Aug 20, 2018
 *      Author: jelavice
 */

#pragma once

#include <string>

#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <xpp_msgs/RobotStateJoint.h>

#include "excavator_model/ExcavatorModel.hpp"

//todo this depends on the m545 State
namespace xpp {

class M545StateConverter
{
 public:

  M545StateConverter(const std::string& xpp_state_topic, const std::string& m545_state_topic);
  virtual ~M545StateConverter() = default;

 private:

  void StateCallback(const xpp_msgs::RobotStateJoint& msg);

  void FillKindrVector(const xpp_msgs::RobotStateJoint& msg,
                       excavator_model::JointPositions &kindrVec,
                       int starting_index_kindr,
                       int starting_index_xpp,
                       int numJoints);

  ros::Subscriber xpp_state_sub_;
  ros::Publisher m545_state_pub_;

};

} /* namespace xpp */
