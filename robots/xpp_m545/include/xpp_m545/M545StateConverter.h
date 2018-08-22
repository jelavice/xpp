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

#include <xpp_msgs/RobotStateCartesianPlusJoints.h>

#include "excavator_model/ExcavatorModel.hpp"

//todo this depends on the m545 State
namespace xpp {

class M545StateConverter
{
 public:

  M545StateConverter(const std::string& xpp_state_topic, const std::string& m545_state_topic);
  virtual ~M545StateConverter() = default;

 private:

  void StateCallback(const xpp_msgs::RobotStateCartesianPlusJoints& msg);

  void FillKindrVector(const xpp_msgs::RobotStateCartesianPlusJoints& msg,
                       excavator_model::JointPositions &kindrVec, int starting_index_kindr,
                       int ee_id);

  ros::Subscriber xpp_state_sub_;
  ros::Publisher m545_state_pub_;

};

} /* namespace xpp */
