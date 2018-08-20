/*
 * M545JointConverter.h
 *
 *  Created on: Aug 20, 2018
 *      Author: jelavice
 */

#pragma once


#include <string>

#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <xpp_msgs/RobotStateJoint.h>

//todo this depends on the m545 State
namespace xpp {


class M545StateConverter {
public:

  M545StateConverter (const std::string& towr_state_topic,
                           const std::string& m545_state_topic);
  virtual ~M545StateConverter () = default;

private:

  //todo implement this guy
  //it gets the message copies all the crap
  // and the n it just add default values for joints that are not optimized over
  // e.g. claw, ee_yaw etc
  void StateCallback(const xpp_msgs::RobotStateJoint& msg);

  ros::Subscriber xpp_state_sub_;
  ros::Publisher  m545_state_pub_;

};

} /* namespace xpp */
