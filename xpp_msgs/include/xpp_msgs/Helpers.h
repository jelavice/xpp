/*
 * Helpers.h
 *
 *  Created on: Jan 9, 2019
 *      Author: jelavice
 */

#pragma once


#include <xpp_msgs/StateLin3d.h>
#include <xpp_msgs/State6d.h>
#include <xpp_msgs/RobotStateCartesian.h>
#include <xpp_msgs/RobotStateCartesianPlusJoints.h>
#include <xpp_msgs/RobotStateCartesianTrajectory.h>


namespace xpp_msgs {


RobotStateCartesian ExtractCartesianState(const RobotStateCartesianPlusJoints &msg){

  RobotStateCartesian retMsg;
  retMsg.base = msg.base;
  retMsg.ee_contact = msg.ee_contact;
  retMsg.ee_motion = msg.ee_motion;
  retMsg.ee_forces = msg.ee_forces;
  retMsg.time_from_start = msg.time_from_start;

  return retMsg;

}


} /* namespace */
