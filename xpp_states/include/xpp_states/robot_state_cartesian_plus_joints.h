/*
 * robot_state_cartesian_plus_joints.h
 *
 *  Created on: Aug 22, 2018
 *      Author: jelavice
 */

#pragma once

#include "xpp_states/robot_state_cartesian.h"

namespace xpp {

class RobotStateCartesianPlusJoints : public RobotStateCartesian
{

 public:

  RobotStateCartesianPlusJoints(int n_ee)
      : RobotStateCartesian(n_ee)
  {
    joint_positions_.resize(n_ee);

  }
  ~RobotStateCartesianPlusJoints() = default;


  std::vector<Eigen::VectorXd> joint_positions_;

};

} /* namespace */
