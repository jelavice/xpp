/*
 * M545StateConverter.cc
 *
 *  Created on: Aug 20, 2018
 *      Author: jelavice
 */

#include <xpp_m545/M545StateConverter.h>
#include <m545_msgs/M545State.h>
//#include "excavator_model/ExcavatorModel.hpp"
#include "excavator_model/ConversionTraits.hpp"
#include "loco_m545/common/typedefs.hpp"

namespace xpp {

M545StateConverter::M545StateConverter(const std::string& xpp_state_topic,
                                       const std::string& m545_state_topic)
{

  ::ros::NodeHandle nh;
  xpp_state_sub_ = nh.subscribe(xpp_state_topic, 1, &M545StateConverter::StateCallback, this);
  ROS_DEBUG("Subscribed to: %s", xpp_state_sub_.getTopic().c_str());

  m545_state_pub_ = nh.advertise < m545_msgs::M545State > (m545_state_topic, 1);
}

void M545StateConverter::StateCallback(const xpp_msgs::RobotStateCartesianPlusJoints& msg)
{

  excavator_model::ExcavatorState robotState;

  robotState.setTime(any_measurements::Time(0, msg.time_from_start.toNSec()).toChrono());
  robotState.setStatus(excavator_model::ExcavatorState::Status::STATUS_OK);

  robotState.setPositionWorldToBaseInWorldFrame(
      excavator_model::Position(msg.base.pose.position.x, msg.base.pose.position.y,
                                msg.base.pose.position.z));

  robotState.setOrientationBaseToWorld(
      excavator_model::RotationQuaternion(msg.base.pose.orientation.w, msg.base.pose.orientation.x,
                                          msg.base.pose.orientation.y,
                                          msg.base.pose.orientation.z));

  robotState.setLinearVelocityBaseInWorldFrame(excavator_model::LinearVelocity(0.0, 0.0, 0.0));
  robotState.setAngularVelocityBaseInBaseFrame(
      excavator_model::LocalAngularVelocity(0.0, 0.0, 0.0));

  excavator_model::JointPositions joint_positions;


  int n_ee = msg.limb_joints.size();
  for (int ee_id = 0; ee_id < n_ee; ++ee_id) {

    loco_m545::RD::LimbEnum limb = static_cast<loco_m545::RD::LimbEnum>(ee_id);
    unsigned int id_start_kindr = loco_m545::RD::mapKeyEnumToKeyId(
        loco_m545::RD::getLimbStartJoint(limb));

    FillKindrVector(msg, joint_positions, id_start_kindr, ee_id );
  }

  robotState.setJointPositions(joint_positions);


  excavator_model::JointTorques joint_torques;
  excavator_model::JointVelocities joint_velocities;
  joint_torques.setZero();
  joint_velocities.setZero();

  robotState.setJointVelocities(joint_velocities);
  robotState.setJointTorques(joint_torques);
  robotState.setMapToOdomTransform(excavator_model::Pose());

  using conversion_traits = typename excavator_model::ConversionTraits<excavator_model::ExcavatorState, m545_msgs::M545State>;

  m545_msgs::M545State msgRos = conversion_traits::convert(robotState);

  m545_state_pub_.publish(msgRos);

}

void M545StateConverter::FillKindrVector(const xpp_msgs::RobotStateCartesianPlusJoints& msg,
                     excavator_model::JointPositions &kindrVec, int starting_index_kindr,
                     int ee_id)
{

  int numDof = msg.limb_joints.at(ee_id).position.size();
  for (int i = 0; i < numDof; ++i) {
    kindrVec(i + starting_index_kindr) = msg.limb_joints.at(ee_id).position.at(i);
  }

}

} /* namespace */
