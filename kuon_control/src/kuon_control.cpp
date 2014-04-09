////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Kuon Robotic Mobile Platform ROS Package
//
// Link:      https://github.com/roadnarrows-robotics/kuon
//
// ROS Node:  kuon_control
//
// File:      kuon_control.cpp
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief The ROS kuon_control node class implementation.
 *
 * \author Danial Packard (daniel@roadnarrows.com)
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2013-2014  RoadNarrows
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
 */
/*
 * @EulaBegin@
 * 
 * Permission is hereby granted, without written agreement and without
 * license or royalty fees, to use, copy, modify, and distribute this
 * software and its documentation for any purpose, provided that
 * (1) The above copyright notice and the following two paragraphs
 * appear in all copies of the source code and (2) redistributions
 * including binaries reproduces these notices in the supporting
 * documentation.   Substantial modifications to this software may be
 * copyrighted by their authors and need not follow the licensing terms
 * described here, provided that the new terms are clearly indicated in
 * all files where they apply.
 * 
 * IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
 * OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
 * PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
 * EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
 * "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * 
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

//
// System
//
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <map>

//
// Boost libraries
//
#include <boost/bind.hpp>

//
// ROS
//
#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"

//
// ROS generated core, industrial, and kuon messages.
//
#include "trajectory_msgs/JointTrajectory.h"
#include "sensor_msgs/JointState.h"
#include "industrial_msgs/RobotStatus.h"
#include "kuon_control/BrakeCmd.h"
#include "kuon_control/JointStateExtended.h"
#include "kuon_control/MotorHealth.h"
#include "kuon_control/ProductInfo.h"
#include "kuon_control/RobotStatusExtended.h"
#include "kuon_control/SlewCmd.h"
#include "kuon_control/SpeedCmd.h"
#include "kuon_control/Units.h"

//
// ROS generatated kuon services.
//
#include "kuon_control/EStop.h"
#include "kuon_control/Freeze.h"
#include "kuon_control/GetProductInfo.h"
#include "kuon_control/IncrementGovernor.h"
#include "kuon_control/IsAlarmed.h"
#include "kuon_control/IsDescLoaded.h"
#include "kuon_control/Release.h"
#include "kuon_control/ResetEStop.h"
#include "kuon_control/SetGovernor.h"
#include "kuon_control/SetRobotMode.h"
#include "kuon_control/Stop.h"

//
// ROS generated action servers.
//

//
// RoadNarrows
//
#include "rnr/rnrconfig.h"
#include "rnr/units.h"
#include "rnr/log.h"

//
// RoadNarrows embedded kuon library.
//
#include "Kuon/kuon.h"
#include "Kuon/kuonUtils.h"
#include "Kuon/kuonXmlCfg.h"
#include "Kuon/kuonStatus.h"
#include "Kuon/kuonJoint.h"
#include "Kuon/kuonRobot.h"

//
// Node headers.
//
#include "kuon_control.h"


using namespace std;
using namespace kuon;
using namespace kuon_control;


//------------------------------------------------------------------------------
// KuonControl Class
//------------------------------------------------------------------------------

KuonControl::KuonControl(ros::NodeHandle &nh, double hz) :
    m_nh(nh), m_hz(hz)
{
}

KuonControl::~KuonControl()
{
  disconnect();
}

int KuonControl::configure(const string &strCfgFile)
{
  KuonXmlCfg  xml;  // kuon xml instance
  int         rc;   // return code

  if( (rc = xml.loadFile(strCfgFile)) < 0 )
  {
    ROS_ERROR("Loading XML file '%s' failed.", strCfgFile.c_str());
  }

  else if( (rc = xml.setKuonDescFromDOM(*m_robot.getKuonDesc())) < 0 )
  {
    ROS_ERROR("Setting robot description failed.");
  }

  else if( (rc = m_robot.getKuonDesc()->markAsDescribed()) < 0 )
  {
    ROS_ERROR("Failed to finalize descriptions.");
  }

  else
  {
    ROS_INFO("Kuon description loaded:\n\t %s",
       m_robot.getKuonDesc()->getFullProdBrief().c_str());
    rc = KUON_OK;
  }

  return rc;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Services
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

void KuonControl::advertiseServices()
{
  string  strSvc;

  strSvc = "estop";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &KuonControl::estop,
                                          &(*this));

  strSvc = "freeze";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &KuonControl::freeze,
                                          &(*this));

  strSvc = "get_product_info";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &KuonControl::getProductInfo,
                                          &(*this));

  strSvc = "increment_governor";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &KuonControl::incrementGovernor,
                                          &(*this));

  strSvc = "is_alarmed";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &KuonControl::isAlarmed,
                                          &(*this));

  strSvc = "is_desc_loaded";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &KuonControl::isDescLoaded,
                                          &(*this));

  strSvc = "release";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &KuonControl::release,
                                          &(*this));

  strSvc = "reset_estop";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &KuonControl::resetEStop,
                                          &(*this));

  strSvc = "set_governor";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &KuonControl::setGovernor,
                                          &(*this));

  strSvc = "set_robot_mode";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &KuonControl::setRobotMode,
                                          &(*this));

  strSvc = "stop";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &KuonControl::stop,
                                          &(*this));
}

bool KuonControl::estop(EStop::Request  &req,
                        EStop::Response &rsp)
{
  const char *svc = "estop";

  ROS_DEBUG("%s", svc);

  m_robot.estop();

  ROS_INFO("ESTOPPED! You must issue a \"reset_estop\" to continue.");

  return true;
}

bool KuonControl::freeze(Freeze::Request  &req,
                         Freeze::Response &rsp)
{
  const char *svc = "freeze";

  ROS_DEBUG("%s", svc);

  m_robot.freeze();

  ROS_INFO("Robot position frozen.");

  return true;
}

bool KuonControl::getProductInfo(GetProductInfo::Request  &req,
                                 GetProductInfo::Response &rsp)
{
  const char *svc = "get_product_info";

  int   nMajor, nMinor, nRev;

  ROS_DEBUG("%s", svc);

  if( !m_robot.isDescribed() )
  {
    ROS_ERROR("%s failed: "
              "Robot description not loaded - unable to determine info.",
              svc);
    return false;
  }

  m_robot.getVersion(nMajor, nMinor, nRev);

  rsp.i.maj             = nMajor;
  rsp.i.min             = nMinor;
  rsp.i.rev             = nRev;
  rsp.i.version_string  = m_robot.getVersion();
  rsp.i.product_id      = m_robot.getProdId();
  rsp.i.product_name    = m_robot.getProdName();
  rsp.i.desc            = m_robot.getFullProdBrief();

  return true;
}

bool KuonControl::incrementGovernor(IncrementGovernor::Request  &req,
                                    IncrementGovernor::Response &rsp)
{
  const char *svc = "increment_governor";

  ROS_DEBUG("%s", svc);

  rsp.governor = m_robot.incrementGovernor(req.delta);

  ROS_INFO("Robot governor set at %5.1f%%.", rsp.governor * 100.0);

  return true;
}

bool KuonControl::isAlarmed(IsAlarmed::Request  &req,
                            IsAlarmed::Response &rsp)
{
  const char *svc = "is_alarmed";

  ROS_DEBUG("%s", svc);

  rsp.is_alarmed = m_robot.isAlarmed();

  if( rsp.is_alarmed )
  {
    ROS_WARN("Kuon is alarmed.");
  }

  return true;
}

bool KuonControl::isDescLoaded(IsDescLoaded::Request  &req,
                               IsDescLoaded::Response &rsp)
{
  const char *svc = "is_desc_loaded";

  ROS_DEBUG("%s", svc);

  rsp.is_desc_loaded = m_robot.isDescribed();

  if( !rsp.is_desc_loaded )
  {
    ROS_WARN("Kuon description file not loaded.");
  }

  return true;
}

bool KuonControl::release(Release::Request  &req,
                          Release::Response &rsp)
{
  const char *svc = "release";

  ROS_DEBUG("%s", svc);

  m_robot.release();

  ROS_INFO("Robot released, motors are undriven.");

  return true;
}

bool KuonControl::resetEStop(ResetEStop::Request  &req,
                             ResetEStop::Response &rsp)
{
  const char *svc = "reset_estop";

  ROS_DEBUG("%s", svc);

  m_robot.resetEStop();

  ROS_INFO("EStop reset.");

  return true;
}

bool KuonControl::setGovernor(SetGovernor::Request  &req,
                              SetGovernor::Response &rsp)
{
  const char *svc = "set_governor";

  ROS_DEBUG("%s", svc);

  rsp.governor = m_robot.setGovernor(req.governor);

  ROS_INFO("Robot governor set at %5.1f%%.", rsp.governor * 100.0);

  return true;
}

bool KuonControl::setRobotMode(SetRobotMode::Request  &req,
                               SetRobotMode::Response &rsp)
{
  const char *svc = "set_robot_mode";

  ROS_DEBUG("%s", svc);

  m_robot.setRobotMode((KuonRobotMode)req.mode.val);

  ROS_INFO("Robot mode set to %d.", req.mode.val);

  return true;
}

bool KuonControl::stop(Stop::Request  &req,
                       Stop::Response &rsp)
{
  const char *svc = "stop";

  ROS_DEBUG("%s", svc);

  m_robot.freeze();

  ROS_INFO("Robot stopped (position frozen).");

  return true;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Topic Publishers
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

void KuonControl::advertisePublishers(int nQueueDepth)
{
  string  strPub;

  strPub = "joint_states";
  m_publishers[strPub] =
    m_nh.advertise<sensor_msgs::JointState>(strPub, nQueueDepth);

  strPub = "joint_states_ex";
  m_publishers[strPub] =
    m_nh.advertise<JointStateExtended>(strPub,nQueueDepth);

  strPub = "robot_status";
  m_publishers[strPub] =
    m_nh.advertise<industrial_msgs::RobotStatus>(strPub, nQueueDepth);

  strPub = "robot_status_ex";
  m_publishers[strPub] =
    m_nh.advertise<RobotStatusExtended>(strPub, nQueueDepth);
}

void KuonControl::publish()
{
  publishJointState();
  publishRobotStatus();
}

void KuonControl::publishJointState()
{
  KuonJointStatePoint   state;

  // get robot's extended joint state.
  m_robot.getJointState(state);
  
  // update joint state message
  updateJointStateMsg(state, m_msgJointState);

  // publish joint state messages
  m_publishers["joint_states"].publish(m_msgJointState);

  // update extended joint state message
  updateExtendedJointStateMsg(state, m_msgJointStateEx);

  // publish extened joint state messages
  m_publishers["joint_states_ex"].publish(m_msgJointStateEx);
}

void KuonControl::publishRobotStatus()
{
  KuonRobotStatus status;   // really status 

  // get robot's extended status.
  m_robot.getRobotStatus(status);

  // update robot status message
  updateRobotStatusMsg(status, m_msgRobotStatus);

  // publish robot status message
  m_publishers["robot_status"].publish(m_msgRobotStatus);

  // update extended robot status message
  updateExtendedRobotStatusMsg(status, m_msgRobotStatusEx);

  // publish extened robot status message
  m_publishers["robot_status_ex"].publish(m_msgRobotStatusEx);
}

void KuonControl::updateJointStateMsg(KuonJointStatePoint     &state,
                                      sensor_msgs::JointState &msg)
{
  //
  // Clear previous joint state data.
  //
  msg.name.clear();
  msg.position.clear();
  msg.velocity.clear();
  msg.effort.clear();

  //
  // Set joint state header.
  //
  msg.header.stamp    = ros::Time::now();
  msg.header.frame_id = "0";
  msg.header.seq++;

  //
  // Set joint state state values;
  //
  for(int n=0; n<state.getNumPoints(); ++n)
  {
    // joint state
    msg.name.push_back(state[n].m_strName);
    msg.position.push_back(state[n].m_fPosition);
    msg.velocity.push_back(state[n].m_fVelocity);
    msg.effort.push_back(state[n].m_fEffort);
  }
}

void KuonControl::updateExtendedJointStateMsg(KuonJointStatePoint &state,
                                              JointStateExtended  &msg)
{
  // 
  // Clear previous extended joint state data.
  //
  msg.name.clear();
  msg.position.clear();
  msg.velocity.clear();
  msg.effort.clear();

  msg.odometer.clear();
  msg.encoder.clear();
  msg.velocity_mps.clear();
  msg.speed.clear();
  msg.power_elec.clear();
  msg.power_mech.clear();
  msg.brake.clear();
  msg.slew.clear();

  //
  // Set extended joint state header.
  //
  msg.header.stamp    = ros::Time::now();
  msg.header.frame_id = "0";
  msg.header.seq++;

  //
  // Set extended joint state values;
  //
  for(int n=0; n<state.getNumPoints(); ++n)
  {
    msg.name.push_back(state[n].m_strName);
    msg.position.push_back(state[n].m_fPosition);
    msg.velocity.push_back(state[n].m_fVelocity);
    msg.effort.push_back(state[n].m_fEffort);

    msg.odometer.push_back(state[n].m_fOdometer);
    msg.encoder.push_back(state[n].m_nEncoder);
    msg.velocity_mps.push_back(state[n].m_fVelocityMps);
    msg.speed.push_back(state[n].m_nSpeed);
    msg.power_elec.push_back(state[n].m_fPe);
    msg.power_mech.push_back(state[n].m_fPm);
    msg.brake.push_back(state[n].m_fBrake);
    msg.slew.push_back(state[n].m_fSlew);
  }
}

void KuonControl::updateRobotStatusMsg(KuonRobotStatus              &status,
                                       industrial_msgs::RobotStatus &msg)
{
  //
  // Set robot status header.
  //
  msg.header.stamp    = ros::Time::now();
  msg.header.frame_id = "0";
  msg.header.seq++;

  //
  // Set industrial message compliant robot status values.
  //
  msg.mode.val            = status.m_eRobotMode;
  msg.e_stopped.val       = status.m_eIsEStopped;
  msg.drives_powered.val  = status.m_eAreDrivesPowered;
  msg.motion_possible.val = status.m_eIsMotionPossible;
  msg.in_motion.val       = status.m_eIsInMotion;
  msg.in_error.val        = status.m_eIsInError;
  msg.error_code          = status.m_nErrorCode;
}

void KuonControl::updateExtendedRobotStatusMsg(KuonRobotStatus     &status,
                                               RobotStatusExtended &msg)
{
  MotorHealth health;
  int         i;

  //
  // Set extended robot status header.
  //
  msg.header.stamp    = ros::Time::now();
  msg.header.frame_id = "0";
  msg.header.seq++;

  //
  // Set kuon message extended robot status values.
  //
  msg.mode.val            = status.m_eRobotMode;
  msg.e_stopped.val       = status.m_eIsEStopped;
  msg.drives_powered.val  = status.m_eAreDrivesPowered;
  msg.motion_possible.val = status.m_eIsMotionPossible;
  msg.in_motion.val       = status.m_eIsInMotion;
  msg.in_error.val        = status.m_eIsInError;
  msg.error_code          = status.m_nErrorCode;

  msg.governor  = status.m_fGovernor;
  msg.battery   = status.m_fBattery;

  // clear previous data
  msg.motor_health.clear();

  //
  // Health
  //
  for(i=0; i<status.m_vecMotorHealth.size(); ++i)
  {
    health.name     = status.m_vecMotorHealth[i].m_strName;
    health.temp     = status.m_vecMotorHealth[i].m_fTemperature;
    health.voltage  = status.m_vecMotorHealth[i].m_fVoltage;
    health.alarm    = status.m_vecMotorHealth[i].m_uAlarms;

    msg.motor_health.push_back(health);
  }
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Subscribed Topics
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

void KuonControl::subscribeToTopics(int nQueueDepth)
{
  string  strSub;

  strSub = "brake_cmd";
  m_subscriptions[strSub] = m_nh.subscribe(strSub, nQueueDepth,
                                          &KuonControl::execBrakeCmd,
                                          &(*this));

  strSub = "slew_cmd";
  m_subscriptions[strSub] = m_nh.subscribe(strSub, nQueueDepth,
                                          &KuonControl::execSlewCmd,
                                          &(*this));

  strSub = "speed_cmd";
  m_subscriptions[strSub] = m_nh.subscribe(strSub, nQueueDepth,
                                          &KuonControl::execSpeedCmd,
                                          &(*this));

  strSub = "move_command";
  m_subscriptions[strSub] = m_nh.subscribe(strSub, nQueueDepth,
                                          &KuonControl::execMoveCmd,
                                          &(*this));
}

void KuonControl::execBrakeCmd(const kuon_control::BrakeCmd &msg)
{
  m_robot.setBrake(msg.brake);
}

void KuonControl::execSlewCmd(const kuon_control::SlewCmd &msg)
{
  m_robot.setSlew(msg.slew);
}

void KuonControl::execSpeedCmd(const kuon_control::SpeedCmd &msg)
{
  units_t   units = toUnits(msg.units.e);

  m_robot.setSpeed(msg.left_motors, msg.right_motors, units);
}

void KuonControl::execMoveCmd(const trajectory_msgs::JointTrajectory &jt)
{
  ROS_DEBUG("Executing move_command.");

  KuonWheelTrajectoryPoint pt;

  // load trajectory point
  for(int j=0; j<jt.joint_names.size(); ++j)
  {
    pt.append(jt.joint_names[j],
              jt.points[0].positions[j], 
              jt.points[0].velocities[j]);
    ROS_INFO("%s: pos=%5.3f speed=%2.1f", jt.joint_names[j].c_str(), 
                                          jt.points[0].positions[j], 
                                          jt.points[0].velocities[j]);
  }

  m_robot.move(pt);
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Utilities
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

units_t KuonControl::toUnits(uint_t u)
{
  switch(u)
  {
    case Units::UNITS_PERCENT:
      return units_percent;
    case Units::UNITS_NORM:
      return units_norm;
    default:
      return units_raw;
  }
}
