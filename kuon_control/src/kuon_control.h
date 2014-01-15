////////////////////////////////////////////////////////////////////////////////
//
// Package: RoadNarrows Robotics ROS Kuon Package
//
// Link: https://github.com/roadnarrows-robotics/kuon
//
// ROS Node: kuon_control
//
// File: kuon_control.h
//
/*! \file
*
* \brief The ROS kuon_control node supported services.
*
* \author Daniel Packard (daniel@roadnarrows.com)
*
* \par Copyright:
* (C) 2013 RoadNarrows
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
* documentation. Substantial modifications to this software may be
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
#ifndef _KUON_CONTROL_H
#define _KUON_CONTROL_H
#include <string>

#include "KuonRobot.h"

#include "ros/ros.h"

#include "kuon_control/BrakeCmd.h"
#include "kuon_control/EStop.h"
#include "kuon_control/IncrementGovernor.h"
#include "kuon_control/KuonState.h"
#include "kuon_control/KuonStatus.h"
#include "kuon_control/QueryVersion.h"
#include "kuon_control/ResetEStop.h"
#include "kuon_control/SlewCmd.h"
#include "kuon_control/SpeedCmd.h"
#include "kuon_control/Version.h"

namespace kuon_control 
{

class KuonControlNode 
{
public:

  KuonControlNode()
  {
    m_pRobot = new KuonRobot();
  }

  ~KuonControlNode() {disconnect();}

  int connect();
  int disconnect();

  // --- Service callbacks
  bool EStop(kuon_control::EStop::Request &req,
             kuon_control::EStop::Response &rsp);

  bool ResetEStop(kuon_control::ResetEStop::Request &req,
                  kuon_control::ResetEStop::Response &rsp);

  bool QueryVersion(kuon_control::QueryVersion::Request &req,
                    kuon_control::QueryVersion::Response &rsp);

  bool IncrementGovernor(kuon_control::IncrementGovernor::Request &req,
                         kuon_control::IncrementGovernor::Response &rsp);

  // --- Subscriptions
  void brake_cmdCB(const kuon_control::BrakeCmd &cmd)
  {
    ROS_INFO("received brake command: %d",cmd.val );
    m_pRobot->setBrake(cmd.val);
  }

  void slew_cmdCB(const kuon_control::SlewCmd &cmd)
  {
    ROS_INFO("received slew command: %d",cmd.val);
    m_pRobot->setSlew(cmd.val);
  }

  void speed_cmdCB(const kuon_control::SpeedCmd &cmd)
  {
    ROS_INFO("received speed command: %d %d",cmd.left, cmd.right);
    m_pRobot->setSpeeds(cmd.left, cmd.right);
  }
  
  // --- Publications
  int UpdateStatus(kuon_control::KuonStatus &status)
  {
    status.is_estopped    = m_pRobot->isEStopped();
    status.governor_value = m_pRobot->QueryGovernorVal();
  }

  int UpdateState(kuon_control::KuonState &state);

protected:
  KuonRobot *m_pRobot;      ///< Kuon robot handle
  bool       m_bIsEStopped; ///< Kuon is [not] estopped
  float      m_fGovernor;   ///< Normalized governor setting [min:0.0, max:1.0]
};

}

#endif // _KUON_CONTROL_H
