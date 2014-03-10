////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Kuon Robot Package
//
// Link:      https://github.com/roadnarrows-robotics/kuon
//
// ROS Node:  kuon_teleop
//
// File:      kuon_teleop.cpp
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief The ROS kuon_teleop node class implementation.
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

#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

#include <string>
#include <map>

//
// ROS
//
#include "ros/ros.h"

//
// RoadNarrows
//
#include "rnr/rnrconfig.h"
#include "rnr/hid/HIDXbox360.h"

//
// ROS generated Kuon messages.
//
#include "kuon_control/BrakeCmd.h"        // publish
#include "kuon_control/KuonStatus.h"      // subscribe (TBD)
#include "kuon_control/KuonState.h"       // subscribe
#include "kuon_control/SlewCmd.h"         // publish
#include "kuon_control/SpeedCmd.h"        // publish
#include "kuon_control/Version.h"         // service

//
// ROS generatated Kuon services.
//
#include "kuon_control/EStop.h"
#include "kuon_control/IncrementGovernor.h"
#include "kuon_control/QueryVersion.h"
#include "kuon_control/ResetEStop.h"
//#include "kuon_control/SetRobotMode.h"

//
// ROS generated HID messages.
//
#include "hid/ConnStatus.h"           // subscribe
#include "hid/Controller360State.h"   // subscribe
#include "hid/LEDPattern.h"           // service
#include "hid/RumbleCmd.h"            // publish

//
// ROS generatated HID services.
//
#include "hid/SetLED.h"
#include "hid/SetRumble.h"

//
// Node headers.
//
#include "kuon_teleop.h"

using namespace std;
using namespace hid;
using namespace kuon_control;
using namespace kuon;


//------------------------------------------------------------------------------
// KuonTeleop Class
//------------------------------------------------------------------------------

KuonTeleop::KuonTeleop(ros::NodeHandle &nh) : m_nh(nh)
{
  m_eState          = TeleopStateUninit;
  m_bHasXboxComm    = false;
  m_nWdXboxCounter  = 0;
  m_bHasKuonComm    = false;
  m_nWdKuonCounter  = 0;
  m_bHasFullComm    = false;
}

KuonTeleop::~KuonTeleop()
{
}


//..............................................................................
// Services
//..............................................................................

void KuonTeleop::advertiseServices()
{
  // No services
}


//..............................................................................
// Topic Publishers
//..............................................................................

void KuonTeleop::advertisePublishers(int nQueueDepth)
{
  string  strPub;

  strPub = "/kuon_control/brake_cmd";
  m_publishers[strPub] =
    m_nh.advertise<kuon_control::BrakeCmd>(strPub,nQueueDepth);

  strPub = "/kuon_control/slew_cmd";
  m_publishers[strPub] =
    m_nh.advertise<kuon_control::SlewCmd>(strPub, nQueueDepth);

  strPub = "/kuon_control/speed_cmd";
  m_publishers[strPub] =
    m_nh.advertise<kuon_control::SpeedCmd>(strPub, nQueueDepth);

  strPub = "/xbox_360/rumble_command";
  m_publishers[strPub] =
    m_nh.advertise<hid::RumbleCmd>(strPub, nQueueDepth);
}

void KuonTeleop::publishBrakeCmd(int brake)
{
  BrakeCmd  msg;
  
  msg.val = brake;

  // publish
  m_publishers["/kuon_control/brake_cmd"].publish(msg);
}

void KuonTeleop::publishSlewCmd(int slew)
{
  SlewCmd msg;
  
  msg.val = slew;

  // publish
  m_publishers["/kuon_control/slew_cmd"].publish(msg);
}

void KuonTeleop::publishSpeedCmd(int speedLeft, int speedRight)
{
  SpeedCmd msg;
  
  msg.left  = speedLeft;
  msg.right = speedRight;

  // publish
  m_publishers["/kuon_control/speed_cmd"].publish(msg);
}

void KuonTeleop::publishRumbleCmd(int motorLeft, int motorRight)
{
  RumbleCmd msg;
  
  // publish
  m_publishers["/xbox_360/rumble_command"].publish(msg);
}


//..............................................................................
// Subscribed Topics
//..............................................................................

void KuonTeleop::subscribeToTopics(int nQueueDepth)
{
  string  strSub;

  strSub = "/kuon_control/kuon_status";
  m_subscriptions[strSub] = m_nh.subscribe(strSub, nQueueDepth,
                                          &KuonTeleop::cbKuonStatus,
                                          &(*this));

  strSub = "/xbox_360/conn_status";
  m_subscriptions[strSub] = m_nh.subscribe(strSub, nQueueDepth,
                                          &KuonTeleop::cbXboxConnStatus,
                                          &(*this));

  strSub = "/xbox_360/controller_360_state";
  m_subscriptions[strSub] = m_nh.subscribe(strSub, nQueueDepth,
                                          &KuonTeleop::cbXboxBttnState,
                                          &(*this));
}

void KuonTeleop::cbKuonStatus(const kuon_control::KuonStatus &msg)
{
  ROS_DEBUG("Received Kuon status.");

  m_bHasKuonComm    = true;
  m_nWdKuonCounter  = 0;
}

void KuonTeleop::cbXboxConnStatus(const hid::ConnStatus &msg)
{
  ROS_DEBUG("Received Xbox360 connectivity status.");

  m_bHasXboxComm    = msg.is_connected && msg.is_linked;
  m_nWdXboxCounter  = 0;
}

void KuonTeleop::cbXboxBttnState(const hid::Controller360State &msg)
{
  ROS_DEBUG("Received Xbox360 button state.");

  if( m_bHasFullComm )
  {
    switch( m_eState )
    {
      case TeleopStateReady:
        execAllButtonActions(msg);
        break;
      case TeleopStatePause:
        if( buttonOffToOn(rnr::Xbox360FeatIdBack) )
        {
          buttonStart();
        }
        break;
      case TeleopStateUninit:
      default:
        m_eState = TeleopStatePause;
        //setLED(z);
        break;
    }
  }

  m_msgXboxState = msg;
}


//..............................................................................
// Sanity
//..............................................................................

void KuonTeleop::commCheck()
{
  if( m_bHasXboxComm )
  {
    if( ++m_nWdXboxCounter >= 30 )
    {
      m_bHasXboxComm = false;
    }
  }

  if( m_bHasKuonComm )
  {
    if( ++m_nWdKuonCounter >= 10 )
    {
      m_bHasKuonComm = false;
    }
  }

  bool hasComm  = m_bHasXboxComm && m_bHasKuonComm;

  if( m_bHasFullComm && !hasComm )
  {
    putRobotInSafeMode();
  }

  m_bHasFullComm = hasComm;
}

void KuonTeleop::putRobotInSafeMode()
{
  // stop robot
  publishSpeedCmd(0, 0);

  // set hard brake
  publishBrakeCmd(0);

  // set robot mode
  // TBD
 
  m_eState = TeleopStateUninit;
}


//..............................................................................
// Button Actions
//..............................................................................

void KuonTeleop::execAllButtonActions(const hid::Controller360State &msg)
{
}

bool KuonTeleop::buttonOffToOn(int id)
{
}

void KuonTeleop::buttonStart()
{
}
