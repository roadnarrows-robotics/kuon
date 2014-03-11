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
// Boost
//
#include "boost/assign.hpp"

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
// TBD #include "kuon_control/SetRobotMode.h"

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
using namespace boost::assign;
using namespace hid;
using namespace kuon_control;
using namespace kuon;


//------------------------------------------------------------------------------
// KuonTeleop Class
//------------------------------------------------------------------------------

KuonTeleop::KuonTeleop(ros::NodeHandle &nh, double hz) : m_nh(nh), m_hz(hz)
{
  m_eState          = TeleopStateUninit;
  m_bHasXboxComm    = false;
  m_nWdXboxCounter  = 0;
  m_nWdXboxTimeout  = countsPerSecond(1.0);
  m_bHasKuonComm    = false;
  m_nWdKuonCounter  = 0;
  m_nWdKuonTimeout  = countsPerSecond(1.0);
  m_bHasFullComm    = false;

  m_buttonState = map_list_of
      (ButtonIdEStop,   0)
      (ButtonIdGovUp,   0)
      (ButtonIdGovDown, 0)
      (ButtonIdPause,   0)
      (ButtonIdStart,   0)
      (ButtonIdMoveX,   0)
      (ButtonIdMoveY,   0)
      (ButtonIdBrake,   0)
      (ButtonIdSlew,    0);
}

KuonTeleop::~KuonTeleop()
{
}


//..............................................................................
// Server Services
//..............................................................................

void KuonTeleop::advertiseServices()
{
  // No services
}


//..............................................................................
// Client Services
//..............................................................................

void KuonTeleop::clientServices()
{
  string  strSvc;

  strSvc = "/xbox_360/set_led";
  m_clientServices[strSvc] = m_nh.serviceClient<hid::SetLED>(strSvc);

  strSvc = "/xbox_360/set_rumble";
  m_clientServices[strSvc] = m_nh.serviceClient<hid::SetRumble>(strSvc);

  // TBD strSvc = "/kuon_control/set_robot_mode";
  // TBD m_clientServices[strSvc] =
  //    m_nh.serviceClient<kuon_control::SetRobotMode>(strSvc);

  strSvc = "/kuon_control/estop";
  m_clientServices[strSvc] = m_nh.serviceClient<kuon_control::EStop>(strSvc);

  strSvc = "/kuon_control/reset_estop";
  m_clientServices[strSvc] =
      m_nh.serviceClient<kuon_control::ResetEStop>(strSvc);

  strSvc = "/kuon_control/increment_governor";
  m_clientServices[strSvc] =
      m_nh.serviceClient<kuon_control::IncrementGovernor>(strSvc);
}

void KuonTeleop::setLED(int pattern)
{
  hid::SetLED svc;

  svc.request.led_pattern.val = pattern;

  if( m_clientServices["/xbox_360/set_led"].call(svc) )
  {
    ROS_INFO("Xbox360 LED set to pattern to %d", pattern);
  }
  else
  {
    ROS_ERROR("Failed to set Xbox360 LED.");
  }
}

void KuonTeleop::setRumble(int motorLeft, int motorRight)
{
  hid::SetRumble svc;

  svc.request.left_rumble  = motorLeft;
  svc.request.right_rumble = motorRight;

  if( m_clientServices["/xbox_360/set_rumble"].call(svc) )
  {
    ROS_INFO("Xbox360 rumble motors set to %d, %d", motorLeft, motorRight);
  }
  else
  {
    ROS_ERROR("Failed to set Xbox360 rumble motors.");
  }
}

void KuonTeleop::estop()
{
  kuon_control::EStop svc;

  if( m_clientServices["/kuon_control/estop"].call(svc) )
  {
    ROS_INFO("Kuon emergency stopped.");
  }
  else
  {
    ROS_ERROR("Failed to estop Kuon.");
  }
}

void KuonTeleop::resetEStop()
{
  kuon_control::ResetEStop svc;

  if( m_clientServices["/kuon_control/reset_estop"].call(svc) )
  {
    ROS_INFO("Kuon emergency stopped has been reset.");
  }
  else
  {
    ROS_ERROR("Failed to reset estop.");
  }
}

void KuonTeleop::setGovernor(float delta)
{
  kuon_control::IncrementGovernor svc;

  svc.request.delta = delta;

  if( m_clientServices["/kuon_control/increment_governor"].call(svc) )
  {
    ROS_INFO("Kuon governor set at %%5.1f", svc.response.value);
  }
  else
  {
    ROS_ERROR("Failed to set governor.");
  }
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
  
  msg.left_rumble  = motorLeft;
  msg.right_rumble = motorRight;

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
  ButtonState buttonState;

  ROS_DEBUG("Received Xbox360 button state.");

  if( m_bHasFullComm )
  {
    msgToState(msg, buttonState);

    switch( m_eState )
    {
      case TeleopStateReady:
        execAllButtonActions(buttonState);
        break;
      case TeleopStatePaused:
        buttonStart(buttonState);    // only button active in pause state
        break;
      case TeleopStateUninit:
      default:
        m_eState = TeleopStatePaused;
        setLED(LEDPatPaused);
        break;
    }
  }

  m_buttonState = buttonState;
}


//..............................................................................
// Sanity
//..............................................................................

void KuonTeleop::commCheck()
{
  if( m_bHasXboxComm )
  {
    if( ++m_nWdXboxCounter >= m_nWdXboxTimeout )
    {
      m_bHasXboxComm = false;
    }
  }

  if( m_bHasKuonComm )
  {
    if( ++m_nWdKuonCounter >= m_nWdKuonTimeout )
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

  setLED(LEDPatUninit);
}


//..............................................................................
// Xbox Actions
//..............................................................................

void KuonTeleop::msgToState(const hid::Controller360State &msg,
                            ButtonState                   &buttonState)
{
  buttonState[ButtonIdEStop]    = msg.b_button;
  buttonState[ButtonIdGovUp]    = msg.dpad_up;
  buttonState[ButtonIdGovDown]  = msg.dpad_down;
  buttonState[ButtonIdPause]    = msg.back_button;
  buttonState[ButtonIdStart]    = msg.start_button;
  buttonState[ButtonIdMoveX]    = msg.left_joy_x;
  buttonState[ButtonIdMoveY]    = msg.left_joy_y;
  buttonState[ButtonIdBrake]    = msg.left_trig;
  buttonState[ButtonIdSlew]     = msg.right_trig;
}

bool KuonTeleop::buttonOffToOn(int id, ButtonState &buttonState)
{
  return (m_buttonState[id] == 0) && (buttonState[id] == 1);
}

void KuonTeleop::execAllButtonActions(ButtonState &buttonState)
{
}

void KuonTeleop::buttonStart(ButtonState &buttonState)
{
  if( buttonOffToOn(ButtonIdPause, buttonState) )
  {
    ROS_INFO("Manual operation active, auto mode disabled.");

    m_eState = TeleopStateReady;

    if( m_msgRobotStatus.e_stopped )
    {
      resetEStop();
    }

    setLED(LEDPatReady);
  }
}

void KuonTeleop::buttonPause(ButtonState &buttonState)
{
  if( buttonOffToOn(ButtonIdPause, buttonState) )
  {
    ROS_INFO("Manual operation paused, auto mode enabled.");

    m_eState = TeleopStatePaused;

    setLED(LEDPatPaused);
  }
}

void KuonTeleop::buttonEStop(ButtonState &buttonState)
{
  static int  clicks        = 0;            // number of button clicks
  static int  intvlCounter  = 0;            // intra-click interval counter
  static int  intvlTimeout  = countsPerSecond(0.3); // intra-click timeout

  //
  // Robot is estopped. This can come from a different node source. Make sure
  // counters are cleared.
  //
  if( m_msgRobotStatus.e_stopped )
  {
    clicks = 0;
    intvlCounter = 0;
    return;
  }

  // button off to on
  if( buttonOffToOn(ButtonIdEStop, buttonState) )
  {
    ++clicks;
  }

  switch( clicks )
  {
    case 0:     // no click
      break;
    case 1:     // single click
      if( intvlCounter > intvlTimeout )
      {
        clicks = 0;
        intvlCounter = 0;
      }
      break;
    case 2:     // double click
      if( intvlCounter <= intvlTimeout )
      {
        estop();
      }
      clicks = 0;
      intvlCounter = 0;
      break;
    default:    // multiple clicks
      clicks = 0;
      intvlCounter = 0;
      break;
  }
}

void KuonTeleop::buttonGovernorUp(ButtonState &buttonState)
{
  float governor = m_msgRobotStatus.governor_value;
  float inc;

  if( buttonOffToOn(ButtonIdGovUp, buttonState) )
  {
    inc = 0.1;

    governor += inc;

    if( governor > 1.0 )
    {
      inc = 1.0 - governor;
    }

    if( inc <= 0.0 )
    {
      return;
    }

    setGovernor(inc);
  }
}

void KuonTeleop::buttonGovernorDown(ButtonState &buttonState)
{
  float governor = m_msgRobotStatus.governor_value;
  float dec;

  if( buttonOffToOn(ButtonIdGovUp, buttonState) )
  {
    dec = -0.1;

    if( (governor + dec) < 0.0 )
    {
      dec = -governor;
    }

    if( dec == 0.0 )
    {
      return;
    }

    setGovernor(dec);
  }
}

void KuonTeleop::buttonBrake(ButtonState &buttonState)
{
  int   brake;

  if( canMove() )
  {
    brake = 31 - (int)(31.0 * (float)buttonState[ButtonIdBrake]/255.0);

    publishBrakeCmd(brake);
  }
}

void KuonTeleop::buttonSlew(ButtonState &buttonState)
{
  int   slew;

  if( canMove() )
  {
    slew = (int)(80.0 - (float)buttonState[ButtonIdSlew]/255.0);

    publishSlewCmd(slew);
  }
}

void KuonTeleop::buttonMove(ButtonState &buttonState)
{
  int   joy_x;
  int   joy_y;
  int   speedLeft;
  int   speedRight;

  if( canMove() )
  {
    joy_x = buttonState[ButtonIdMoveX];
    joy_y = buttonState[ButtonIdMoveY];

    // mix throttle x and y values
    speedLeft  = joy_x + joy_y;
    speedRight = joy_y - joy_x;
  
    // scale 16-bit throttle values to 9-bit speed values
    speedLeft  >>= 7;
    speedRight >>= 7;

    // cap speeds
    if( speedLeft > 249 )
    {
      speedLeft = 249;
    }
    else if( speedLeft < -249 )
    {
      speedLeft = -249;
    }

    if( speedRight > 249 )
    {
      speedRight = 249;
    }
    else if( speedRight < -249 )
    {
      speedRight = -249;
    }

    publishSpeedCmd(speedLeft, speedRight);
  }
}
