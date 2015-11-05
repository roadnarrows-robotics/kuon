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
#include "kuon_control/BrakeCmd.h"            // publish
#include "kuon_control/JointStateExtended.h"  // subscribe
#include "kuon_control/MotorHealth.h"         // message
#include "kuon_control/ProductInfo.h"         // service
#include "kuon_control/RobotStatusExtended.h" // subscribe
#include "kuon_control/SlewCmd.h"             // publish
#include "kuon_control/SpeedCmd.h"            // publish
#include "kuon_control/Units.h"               // message

//
// ROS generatated Kuon services.
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
// RoadNarrows embedded kuon library.
//
#include "Kuon/kuon.h"
#include "Kuon/kuonUtils.h"

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
  m_nWdXboxTimeout  = countsPerSecond(3.0);
  m_bHasRobotComm   = false;
  m_nWdRobotCounter = 0;
  m_nWdRobotTimeout = countsPerSecond(5.0);
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

  strSvc = "/kuon_control/estop";
  m_clientServices[strSvc] = m_nh.serviceClient<kuon_control::EStop>(strSvc);

  strSvc = "/kuon_control/freeze";
  m_clientServices[strSvc] = m_nh.serviceClient<kuon_control::Freeze>(strSvc);

  strSvc = "/kuon_control/increment_governor";
  m_clientServices[strSvc] =
      m_nh.serviceClient<kuon_control::IncrementGovernor>(strSvc);

  strSvc = "/kuon_control/release";
  m_clientServices[strSvc] = m_nh.serviceClient<kuon_control::Release>(strSvc);

  strSvc = "/kuon_control/reset_estop";
  m_clientServices[strSvc] =
      m_nh.serviceClient<kuon_control::ResetEStop>(strSvc);

  strSvc = "/kuon_control/set_governor";
  m_clientServices[strSvc] =
    m_nh.serviceClient<kuon_control::SetRobotMode>(strSvc);

  strSvc = "/kuon_control/set_robot_mode";
  m_clientServices[strSvc] =
    m_nh.serviceClient<kuon_control::SetRobotMode>(strSvc);
}

void KuonTeleop::setLED(int pattern)
{
  hid::SetLED svc;

  svc.request.led_pattern.val = pattern;

  if( m_clientServices["/xbox_360/set_led"].call(svc) )
  {
    ROS_DEBUG("Xbox360 LED set to pattern to %d", pattern);
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
    ROS_DEBUG("Xbox360 rumble motors set to %d, %d", motorLeft, motorRight);
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

void KuonTeleop::freeze()
{
  kuon_control::Freeze svc;

  if( m_clientServices["/kuon_control/freeze"].call(svc) )
  {
    ROS_INFO("Kuon frozen.");
  }
  else
  {
    ROS_ERROR("Failed to freeze Kuon.");
  }
}

void KuonTeleop::incrementGovernor(float delta)
{
  kuon_control::IncrementGovernor svc;

  svc.request.delta = delta;

  if( m_clientServices["/kuon_control/increment_governor"].call(svc) )
  {
    ROS_DEBUG("Kuon governor set at %5.1f", svc.response.governor);
  }
  else
  {
    ROS_ERROR("Failed to increment governor.");
  }
}

void KuonTeleop::release()
{
  kuon_control::Release svc;

  if( m_clientServices["/kuon_control/release"].call(svc) )
  {
    ROS_INFO("Kuon released.");
  }
  else
  {
    ROS_ERROR("Failed to release Kuon.");
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

void KuonTeleop::setGovernor(float governor)
{
  kuon_control::SetGovernor svc;

  svc.request.governor = governor;

  if( m_clientServices["/kuon_control/set_governor"].call(svc) )
  {
    ROS_DEBUG("Kuon governor set at %5.1f", svc.response.governor);
  }
  else
  {
    ROS_ERROR("Failed to set governor.");
  }
}

void KuonTeleop::setRobotMode(int mode)
{
  kuon_control::SetRobotMode svc;

  svc.request.mode.val = mode;

  if( m_clientServices["/kuon_control/set_robot_mode"].call(svc) )
  {
    ROS_DEBUG("Kuon mode set to %d.", svc.request.mode.val);
  }
  else
  {
    ROS_ERROR("Failed to set robot mode.");
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

void KuonTeleop::publishBrakeCmd(float brake)
{
  BrakeCmd  msg;
  
  msg.brake = brake;

  // publish
  m_publishers["/kuon_control/brake_cmd"].publish(msg);

  ROS_DEBUG("Brake at %4.2f of maximum.", msg.brake);
}

void KuonTeleop::publishSlewCmd(float slew)
{
  SlewCmd msg;
  
  msg.slew = slew;

  // publish
  m_publishers["/kuon_control/slew_cmd"].publish(msg);

  ROS_DEBUG("Slew at %4.2f of minimum.", msg.slew);
}

void KuonTeleop::publishSpeedCmd(double speedLeft, double speedRight)
{
  SpeedCmd msg;

  msg.units.e      = Units::UNITS_NORM;
  msg.left_motors  = speedLeft;
  msg.right_motors = speedRight;

  // publish
  m_publishers["/kuon_control/speed_cmd"].publish(msg);

  ROS_DEBUG("Speed = %6.1lf%%, %6.1lf%%.",
      msg.left_motors*100.0, msg.right_motors*100.0);
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

  strSub = "/kuon_control/robot_status_ex";
  m_subscriptions[strSub] = m_nh.subscribe(strSub, nQueueDepth,
                                          &KuonTeleop::cbRobotStatus,
                                          &(*this));

  // not needed
  //strSub = "/kuon_control/joint_state_ex";
  //m_subscriptions[strSub] = m_nh.subscribe(strSub, nQueueDepth,
  //                                        &KuonTeleop::cbJointState,
  //                                        &(*this));

  strSub = "/xbox_360/conn_status";
  m_subscriptions[strSub] = m_nh.subscribe(strSub, nQueueDepth,
                                          &KuonTeleop::cbXboxConnStatus,
                                          &(*this));

  strSub = "/xbox_360/controller_360_state";
  m_subscriptions[strSub] = m_nh.subscribe(strSub, nQueueDepth,
                                          &KuonTeleop::cbXboxBttnState,
                                          &(*this));
}

void KuonTeleop::cbRobotStatus(const kuon_control::RobotStatusExtended &msg)
{
  ROS_DEBUG("Received robot status.");

  m_bHasRobotComm   = true;
  m_nWdRobotCounter = 0;

  m_msgRobotStatus = msg;
}

void KuonTeleop::cbJointState(const kuon_control::JointStateExtended &msg)
{
  ROS_DEBUG("Received joint state.");
}

void KuonTeleop::cbXboxConnStatus(const hid::ConnStatus &msg)
{
  ROS_DEBUG("Received Xbox360 connectivity status.");

  m_bHasXboxComm    = msg.is_connected && msg.is_linked;
  m_nWdXboxCounter  = 0;

  m_msgConnStatus = msg;
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
        buttonStart(buttonState); // only button active in pause state
        break;
      case TeleopStateUninit:
      default:
        pause();
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

  if( m_bHasRobotComm )
  {
    if( ++m_nWdRobotCounter >= m_nWdRobotTimeout )
    {
      m_bHasRobotComm = false;
    }
  }

  bool hasComm  = m_bHasXboxComm && m_bHasRobotComm;

  // had communitcation, but no more
  if( m_bHasFullComm && !hasComm )
  {
    ROS_INFO("Lost communication with Xbox360 and/or Kuon.");
    putRobotInSafeMode(m_msgConnStatus.is_connected);
  }

  m_bHasFullComm = hasComm;

  // not really a communication check function, but convenient.
  if( m_eState == TeleopStatePaused )
  {
    driveLEDsFigure8Pattern();
  }
}

void KuonTeleop::putRobotInSafeMode(bool bHard)
{
  static float  fGovDft = 0.20;

  // stop robot with 'parking' brake at full
  freeze();

  // set robot mode
  setRobotMode(KuonRobotModeAuto);
 
  if( bHard )
  {
    setGovernor(fGovDft);
  }
  
  m_eState = TeleopStateUninit;

  setLED(LEDPatOn);
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

void KuonTeleop::execAllButtonActions(ButtonState &buttonState)
{
  // emergency stop
  buttonEStop(buttonState);

  //
  // Teleoperation state.
  // /
  if( m_eState == TeleopStateReady )
  {
    buttonPause(buttonState);
  }
  else if( m_eState == TeleopStatePaused )
  {
    buttonStart(buttonState);
  }

  //
  // Moves.
  //
  if( canMove() )
  {
    buttonSpeed(buttonState);
    buttonBrake(buttonState);
    buttonSlew(buttonState);
  }

  //
  // Other.
  //
  buttonGovernorUp(buttonState);
  buttonGovernorDown(buttonState);
}

void KuonTeleop::buttonStart(ButtonState &buttonState)
{
  if( buttonOffToOn(ButtonIdStart, buttonState) )
  {
    ROS_INFO("Manual operation active, auto mode disabled.");

    if( m_msgRobotStatus.e_stopped.val == industrial_msgs::TriState::TRUE )
    {
      resetEStop();
    }

    setRobotMode(KuonRobotModeManual);

    ready();
  }
}

void KuonTeleop::buttonPause(ButtonState &buttonState)
{
  if( buttonOffToOn(ButtonIdPause, buttonState) )
  {
    ROS_INFO("Manual operation paused, auto mode enabled.");

    setRobotMode(KuonRobotModeAuto);

    pause();
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
  if( m_msgRobotStatus.e_stopped.val == industrial_msgs::TriState::TRUE )
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
        pause();
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
  if( buttonOffToOn(ButtonIdGovUp, buttonState) )
  {
    incrementGovernor(0.1);
  }
}

void KuonTeleop::buttonGovernorDown(ButtonState &buttonState)
{
  if( buttonOffToOn(ButtonIdGovDown, buttonState) )
  {
    incrementGovernor(-0.1);
  }
}

void KuonTeleop::buttonBrake(ButtonState &buttonState)
{
  float   brake;

  if( buttonDiff(ButtonIdBrake, buttonState) )
  {
    brake = (float)buttonState[ButtonIdBrake]/(float)XBOX360_TRIGGER_MAX;

    publishBrakeCmd(brake);
  }
}

void KuonTeleop::buttonSlew(ButtonState &buttonState)
{
  float   slew;

  if( buttonDiff(ButtonIdSlew, buttonState) )
  {
    slew = (float)buttonState[ButtonIdSlew]/(float)XBOX360_TRIGGER_MAX;

    publishSlewCmd(slew);
  }
}

void KuonTeleop::buttonSpeed(ButtonState &buttonState)
{
  double  joy_x;
  double  joy_y;
  double  speedLeft;
  double  speedRight;

  joy_x = (double)buttonState[ButtonIdMoveX];
  joy_y = (double)buttonState[ButtonIdMoveY];

  //
  // Note: kuon_control has watchdog on this subscribed speed message. It
  // will timout and stop the robot if not sent frequently. So always send if
  // different or not zero.
  //
  if( !buttonDiff(ButtonIdMoveX, buttonState) &&
      !buttonDiff(ButtonIdMoveY, buttonState) &&
      (joy_x == 0) && (joy_y == 0) )
  {
    return;
  }

  // mix throttle x and y values
  speedLeft   = (joy_x + joy_y) / (double)XBOX360_JOY_MAX;
  speedRight  = (joy_y - joy_x) / (double)XBOX360_JOY_MAX;
  
  // cap
  speedLeft   = fcap(speedLeft, -1.0, 1.0);
  speedRight  = fcap(speedRight, -1.0, 1.0);

  publishSpeedCmd(speedLeft, speedRight);
}


//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
// Support
//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

void KuonTeleop::pause()
{
  m_eState = TeleopStatePaused;

  setLED(LEDPatPaused);
}

void KuonTeleop::ready()
{
  m_eState = TeleopStateReady;

  setLED(LEDPatReady);
}

void KuonTeleop::driveLEDsFigure8Pattern()
{
  static int nLEDTimeout = -1;
  static int nLEDCounter = 0;
  static int iLED = 0;
  static int LEDPat[] =
  {
    XBOX360_LED_PAT_1_ON, XBOX360_LED_PAT_2_ON,
    XBOX360_LED_PAT_3_ON, XBOX360_LED_PAT_4_ON
  };

  // lazy init
  if( nLEDTimeout < 0 )
  {
    nLEDTimeout = countsPerSecond(0.50);
  }

  // switch pattern
  if( nLEDCounter++ >= nLEDTimeout )
  {
    iLED = (iLED + 1) % arraysize(LEDPat);
    setLED(LEDPat[iLED]);
    nLEDCounter = 0;
  }
}

