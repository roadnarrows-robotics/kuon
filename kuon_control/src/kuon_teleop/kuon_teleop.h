////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Kuon Robot Package
//
// Link:      https://github.com/roadnarrows-robotics/kuon
//
// ROS Node:  kuon_teleop
//
// File:      kuon_teleop.h
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief The ROS kuon_teleop node class interface.
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

#ifndef _KUON_TELEOP_H
#define _KUON_TELEOP_H

#include <string>
#include <map>

//
// Includes for boost libraries
//
#include <boost/bind.hpp>

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
#include "hid/Controller360State.h"   // subscribe
#include "hid/ConnStatus.h"           // subscribe
#include "hid/LEDPattern.h"           // service
#include "hid/RumbleCmd.h"            // publish

//
// ROS generatated Kuon services.
//
#include "hid/SetLED.h"
#include "hid/SetRumble.h"


namespace kuon
{
  /*!
   * \brief The class embodiment of the kuon_teleop ROS node.
   */
  class KuonTeleop
  {
  public:
    /*! map of ROS server services type */
    typedef std::map<std::string, ros::ServiceServer> MapServices;

    /*! map of ROS client services type */
    typedef std::map<std::string, ros::ServiceClient> MapClientServices;

    /*! map of ROS publishers type */
    typedef std::map<std::string, ros::Publisher> MapPublishers;

    /*! map of ROS subscriptions type */
    typedef std::map<std::string, ros::Subscriber> MapSubscriptions;

    /*!
     * \brief Teleoperation state.
     */
    enum TeleopState
    {
      TeleopStateUninit,    ///< not initialized
      TeleopStatePaused,    ///< paused
      TeleopStateReady,     ///< ready and running
    };

    /*!
     * \brief Xbox360 button map ids.
     */
    enum ButtonId
    {
      ButtonIdEStop   = rnr::Xbox360FeatIdBButton,      ///< emergency stop
      ButtonIdGovUp   = rnr::Xbox360FeatIdPadUp,        ///< governor speed up
      ButtonIdGovDown = rnr::Xbox360FeatIdPadDown,      ///< governor speed down
      ButtonIdPause   = rnr::Xbox360FeatIdBack,         ///< pause teleop
      ButtonIdStart   = rnr::Xbox360FeatIdStart,        ///< start teleop
      ButtonIdMoveX   = rnr::Xbox360FeatIdLeftJoyX,     ///< move fwd/bwd
      ButtonIdMoveY   = rnr::Xbox360FeatIdLeftJoyY,     ///< turn left/right
      ButtonIdBrake   = rnr::Xbox360FeatIdLeftTrigger,  ///< ease brake
      ButtonIdSlew    = rnr::Xbox360FeatIdRightTrigger  ///< slew
    };

    /*! teleop button state type */
    typedef std::map<int, int> ButtonState;

    /*!
     * \brief Xbox360 LED patterns.
     */
    enum LEDPat
    {
      LEDPatOff    = XBOX360_LED_PAT_ALL_OFF,     ///< all off
      LEDPatOn     = XBOX360_LED_PAT_ALL_BLINK,   ///< default xbox on pattern
      LEDPatPaused = XBOX360_LED_PAT_4_ON,        ///< pause teleop
      LEDPatReady  = XBOX360_LED_PAT_ALL_SPIN     ///< ready to teleop
    };

    /*!
     * \brief Default initialization constructor.
     *
     * \param nh  Bound node handle.
     * \param hz  Application nominal loop rate in Hertz.
     */
    KuonTeleop(ros::NodeHandle &nh, double hz);

    /*!
     * \brief Destructor.
     */
    virtual ~KuonTeleop();

    /*!
     * \brief Advertise all server services.
     */
    virtual void advertiseServices();

    /*!
     * \brief Initialize client services.
     */
    virtual void clientServices();

    /*!
     * \brief Advertise all publishers.
     *
     * \param nQueueDepth   Maximum queue depth.
     */
    virtual void advertisePublishers(int nQueueDepth=10);

    /*!
     * \brief Subscribe to all topics.
     *
     * \param nQueueDepth   Maximum queue depth.
     */
    virtual void subscribeToTopics(int nQueueDepth=10);

    /*!
     * \brief Publish.
     *
     * Call in main loop.
     */
    virtual void publish()
    {
      // No periodic publishing. All event driven.
    }

    /*!
     * \brief Check communications.
     *
     * Call in main loop.
     */
    virtual void commCheck();

    /*!
     * \brief Put robot into safe mode.
     *
     * \param bHard   Harden safe mode. When teleop node dies or xbox is 
     *                physically disconnected, robot is set to known defaults.
     */
    void putRobotInSafeMode(bool bHard);

    bool canMove()
    {
      if((m_eState == TeleopStateReady) &&
         (m_msgRobotStatus.e_stopped.val == industrial_msgs::TriState::FALSE) &&
         (m_msgRobotStatus.in_error.val == industrial_msgs::TriState::FALSE))
      {
        return true;
      }
      else
      {
        return false;
      }
    }

    /*!
     * \brief Get bound node handle.
     *
     * \return Node handle.
     */
    ros::NodeHandle &getNodeHandle()
    {
      return m_nh;
    }

    /*!
     * \brief Convert seconds to loop counts.
     *
     * \param seconds Seconds.
     *
     * \return Count.
     */
    int countsPerSecond(double seconds)
    {
      return (int)(seconds * m_hz);
    }

  protected:
    ros::NodeHandle  &m_nh;       ///< the node handler bound to this instance
    double            m_hz;       ///< application nominal loop rate

    // ROS services, publishers, subscriptions.
    MapServices       m_services;       ///< kuon teleop as server services
    MapClientServices m_clientServices; ///< kuon teleop as client services
    MapPublishers     m_publishers;     ///< kuon teleop publishers
    MapSubscriptions  m_subscriptions;  ///< kuon teleop subscriptions

    // state
    TeleopState       m_eState;           ///< teleoperation state
    bool              m_bHasXboxComm;     ///< Xbox communications is [not] good
    int               m_nWdXboxCounter;   ///< Xbox watchdog counter
    int               m_nWdXboxTimeout;   ///< Xbox watchdog timeout
    bool              m_bHasRobotComm;    ///< Kuon communications is [not] good
    int               m_nWdRobotCounter;  ///< Kuon watchdog counter
    int               m_nWdRobotTimeout;  ///< Kuon watchdog timeout
    bool              m_bHasFullComm;     ///< good full communications
    ButtonState       m_buttonState;      ///< saved button state

    // messages
    kuon_control::RobotStatusExtended m_msgRobotStatus;
                                                ///< saved last robot status 
    hid::ConnStatus           m_msgConnStatus;  ///< saved last conn status 


    //..........................................................................
    // Server Service callbacks
    //..........................................................................

    // none


    //..........................................................................
    // Client Services
    //..........................................................................

    /*!
     * \brief Set Xbox360 LED pattern client service request.
     *
     * \param pattern   LED pattern.
     */
    void setLED(int pattern);

    /*!
     * \brief Set Xbox360 left and right rumble motors client service request.
     *
     * \param motorLeft   Left motor speed.
     * \param motorRight  Right motor speed.
     */
    void setRumble(int motorLeft, int motorRight);

    /*!
     * \brief Emergency stop robot client service request.
     */
    void estop();

    /*!
     * \brief Freeze (stop) the robot with full brake applied client service
     * request.
     */
    void freeze();

    /*!
     * \brief Increment/decrement robot's speed limiting governor.
     *
     * \param delta     \h_plusmn delta from current governor setting..
     */
    void incrementGovernor(float delta);

    /*!
     * \brief Release (stop) the robot with no brake applied client service
     * request.
     */
    void release();

    /*!
     * \brief Reset emergency stop condition.
     */
    void resetEStop();

    /*!
     * \brief Set robot's speed limiting governor.
     *
     * \param governor  New governor setting.
     */
    void setGovernor(float governor);

    /*!
     * \brief Set robot's operation mode.
     *
     * \param mode    New rebot mode: auto or manual.
     */
    void setRobotMode(int mode);


    //..........................................................................
    // Topic Publishers
    //..........................................................................

    /*!
     * \brief Publish brake command.
     *
     * \param brake   Motor brake from coasting (0.0) to full brake (1.0).
     */
    void publishBrakeCmd(float brake);

    /*!
     * \brief Publish slew command.
     *
     * \param slew  Power slew command from instant full power applied (0.0) to 
     *              slowest power ramping (1.0).
     */
    void publishSlewCmd(float slew);

    /*!
     * \brief Publish speed command.
     *
     * \param speedLeft   Speed of left motors [-1.0, 1.0].
     * \param speedRight  Speed of right motors [-1.0, 1.0].
     */
    void publishSpeedCmd(double speedLeft, double speedRight);

    /*!
     * \brief Publish Xbox360 rumble command.
     *
     * \param motorLeft   Left rumble motor speed.
     * \param motorRight  Right rumble motor speed.
     */
    void publishRumbleCmd(int motorLeft, int motorRight);


    //..........................................................................
    // Subscribed Topic Callbacks
    //..........................................................................

    /*!
     * \brief Robot status callback.
     *
     * \param msg Received subscribed topic.
     */
    void cbRobotStatus(const kuon_control::RobotStatusExtended &msg);

    /*!
     * \brief Robot joint state callback.
     *
     * \param msg Received subscribed topic.
     */
    void cbJointState(const kuon_control::JointStateExtended &msg);

    /*!
     * \brief Xbox360 HID connectivity status callback.
     *
     * \param msg Received subscribed topic.
     */
    void cbXboxConnStatus(const hid::ConnStatus &msg);

    /*!
     * \brief Xbox360 HID button state callback.
     *
     * \param msg Received subscribed topic.
     */
    void cbXboxBttnState(const hid::Controller360State &msg);

    //..........................................................................
    // Xbox Actions
    //..........................................................................

    void msgToState(const hid::Controller360State &msg,
                    ButtonState                   &buttonState);

    bool buttonOffToOn(int id, ButtonState &buttonState)
    {
      return (m_buttonState[id] == 0) && (buttonState[id] == 1);
    }

    bool buttonDiff(int id, ButtonState &buttonState)
    {
      return m_buttonState[id] != buttonState[id];
    }

    void execAllButtonActions(ButtonState &buttonState);

    void buttonStart(ButtonState &buttonState);

    void buttonPause(ButtonState &buttonState);

    void buttonEStop(ButtonState &buttonState);

    void buttonGovernorUp(ButtonState &buttonState);

    void buttonGovernorDown(ButtonState &buttonState);

    void buttonBrake(ButtonState &buttonState);

    void buttonSlew(ButtonState &buttonState);

    void buttonSpeed(ButtonState &buttonState);

    //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
    // Support 
    //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

    /*!
     * \brief Go to pause teleoperation state.
     */
    void pause();

    /*!
     * \brief Go to ready to teleoperate state.
     */
    void ready();

    /*!
     * \brief Drive Xbox360 LEDs into a figure 8 pattern.
     */
    void driveLEDsFigure8Pattern();
  };

} // namespace kuon


#endif // _KUON_TELEOP_H
