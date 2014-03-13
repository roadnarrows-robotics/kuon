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
#include "kuon_control/BrakeCmd.h"      // publish
#include "kuon_control/KuonStatus.h"    // subscribe (TBD)
#include "kuon_control/KuonState.h"     // subscribe
#include "kuon_control/SlewCmd.h"       // publish
#include "kuon_control/SpeedCmd.h"      // publish
#include "kuon_control/Version.h"       // service

//
// ROS generatated Kuon services.
//
#include "kuon_control/EStop.h"
#include "kuon_control/IncrementGovernor.h"
#include "kuon_control/ResetEStop.h"
#include "kuon_control/QueryVersion.h"

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
      LEDPatOn     = XBOX360_LED_PAT_ALL_BLINK,   ///< default xbox on pattern
      LEDPatPaused = XBOX360_LED_PAT_ALL_SPIN_2,  ///< temp, auto-trans to blink
      LEDPatReady  = XBOX360_LED_PAT_ALL_SPIN     ///< spin
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
      return (m_eState == TeleopStateReady) && !m_msgRobotStatus.e_stopped;
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
    TeleopState       m_eState;         ///< teleoperation state
    bool              m_bHasXboxComm;   ///< Xbox communications is [not] good
    int               m_nWdXboxCounter; ///< Xbox watchdog counter
    int               m_nWdXboxTimeout; ///< Xbox watchdog timeout
    bool              m_bHasKuonComm;   ///< Kuon communications is [not] good
    int               m_nWdKuonCounter; ///< Kuon watchdog counter
    int               m_nWdKuonTimeout; ///< Kuon watchdog timeout
    bool              m_bHasFullComm;   ///< good full communications
    ButtonState       m_buttonState;    ///< saved button state

    // messages
    kuon_control::KuonStatus  m_msgRobotStatus; ///< saved last Kuon status 
    hid::ConnStatus           m_msgConnStatus;  ///< saved last conn status 


    //..........................................................................
    // Server Service callbacks
    //..........................................................................

    // none


    //..........................................................................
    // Client Servicec
    //..........................................................................

    void setLED(int pattern);

    void setRumble(int motorLeft, int motorRight);

    // TBD void setRobotMode(int mode);

    void estop();

    void resetEStop();

    void setGovernor(float delta);


    //..........................................................................
    // Topic Publishers
    //..........................................................................

    /*!
     * \brief Publish brake command.
     */
    void publishBrakeCmd(int brake);

    /*!
     * \brief Publish slew command.
     */
    void publishSlewCmd(int slew);

    /*!
     * \brief Publish speed command.
     */
    void publishSpeedCmd(int speedLeft, int speedRight);

    /*!
     * \brief Publish Xbox360 rumble command.
     */
    void publishRumbleCmd(int motorLeft, int motorRight);


    //..........................................................................
    // Subscribed Topic Callbacks
    //..........................................................................

    /*!
     * \brief Kuon status callback.
     *
     * \param msg Received subscribed topic.
     */
    void cbKuonStatus(const kuon_control::KuonStatus &msg);

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
    // Sanity
    //..........................................................................

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
  };

} // namespace kuon


#endif // _KUON_TELEOP_H
