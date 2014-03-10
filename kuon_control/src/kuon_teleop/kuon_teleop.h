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
    /*! map of ROS services type */
    typedef std::map<std::string, ros::ServiceServer> MapServices;

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
      TeleopStatePause,     ///< paused
      TeleopStateReady,     ///< ready and running
    };

    /*!
     * \brief Default initialization constructor.
     *
     * \param nh  Bound node handle.
     */
    KuonTeleop(ros::NodeHandle &nh);

    /*!
     * \brief Destructor.
     */
    virtual ~KuonTeleop();

    /*!
     * \brief Advertise all services.
     */
    virtual void advertiseServices();

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
     * \brief Get bound node handle.
     *
     * \return Node handle.
     */
    ros::NodeHandle &getNodeHandle()
    {
      return m_nh;
    }

  protected:
    ros::NodeHandle  &m_nh;       ///< the node handler bound to this instance

    // ROS services, publishers, subscriptions.
    MapServices       m_services;       ///< kuon teleop services
    MapPublishers     m_publishers;     ///< kuon teleop publishers
    MapSubscriptions  m_subscriptions;  ///< kuon teleop subscriptions

    // state
    TeleopState       m_eState;         ///< teleoperation state
    bool              m_bHasXboxComm;   ///< good communications with Xbox
    int               m_nWdXboxCounter; ///< Xbox watchdog counter
    bool              m_bHasKuonComm;   ///< good communications with Kuon
    int               m_nWdKuonCounter; ///< Kuon watchdog counter
    bool              m_bHasFullComm;   ///< good full communications
    hid::Controller360State m_msgXboxState; ///< saved last Xbox button state


    //..........................................................................
    // Service callbacks
    //..........................................................................

    // none


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

    /*!
     * \brief Put robot into safe mode.
     */
    void putRobotInSafeMode();


    //..........................................................................
    // Xbox Button Actions
    //..........................................................................

    void execAllButtonActions(const hid::Controller360State &msg);

    bool buttonOffToOn(int id);

    void buttonStart();
  };

} // namespace kuon


#endif // _KUON_TELEOP_H
