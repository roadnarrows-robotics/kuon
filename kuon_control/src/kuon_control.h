////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Kuon Robotiic Mobile Platform ROS Package
//
// Link:      https://github.com/roadnarrows-robotics/kuon
//
// ROS Node:  kuon_control
//
// File:      kuon_control.h
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief The ROS kuon_control node class interface.
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

#ifndef _KUON_CONTROL_H
#define _KUON_CONTROL_H

//
// System
//
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
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "control_msgs/FollowJointTrajectoryAction.h"

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
#include "Kuon/kuonRobot.h"

//
// Node headers.
//
#include "kuon_control.h"


namespace kuon_control
{
  /*!
   * \brief The class embodiment of the kuon_control ROS node.
   */
  class KuonControl
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
     * \brief Default initialization constructor.
     *
     * \param nh  Bound node handle.
     * \param hz  Application nominal loop rate in Hertz.
     */
    KuonControl(ros::NodeHandle &nh, double hz);

    /*!
     * \brief Destructor.
     */
    virtual ~KuonControl();

    /*!
     * \brief Configure Kuon product specifics.
     *
     * \param strCfgFile    XML configuration file name.
     *
     * \return Returns KUON_OK of success, \h_lt 0 on failure.
     */
    virtual int configure(const std::string &strCfgFile);

    /*!
     * \brief Connect to Kuon hardware.
     *
     * \param strDevMotorCtlr0    Motor controller 0.
     * \param strDevMotorCtlr1    Motor controller 1.
     * \param nBaudRateMotorCtlr  Motor ontroller baud rate.
     *
     * \return Returns KUON_OK of success, \h_lt 0 on failure.
     */
    int connect(const std::string &strDevMotorCtlr0,
                const std::string &strDevMotorCtlr1,
                int                nBaudRateMotorCtlr)
    {
      return m_robot.connect(strDevMotorCtlr0, strDevMotorCtlr1,
                             nBaudRateMotorCtlr);
    }

    /*!
     * \brief Disconnect from Kuon.
     *
     * \return Returns KUON_OK of success, \h_lt 0 on failure.
     */
    int disconnect()
    {
      m_robot.disconnect();
    }

    /*!
     * \brief Advertise all server services.
     */
    virtual void advertiseServices();

    /*!
     * \brief Initialize client services.
     */
    virtual void clientServices()
    {
      // No client services
    }

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
    virtual void publish();

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
     * \brief Get bound embedded robot instance.
     *
     * \return Robot instance.
     */
    kuon::KuonRobot &getRobot()
    {
      return m_robot;
    }

    /*!
     * \brief Update joint state message from current robot joint state.
     *
     * \param [in] state  Robot joint state.
     * \param [out] msg   Joint state message.
     */
    void updateJointStateMsg(kuon::KuonJointStatePoint &state,
                             sensor_msgs::JointState   &msg);

    /*!
     * \brief Update extended joint state message from current robot joint
     * state.
     *
     * \param [in] state  Robot joint state.
     * \param [out] msg   Extended joint state message.
     */
    void updateExtendedJointStateMsg(kuon::KuonJointStatePoint &state,
                                     JointStateExtended        &msg);

    /*!
     * \brief Update robot status message from current robot status.
     *
     * \param [in] status Robot status.
     * \param [out] msg   Robot status message.
     */
    void updateRobotStatusMsg(kuon::KuonRobotStatus        &status,
                              industrial_msgs::RobotStatus &msg);

    /*!
     * \brief Update extended robot status message from current robot status.
     *
     * \param [in] status Robot status.
     * \param [out] msg   Extended roobt status message.
     */
    void updateExtendedRobotStatusMsg(kuon::KuonRobotStatus &status,
                                      RobotStatusExtended   &msg);

  protected:
    ros::NodeHandle  &m_nh;       ///< the node handler bound to this instance
    double            m_hz;       ///< application nominal loop rate
    kuon::KuonRobot   m_robot;    ///< real-time, Kuon robotic mobile platform

    // ROS services, publishers, subscriptions.
    MapServices       m_services;       ///< Kuon control server services
    MapClientServices m_clientServices; ///< Kuon control client services
    MapPublishers     m_publishers;     ///< Kuon control publishers
    MapSubscriptions  m_subscriptions;  ///< Kuon control subscriptions

    // Messages for published data.
    sensor_msgs::JointState       m_msgJointState;  ///< joint state message
    JointStateExtended            m_msgJointStateEx;
                                              ///< extended joint state message
    industrial_msgs::RobotStatus  m_msgRobotStatus; ///< robot status message
    RobotStatusExtended           m_msgRobotStatusEx;
                                              ///< extended robot status message

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Service callbacks
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Emergency stop robot service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool estop(kuon_control::EStop::Request  &req,
               kuon_control::EStop::Response &rsp);

    /*!
     * \brief Freeze (stop) robot service callback.
     *
     * The motor 'parking' brake is set to full.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool freeze(kuon_control::Freeze::Request  &req,
                kuon_control::Freeze::Response &rsp);

    /*!
     * \brief Get robot product information service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool getProductInfo(kuon_control::GetProductInfo::Request  &req,
                        kuon_control::GetProductInfo::Response &rsp);

    /*!
     * \brief Increment/decrement robot speed limit governor service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool incrementGovernor(kuon_control::IncrementGovernor::Request  &req,
                           kuon_control::IncrementGovernor::Response &rsp);

    /*!
     * \brief Test if robot is alarmed service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool isAlarmed(kuon_control::IsAlarmed::Request  &req,
                   kuon_control::IsAlarmed::Response &rsp);

    /*!
     * \brief Test if robot description has been loaded service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool isDescLoaded(kuon_control::IsDescLoaded::Request  &req,
                      kuon_control::IsDescLoaded::Response &rsp);

    /*!
     * \brief Release drive power to robot motors service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool release(kuon_control::Release::Request  &req,
                 kuon_control::Release::Response &rsp);

    /*!
     * \brief Release robot's emergency stop condition service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool resetEStop(kuon_control::ResetEStop::Request  &req,
                    kuon_control::ResetEStop::Response &rsp);

    /*!
     * \brief Set robot speed limit governor service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool setGovernor(kuon_control::SetGovernor::Request  &req,
                     kuon_control::SetGovernor::Response &rsp);

    /*!
     * \brief Set robot's manual/auto mode service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool setRobotMode(kuon_control::SetRobotMode::Request  &req,
                      kuon_control::SetRobotMode::Response &rsp);

    /*!
     * \brief Stop (freeze) robot service callback.
     *
     * The motor 'parking' brake is set to full.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool stop(kuon_control::Stop::Request  &req,
              kuon_control::Stop::Response &rsp);


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Topic Publishers
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Publish joint state and extended joint state topics.
     */
    void publishJointState();

    /*!
     * \brief Publish robot status and extended robot status topics.
     */
    void publishRobotStatus();


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Subscribed Topic Callbacks
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Execute set brake subscribed topic callback.
     *
     * \param msg Subscribed message.
     */
    void execBrakeCmd(const kuon_control::BrakeCmd &msg);

    /*!
     * \brief Execute set power slew subscribed topic callback.
     *
     * \param msg Subscribed message.
     */
    void execSlewCmd(const kuon_control::SlewCmd &msg);

    /*!
     * \brief Execute set speed subscribed topic callback.
     *
     * \param msg Subscribed message.
     */
    void execSpeedCmd(const kuon_control::SpeedCmd &msg);

    /*!
     * \brief Execute move trajectory subscribed topic callback.
     *
     * \param jt  Joint trajectory message.
     */
    void execMoveCmd(const trajectory_msgs::JointTrajectory &jt);


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Utilities
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Convert kuon_control Units message 'enum' to standard RN units.
     *
     * \param u   Units to convert.
     *
     * \return Converted units.
     */   
    units_t toUnits(uint_t u);
  };

} // namespace hc


#endif // _KUON_CONTROL_H
