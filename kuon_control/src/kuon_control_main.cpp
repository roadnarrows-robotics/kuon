////////////////////////////////////////////////////////////////////////////////
//
// Package: RoadNarrows Robotics ROS Kuon Package
//
// Link: https://github.com/roadnarrows-robotics/kuon
//
// ROS Node: kuon_control
//
// File: kuon_control.cpp
//
/*! \file
*
* \brief The ROS kuon_control node.
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

#include <string>
#include <stdio.h>

#include "ros/ros.h"

#include "rnr/log.h"
#include "kuon_control.h"

using namespace std;
using namespace kuon_control;

const string &NodeName = "kuon_control";

int main(int argc, char **argv)
{
  // set loglevel for RN libs
  LOG_SET_THRESHOLD(LOG_LEVEL_DIAG3);

  KuonControlNode kuon;
  int seq = 0;
  int rc  = 0;

  ros::init(argc, argv, NodeName.c_str());
  ros::NodeHandle n(NodeName.c_str());

  ROS_INFO("%s node started", NodeName.c_str());
  
  if( (rc = kuon.connect()) != 0)
  {
    ROS_ERROR("Failed to connect to Kuon");
    return rc;
  }

  // --- register services
  ros::ServiceServer estop         = n.advertiseService("estop", 
                                      &KuonControlNode::EStop, &kuon);

  ros::ServiceServer reset_estop   = n.advertiseService("reset_estop", 
                                      &KuonControlNode::ResetEStop, &kuon);

fprintf(stderr,"dhp4\n");
  //ros::ServiceServer query_version = n.advertiseService("reset_estop", 
                                      //&KuonControlNode::QueryVersion, &kuon);

  ROS_INFO(" -- Services registered!");

#if 0
  // --- register published topics
  ros::Publisher joint_states_pub = 
    n.advertise<sensor_msgs::JointState>("joint_states", 10);
  ROS_INFO(" -- Published topics registered!");
#endif

  // --- register subscribed topics
  ros::Subscriber speed_cmd_sub = n.subscribe("speed_cmd", 1, 
                                        &KuonControlNode::speed_cmdCB, &kuon);
  ros::Subscriber brake_cmd_sub = n.subscribe("brake_cmd", 1, 
                                        &KuonControlNode::brake_cmdCB, &kuon);
  ros::Subscriber slew_cmd_sub = n.subscribe("slew_cmd", 1, 
                                        &KuonControlNode::slew_cmdCB, &kuon);
  ROS_INFO(" -- Subscribed topics registered!");
  ros::Rate loop_rate(5);
  while(ros::ok())
  {

    /*
    int n;
    if((n = updateJointStates(joint_states, joint_states_ex)) > 0)
    {
      joint_states.header.seq=seq;
      joint_states_pub.publish(joint_states);

      joint_states_ex.header.seq=seq;
      joint_states_ex_pub.publish(joint_states_ex);
    }

    if(updateRobotStatus(robot_status, robot_status_ex) == 0)
    {
      robot_status.header.seq=seq;
      robot_status_pub.publish(robot_status);

      robot_status.header.seq=seq;
      robot_status_ex_pub.publish(robot_status_ex);
    }

    */

    ros::spinOnce(); 
    loop_rate.sleep();
    ++seq;
  }


  return 0;
}
