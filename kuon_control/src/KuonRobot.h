////////////////////////////////////////////////////////////////////////////////
//
// Package: RoadNarrows Robotics Kuon Package
//
// Link: https://github.com/roadnarrows-robotics/kuon
//
// ROS Node: kuon_control
//
// File: KuonRobot.h
//
/*! \file
*
* \brief The kuon robot class
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
#ifndef _KUON_ROBOT_H
#define _KUON_ROBOT_H

#include <string>

#include "Kuon/RS160DControl.h"

class KuonRobot
{
public:
  // --- class constants and enums
  static const int DEFAULT_BAUDRATE = 38400;

  KuonRobot(std::string dev1="/dev/ttyACM0", std::string dev2="/dev/ttyACM1")
    : m_strFrontMots(dev1), m_strRearMots(dev2)
  {
  }

  ~KuonRobot()
  {
    disconnect();
  }

  int connect();
  int disconnect();

  int setSpeeds(int left, int right);
  int setSlew(int s);
  int setBrake(int b);

  void setDeviceNames(std::string dev1, std::string dev2)
  {
    m_strFrontMots = dev1;
    m_strRearMots  = dev2;
  }

protected: 
  std::string m_strFrontMots;
  std::string m_strRearMots;
  int m_fdFrontMots;
  int m_fdRearMots;
};

#endif // _KUON_ROBOT_H
