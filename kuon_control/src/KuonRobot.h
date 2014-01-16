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

// TODO: - move this to libkuon 

#include <stdio.h>
#include <string.h>
#include <string>

#include "rnr/serdev.h"

typedef struct KuonStatusStruct
{
  bool mode;   // false = manual, true = auto
  bool drives_powered;   // always true
  bool in_motion;    // true or false
  bool in_error;     // always false

  float governor_value;  // 0 to 1
  bool e_stopped;    // true or false
} KuonStatus_T;

typedef struct KuonStateStruct
{
  // DHP - coming soon! as soon as we have encoders, etc
} KuonState_T;

class KuonRobot
{
public:
  KuonRobot()
  {
    m_fGovernorVal = 0.1;
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
  int estop();
  int resetEStop(){m_bIsEstopped = false;}

  KuonStatus_T updateStatus()
  {
    KuonStatus_T s;
    s.e_stopped = m_bIsEstopped;
    s.mode = false;
    s.drives_powered = true;
    s.in_motion = m_bInMotion;
    s.in_error = false;

    s.governor_value = m_fGovernorVal;
    fprintf(stderr, " governor value = %f\n", m_fGovernorVal);
    return s;
  }
  KuonState_T  updateState();
  KuonStatus_T queryStatus(){return m_Status;}
  KuonState_T  queryState(){return m_State;}

  bool isEStopped(){ return m_bIsEstopped;}
  float QueryGovernorVal(){ return m_fGovernorVal;}
  float SetGovernorVal(float v)
  {
    if(v>1.0)
    {
      v=1.0;
    }
    else if(v<0.0)
    {
      v=0.0;
    }

    return m_fGovernorVal = v;
  };

protected: 
  int m_fdFrontMots;
  int m_fdRearMots;

  KuonStatus_T m_Status;
  KuonState_T  m_State;

  bool m_bIsEstopped;
  bool m_bInMotion;
  float m_fGovernorVal;
};

#endif // _KUON_ROBOT_H
