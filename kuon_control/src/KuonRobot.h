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

// TODO: - move this to 

#include <string.h>
#include <string>

#include "rnr/serdev.h"

typedef struct KuonStatus
{
  bool isEStopped;
} KuonStatus_T;

typedef struct 
{
  // DHP - coming soon! as soon as we have encoders, etc
} KuonState;

class KuonRobot
{
public:
  KuonRobot()
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
  int estop();

  bool isEStopped(){ return m_bIsEstopped;}
  float QueryGovernorVal(){ return m_fGovernorVal;}
  float SetGovernorVal(float v)
  {
    if(v>1)
    {
      v=1.0;
    }
    else if(v<0)
    {
      v=0.0;
    }

    return m_fGovernorVal = v;
  };

protected: 
  int m_fdFrontMots;
  int m_fdRearMots;

  bool m_bIsEstopped;
  bool m_fGovernorVal;
};

#endif // _KUON_ROBOT_H
