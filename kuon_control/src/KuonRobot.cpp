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

#include <stdio.h>
#include <string>

#include "Kuon/RS160DControl.h"
#include "KuonRobot.h"

const char* dev0 = "/dev/ttyUSB0";
const char* dev1 = "/dev/ttyUSB1";

int KuonRobot::connect()
{
  int error = 0;
  error = RS160DOpenConnection(dev0, &m_fdFrontMots);
  if(error < 0) {
    LOGDIAG2("Failed to open front motor controller");
    exit(1);
  }
  error = RS160DOpenConnection(dev1, &m_fdRearMots);
  if(error < 0) {
    exit(1);
  }
  error = RS160DSetToSerial(m_fdFrontMots);
  if(error < 0) {
    exit(1);
  }
  error = RS160DSetToSerial(m_fdRearMots);
  if(error < 0) {
    exit(1);
  }
}

int KuonRobot::disconnect()
{
  RS160DEStop(m_fdFrontMots, m_fdRearMots);
  RS160DClose(m_fdRearMots);
  RS160DClose(m_fdFrontMots);
}

int KuonRobot::estop()
{
  RS160DEStop(m_fdFrontMots, m_fdRearMots);
  m_bIsEstopped = true;
}

int KuonRobot::setSpeeds(int left, int right)
{
  if(m_bIsEstopped)
  {
    fprintf(stderr,"Kuon estopped - aborting command\n");
    m_bInMotion = false;
    return 0;
  }

  const int right_mot = 0;
  const int left_mot  = 1;

  fprintf(stderr, "Sending speed command: \n");
  fprintf(stderr, "\t fds: %d %d\n", m_fdFrontMots, m_fdRearMots);
  fprintf(stderr, "\t speeds: %d %d\n", left, right);
  fprintf(stderr, "\t mots: %d %d\n", left_mot, right_mot);
  fprintf(stderr, "\t governor: %f\n", m_fGovernorVal);

  m_bInMotion = true;
  RS160DUpdateMotorSpeeds(int(left*m_fGovernorVal),  m_fdFrontMots, left_mot);
  RS160DUpdateMotorSpeeds(int(right*m_fGovernorVal), m_fdFrontMots, right_mot);
  RS160DUpdateMotorSpeeds(int(left*m_fGovernorVal),  m_fdRearMots,  left_mot);
  RS160DUpdateMotorSpeeds(int(right*m_fGovernorVal), m_fdRearMots,  right_mot);
}

int KuonRobot::setSlew(int s)
{
  if(m_bIsEstopped)
  {
    fprintf(stderr,"Kuon estopped - aborting command\n");
    return 0;
  }
  const int right_mot = 0;
  const int left_mot  = 1;

  if (s > 40) 
  {
    s = 40;
  }
  else if (s < 0) 
  {
    s=0;
  }

  RS160DAlterSlew(s, m_fdFrontMots, left_mot);
  RS160DAlterSlew(s, m_fdFrontMots, right_mot);
  RS160DAlterSlew(s, m_fdFrontMots, left_mot);
  RS160DAlterSlew(s, m_fdFrontMots, right_mot);
}

int KuonRobot::setBrake(int b)
{
  if(m_bIsEstopped)
  {
    fprintf(stderr,"Kuon estopped - aborting command\n");
    return 0;
  }

  const int right_mot = 0;
  const int left_mot  = 1;

  if (b > 31) 
  {
    b = 31;
  }
  else if (b < 0) 
  {
    b=0;
  }

  RS160DAlterBraking(b, m_fdFrontMots, left_mot);
  RS160DAlterBraking(b, m_fdFrontMots, right_mot);
  RS160DAlterBraking(b, m_fdFrontMots, left_mot);
  RS160DAlterBraking(b, m_fdFrontMots, right_mot);
}


