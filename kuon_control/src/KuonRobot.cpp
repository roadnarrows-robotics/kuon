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

#include "KuonRobot.h"
#include "rnr/serdev.h"

int KuonRobot::connect()
{
  m_fdFrontMots = SerDevOpen(m_strFrontMots.c_str(), 
                             KuonRobot::DEFAULT_BAUDRATE,
                             8, 'N', 1, false, false);

  m_fdRearMots = SerDevOpen(m_strRearMots.c_str(), 
                             KuonRobot::DEFAULT_BAUDRATE,
                             8, 'N', 1, false, false);

  if(m_fdFrontMots < 0 || m_fdRearMots < 0)
  {
    fprintf(stderr,"DHP - failed to open one of the motor controllers\n");
    return -1;
  }
  else
  {
    fprintf(stderr,"Motor controllers opened successfully\n");
    return 0;
  }
}

int KuonRobot::disconnect()
{
  int rc = 0;

  SerDevFIFOOutputFlush(m_fdRearMots);
  SerDevFIFOOutputFlush(m_fdFrontMots);

  if( SerDevClose(m_fdFrontMots) < 0)
  {
    rc=-1;
    fprintf(stderr, "Failed to close front motor controller\n");
  }

  if( SerDevClose(m_fdRearMots) < 0)
  {
    rc=-1;
    fprintf(stderr,"Failed to close rear motor controller\n");
  }

  return rc;
}


int KuonRobot::setSpeeds(int left, int right)
{
}

int KuonRobot::setSlew(int s)
{
}

int KuonRobot::setBrake(int b)
{
}

