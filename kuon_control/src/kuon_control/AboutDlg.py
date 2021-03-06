###############################################################################
#
#
# Package:   RoadNarrows Robotics Kuon Robotic Mobile Platform ROS Package
#
# Link:      https://github.com/roadnarrows-robotics/kuon
#
# ROS Node:  kuon_*
#
# File:      AboutDlg.py
#
## \file 
##
## $LastChangedDate$
## $Rev$
##
## \brief Kuon about dialog.
##
## \author Daniel Packard (daniel@roadnarrows.com)
## \author Robin Knight (robin.knight@roadnarrows.com)
##  
## \par Copyright:
##   (C) 2014-2016.  RoadNarrows LLC.\n
##   (http://www.roadnarrows.com)\n
##   All Rights Reserved
##
# @EulaBegin@
# @EulaEnd@
#
###############################################################################

import sys
import os
import time

from Tkinter import *
from Tkconstants import *
from tkFileDialog import *
import tkFont

import webbrowser

from kuon_control.Utils import *


# ------------------------------------------------------------------------------
# Class AboutDlg
# ------------------------------------------------------------------------------

#
## \brief Kuon about dialog.
##
class AboutDlg(Toplevel):
  #
  ## \brief Constructor.
  ##
  ## \param cnf     Configuration dictionary.
  ## \param kw      Keyword options.
  #
  def __init__(self, master=None, cnf={}, **kw):
    # initialize dialog data
    kw = self.initData(kw)

    Toplevel.__init__(self, master=master, cnf=cnf, **kw)
    self.title("About Kuon")

    # parent widget's window geometry
    if master is not None:
      self.m_parentGeo = [master.winfo_width(), master.winfo_height(),
                          master.winfo_rootx(), master.winfo_rooty()]
    else:
      self.m_parentGeo = [400, 400, 400, 400]

    #print 'DBG: Parent geometry = {0}x{1}+{2}+{3}'.format(*self.m_parentGeo)

    # Set a good location for this dialog overlaying on top of the parent's
    # geometry. This is a compromise in that this dialog's geometry has not
    # been determined yet.
    glist= [self.m_parentGeo[2] + self.m_parentGeo[0]/6,
            self.m_parentGeo[3] + 30]
    self.geometry('+{0}+{1}'.format(*glist))

    # create and show widgets
    self.createWidgets()

    # allows the enter button to fire either button's action
    self.m_bttnOk.bind('<KeyPress-Return>', func=self.close)

    # start with ok button focused
    #self.m_bttnOk.focus_set()

    # allows us to customize what happens when the close button is pressed
    self.protocol("WM_DELETE_WINDOW", self.close)

    #
    # Modal diagle settings.
    #
    # set the focus on dialog window (needed on Windows)
    self.focus_set()

    # make sure events only go to our dialog
    self.grab_set()

    # make sure dialog stays on top of its parent window (if needed)
    self.transient(master)

    # display the window and wait for it to close
    self.wait_window(self)

  #
  ## \brief Initialize class state data.
  ##
  ## \param kw      Keyword options.
  ##
  ## \return Modified keywords sans this specific class.
  ##
  def initData(self, kw):
    self.m_icons          = {}    # must keep loaded icons referenced
    self.m_prodName       = "Kuon"
    self.m_hwVer          = "0.9.2"
    self.m_prodId         = 0x00
    self.m_prodBrief      = "Kuon Outdoor Robotic Mobile Platform"
    self.m_appVer         = "0.0.0"
    self.m_rnUrl          = "http://www.roadnarrows.com/Kuon"
    self.m_rnEmail        = "support@roadnarrows.com"
    self.m_rnTel          = "+1.800.275.9568"
    if kw.has_key('info'):
      info = kw['info']
      if info is not None:
        self.m_hwVer = info.version_string
        self.m_prodName = info.product_name
        self.m_prodId = info.product_id
        self.m_prodBrief = info.desc
      del kw['info']
    if kw.has_key('app_ver'):
      self.m_appVer = kw['app_ver']
      del kw['app_ver']
    return kw

  #
  ## \brief Create gui widgets with supporting data and show.
  #
  def createWidgets(self):
    imageLoader = ImageLoader(py_pkg="kuon_control.images")

    frame = Frame(self)
    frame.grid(row=0, column=0)

    self.m_icons['kuon_logo'] = imageLoader.load("KuonLogo.png")

    # top heading
    w = Label(frame)
    times32 = tkFont.Font(family="Times",size=32,weight="bold")
    w['font']   = times32
    w['text']   = 'Kuon'
    w['anchor'] = W
    w.grid(row=0, column=1, columnspan=2, sticky=E+W)

    bg      = w['bg']
    lwidth  = 10
    rwidth  = 24

    row = 1

    # product brief
    w = Text(frame)
    w['wrap']   = WORD
    w['width']  = lwidth + rwidth + 5
    w['height']  = 2
    w['relief']  = 'flat'
    w['bg']  = bg
    w.insert(END, self.m_prodBrief)
    w['state']  = 'disabled'
    w.grid(row=row, column=1, columnspan=2, padx=2, sticky=W)

    row += 1

    # product name
    w = Label(frame)
    w['text']   = 'Product:'
    w['anchor'] = W
    w['width'] = lwidth
    w.grid(row=row, column=1, padx=2, sticky=W)

    w = Label(frame)
    w['text']   = self.m_prodName
    w['anchor'] = W
    w['width']  = rwidth
    w.grid(row=row, column=2, padx=2, sticky=W)

    row += 1

    # product id
    w = Label(frame)
    w['text']   = 'Product Id:'
    w['anchor'] = W
    w['width'] = lwidth
    w.grid(row=row, column=1, padx=2, sticky=W)

    w = Label(frame)
    w['text']   = "%02x" % (self.m_prodId)
    w['anchor'] = W
    w['width']  = rwidth
    w.grid(row=row, column=2, padx=2, sticky=W)

    row += 1

    # hw version
    w = Label(frame)
    w['text']   = 'HW Version:'
    w['anchor'] = W
    w['width'] = lwidth
    w.grid(row=row, column=1, padx=2, sticky=W)

    w = Label(frame)
    w['text']   = self.m_hwVer
    w['anchor'] = W
    w['width']  = rwidth
    w.grid(row=row, column=2, padx=2, sticky=W)

    row += 1

    # app version
    w = Label(frame)
    w['text']   = 'App Version:'
    w['anchor'] = W
    w['width'] = lwidth
    w.grid(row=row, column=1, padx=2, sticky=W)

    w = Label(frame)
    w['text']   = self.m_appVer
    w['anchor'] = W
    w['width']  = rwidth
    w.grid(row=row, column=2, padx=2, sticky=W)

    row += 1

    # url
    w = Label(frame)
    w['text']   = 'URL:'
    w['anchor'] = W
    w['width'] = lwidth
    w.grid(row=row, column=1, padx=2, sticky=W)

    w = Button(frame)
    w['text']   = 'www.roadnarrows.com/Kuon'
    w['fg']   = '#aa0000'
    w['activeforeground']   = '#cc0033'
    w['activebackground']   = w['bg']
    w['cursor'] = 'hand1'
    w['anchor'] = W
    w['justify'] = LEFT
    w['relief']   = 'flat'
    w['borderwidth']   = 0
    w['padx']   = 0
    w['pady']   = 0
    w['width']  = rwidth
    w['command']  = lambda aurl=self.m_rnUrl:webbrowser.open_new(self.m_rnUrl)
    w.grid(row=row, column=2, ipadx=0, ipady=0, padx=2, pady=0, sticky=W)

    row += 1

    # support email
    w = Label(frame)
    w['text']   = 'Email:'
    w['anchor'] = W
    w['width'] = lwidth
    w.grid(row=row, column=1, padx=2, sticky=W)

    w = Label(frame)
    w['text']   = self.m_rnEmail
    w['anchor'] = W
    w['width']  = rwidth
    w.grid(row=row, column=2, padx=2, sticky=W)

    row += 1

    # telephone
    w = Label(frame)
    w['text']   = 'Tel:'
    w['anchor'] = W
    w['width'] = lwidth
    w.grid(row=row, column=1, padx=2, sticky=W)

    w = Label(frame)
    w['text']   = self.m_rnTel
    w['anchor'] = W
    w['width']  = rwidth
    w.grid(row=row, column=2, padx=2, sticky=W)

    row += 1

    # product logo
    w = Label(frame)
    if self.m_icons['kuon_logo'] is not None:
      w['image']  = self.m_icons['kuon_logo']
    w['anchor'] = CENTER
    w.grid(row=0, column=0, rowspan=row, sticky=W+N+S)

    # who
    w = Label(frame)
    w['text']   = """
The RoadNarrows Kuon robotic mobile platform is designed and developed by 
RoadNarrows, a robotics and intelligent systems company base in Colorado USA.
We are dedictated to supporting open software and hardware interfaces to foster
a global community of users and developers."""
    w['justify'] = CENTER
    w['anchor'] = CENTER
    w.grid(row=row, column=0, columnspan=3, padx=10, sticky=W+E)

    row += 1

    # ok button
    w = Button(frame, width=10, text='OK', command=self.close)
    w.grid(row=row, column=0, columnspan=3, pady=5)
    w['anchor']  = CENTER
    self.m_bttnOk = w

    row += 1

    # legal
    w = Label(frame)
    helv8 = tkFont.Font(family="Helvetica",size=8)
    w['font']   = helv8
    w['anchor'] = W
    w['fg']     = '#666666'
    w['text']   = "Kuon and the Kuon logos are the trademarks of " \
                  "RoadNarrows LLC"
    w.grid(row=row, column=0, columnspan=3, pady=5)

  #
  ## \brief Destroy window callback.
  #
  def close(self):
    self.destroy()
