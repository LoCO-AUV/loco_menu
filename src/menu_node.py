#! /usr/bin/python3

# This code is a part of the LoCO AUV project.
# Copyright (C) The Regents of the University of Minnesota

# Maintainer: Junaed Sattar <junaed@umn.edu> and the Interactive Robotics and Vision Laboratory

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import rospy

class MenuNode(object):
    def __init__(self, parent):
        self.parent = parent
        self.children = list()
        
        self.running = False
        self.running_since = None
        self.foreground = False
        self.foreground_since = None

    def activate(self):
        self.running = True
        self.running_since = rospy.get_time()
        self.foreground = True
        self.foreground_since = rospy.get_time()

    def deactivate(self):
        self.running = False
        self.running_since = None
        self.foreground = False
        self.foreground_since = None
        
    def set_running(self, val):
        if val:
            self.running = True
            self.running_since = rospy.get_time()
        else:
            self.running = False
            self.running_since = None

    def set_foreground(self, val):
        if val:
            self.foreground = True
            self.foreground_since = rospy.get_time()
        else:
            self.foreground = False
            self.foreground_since = None
