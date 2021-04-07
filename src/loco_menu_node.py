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
import yaml

from menu_node import MenuNode
from menu import Menu
from menu_items import *

from ar_recog.msg import Tags, Tag
from std_msgs.msg import Header, String

# Global variables. 
menu_input = None
input_time = None
input_unhandled = False

# Callback for handling tag input data.
def tag_callback(data):
    global menu_input, input_time, input_unhandled

    if len(data.tags) > 0:
        menu_input =  data.tags[0]
        input_time = data.header.stamp.secs

        rospy.loginfo('Received tag %d'%(menu_input.id))

        if not input_unhandled:
            rospy.loginfo('No current input waiting in queue, so marking unhandled')
            input_unhandled = True

# Wrapper function for the Menu yaml processing
def build_menu(filepath, display_publisher):
    with open(filepath) as file:
        output = yaml.load(file, Loader=yaml.SafeLoader)
        root_menu = Menu(None, output['menu'], display_publisher)

    return root_menu

# Process the current input to effect a state change on the menu.
def menu_state_upate(menu):
    global menu_input, input_time, input_unhandled

    if menu_input.id == 0:
        if menu.parent is None:
            rospy.loginfo("No parent menu, remaining at current level.")

            input_unhandled = False
            return menu, False
        else:
            rospy.loginfo("Going to parent menu.")
            menu.foreground = False
            menu.foreground_since = None

            input_unhandled = False
            return menu.parent, True

    for idx, item in enumerate(menu.children):
        rospy.loginfo('Index %d, Item: %r'%(idx, item))
        if idx == (menu_input.id - 1):
            if type(item) is Menu:
                rospy.loginfo('Going to submenu: %s'%(item.name))
                item.foreground = True
                item.foreground_since = rospy.get_time()

                input_unhandled = False
                return item, True
            else:
                rospy.loginfo('Executing item %r'%(item.name))
                item.execute(menu.display_publisher)

                input_unhandled = False
                return menu, True
    
    input_unhandled = False
    return menu, True


# Send an updated version of the menu display string to the display device.
def menu_graphical_update(menu):
    rospy.loginfo('Sending graphical update.')
    rospy.loginfo(menu.to_display_string())

    msg = String()
    msg.data = menu.to_display_string()
    root_menu.display_publisher.publish(msg)

if __name__ == '__main__':
    rospy.init_node("loco_menu")
    rate = rospy.Rate(10)

    rospy.Subscriber("/loco/tags", Tags, tag_callback)
    display_pub = rospy.Publisher('/loco/menu_display', String, queue_size=1)

    root_menu = build_menu(rospy.get_param('~menu_def_file'), display_pub)
    menu_graphical_update(root_menu)

    while not rospy.is_shutdown():
        if input_unhandled:
            root_menu, display_changed = menu_state_upate(root_menu)
            if display_changed:
                menu_graphical_update(root_menu)

        rate.sleep()
        