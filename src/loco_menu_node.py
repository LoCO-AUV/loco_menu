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
from time import sleep

from loco_oled.ssd1325 import SSD1325

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

            return menu, False
        else:
            rospy.loginfo("Going to parent menu.")
            menu.set_foreground(False)

            return menu.parent, True

    for idx, item in enumerate(menu.children):
        rospy.loginfo('Index %d, Item: %r'%(idx, item))
        if idx == (menu_input.id - 1):
            if type(item) is Menu:
                rospy.loginfo('Going to submenu: %s'%(item.name))
                item.set_foreground(True)

                return item, True
            else:
                rospy.loginfo('Executing item %r'%(item.name))
                item.execute()
                
                return menu, True
    
    return menu, True


# Send an updated version of the menu display string to the display device.
def menu_graphical_update(oled, menu):
    item_types = [Item, ItemService, ItemAction, ItemLaunch, ItemBag, ItemKill, ItemNode]
    oled.set_cursor(0,0)
    oled.set_text_size(1)
    oled.clear_display()

    oled.print_line(f"0. <--  {menu.name}")
    oled.print_line("---------------------")
    for i, c in enumerate(menu.children):
        if type(c) is Menu:
            oled.print_line(f"{i+1}. {c.name}")
        elif type(c) in item_types and not c.running:
            oled.print_line(f"{i+1}. {c.name}")
        elif type(c) in item_types and c.running:
            oled.print_line(f"{i+1}. Kill {c.name}")

    oled.display()

if __name__ == '__main__':
    oled = SSD1325(name="loco_menu")
    oled.configure_dimmensions()
    oled.clear_display()
    oled.draw_loco_logo()
    oled.display()
    sleep(2)

    oled.set_text_size(1)
    oled.set_text_color(1) # Important to remmeber to actually set text color.
    oled.clear_display()
    oled.display()

    rospy.Subscriber("/loco/tags", Tags, tag_callback)

    root_menu = build_menu(rospy.get_param('~menu_def_file'), oled)
    menu_graphical_update(oled, root_menu)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if input_unhandled:
            root_menu, display_changed = menu_state_upate(root_menu)
            if display_changed:
                menu_graphical_update(oled, root_menu)
            rospy.sleep(1)
            input_unhandled = False

        rate.sleep()
        