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
from menu_items import Item, ItemAction, ItemBag, ItemKill, ItemLaunch, ItemNode, ItemService

from ar_recog.msg import Tags, Tag
from std_msgs.msg import Header, String

class Menu(MenuNode): 
    def __init__(self, parent, yaml_data, display_publisher):
        super(Menu, self).__init__(parent)
        self.display_publisher = display_publisher

        # print(yaml_data)
        menu_data = yaml_data
        self.name = menu_data['name']
        
        for node in menu_data['items']:
            node_type = list(node.keys())[0]
            node_yaml = node[node_type]

            if node_type == 'menu':
                self.children.append(Menu(self, node_yaml, display_publisher))

            elif node_type == 'rosaction':
                rospy.loginfo('Creating ActionItem')
                self.children.append(ItemAction(self, node_yaml))

            elif node_type == 'rosservice':
                rospy.loginfo('Creating ServiceItem')
                self.children.append(ItemService(self, node_yaml))

            elif node_type == 'roslaunch':
                rospy.loginfo('Creating LaunchItem')
                self.children.append(ItemLaunch(self, node_yaml))
            
            elif node_type == 'rosbag':
                rospy.loginfo('Creating BagItem')
                self.children.append(ItemBag(self, node_yaml))

            elif node_type == 'rosnode':
                rospy.loginfo('Creating NodeItem')
                self.children.append(ItemNode(self, node_yaml))
                
            elif node_type == 'kill_nodes':
                rospy.loginfo('Creating KillItem')
                self.children.append(ItemKill(self, node_yaml))

    def __str__(self):
        ret = "Menu: %s\nChildren:{ "%(self.name)
        for c in self.children:
            ret += str(c) + '; '
        ret += '}'
        return ret

    def to_display_string(self):
        display_string = "LIT;      %s;"%(self.name)
        for idx, item in enumerate(self.children):
            display_string += "%r. %s;"%((idx+1), item.name)
        return display_string
