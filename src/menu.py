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

from menu_items import Item, ItemService, ItemAction, ItemKill
from ar_recog.msg import Tags, Tag
from std_msgs.msg import Header

class Menu(object): 

    def __init__(self):
        rospy.init_node("loco_menu")
        self.rate = rospy.Rate(30)
        rospy.Subscriber("/loco/tags", Tags, self.tag_callback)

        self.input = None
        self.input_time = None
        self.unhandled_input = False

        self.items = list()
        with open(rospy.get_param('~menu_def_file')) as file:
            output = yaml.load(file, Loader=yaml.SafeLoader)
            menu_data = output['menu']

            for item in menu_data:
                item = item['item']

                rospy.loginfo(item)
                if item['type'] == 'rosservice':
                    rospy.loginfo('Creating ServiceItem')
                    self.items.append(ItemService(item))
                elif item['type'] == 'rosaction':
                    rospy.loginfo('Creating ActionItem')
                    self.items.append(ItemAction(item))
                elif item['type'] == 'kill':
                    rospy.loginfo('Creating KillItem')
                    self.items.append(ItemKill(item))
    
    def tag_callback(self, data):
        if len(data.tags) > 0:
            self.input = data.tags[0]
            #self.input_time= data.header.stamp.sec

            rospy.loginfo('Recieved tag %d'%(self.input.id))

            if not self.unhandled_input:
                rospy.loginfo('No current input waiting in queue, so marking unhandled')
                self.unhandled_input = True
                    
    def menu_update(self):
        if self.unhandled_input:
            rospy.loginfo('Input waiting for handling, searching...')
            for idx, item in enumerate(self.items):
                rospy.loginfo('Index %d, Item: %r'%(idx, item))
                if idx == self.input.id:
                    rospy.loginfo('Item selected, executing')
                    item.execute()
                    self.unhandled_input = False
                    break


    def menu_graphical_update(self):
        pass    

if __name__ == '__main__':
    m = Menu()

    while not rospy.is_shutdown():
        m.menu_update()
        m.rate.sleep()
