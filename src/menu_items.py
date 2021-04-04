#!/usr/bin/env python

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
import rosnode
import actionlib
import pydoc
import time

class Item(object):
    def __init__(self, item_yaml):
        self.name = item_yaml['display']


class ItemService(Item):
    def __init__(self,item_yaml):
        self.name = item_yaml['display']
        self.pre_delay = item_yaml['pre_delay']
        self.service = item_yaml['service']
        self.service_class = pydoc.locate(item_yaml['service_class'])
        
        rospy.wait_for_service(self.service)
        self.proxy = rospy.ServiceProxy(self.service, self.service_class)

        self.params = item_yaml['params']

    def __str__(self):
        return "Item: name=%r type=%r service=%r params=%r"%(self.name, "Service", self.service, self.params)

    def execute(self):
        rospy.loginfo('Executing service item')

        if(self.pre_delay > 0):
            rospy.loginfo("Waiting for %r seconds prior to execution."%(self.pre_delay))
            time.sleep(self.pre_delay)
            
        self.proxy(**self.params)

class ItemKill(Item):
    def __init__(self, item_yaml):
        self.name = item_yaml['display']
        self.pre_delay = item_yaml['pre_delay']
        self.node_names = item_yaml['node_names']

    def __str__(self):
        return "Item: name=%r kill nodes=%r"%(self.name, self.node_names)

    def execute(self):
        rospy.loginfo('Executing node kill item')

        if(self.pre_delay > 0):
            rospy.loginfo("Waiting for %r seconds prior to execution."%(self.pre_delay))
            time.sleep(self.pre_delay)
            

        rosnode.kill_nodes(self.node_names)

# Actions are provided by actionlib, and are similar to services, but have a goal, feedback in the meantime, 
class ItemAction(Item):
    def __init__(self, item_yaml):
        self.name = item_yaml['display']
        self.pre_delay = item_yaml['pre_delay']
        self.action = item_yaml['action']
        self.action_class = pydoc.locate(item_yaml['action_class'])
        self.goal_class = pydoc.locate(item_yaml['goal_class'])
        self.goal_data = item_yaml['goal_data']
        self.result_class = pydoc.locate(item_yaml['result_class'])

        self.wait = item_yaml['wait_for_result']
        self.wait_time = item_yaml['wait_time']
        self.display_feedback = item_yaml['display_feedback']

        self.action_client = actionlib.SimpleActionClient(self.action, self.action_class)
        self.action_client.wait_for_server()

    def __str__(self):
         return "Item: name=%r type=%r action=%r waiting?=%r wait_time=%r"%(self.name, "Action", self.action, self.wait, self.wait_time)

    def execute(self):
        rospy.loginfo("Executing action item")
        if(self.pre_delay > 0):
            rospy.loginfo("Waiting for %r seconds prior to execution."%(self.pre_delay))
            time.sleep(self.pre_delay)
            
        goal = self.goal_class()
        goal.diver_id = self.goal_data
        self.action_client.send_goal(goal)

        if self.wait:
            finished = self.action_client.wait_for_result(rospy.Duration.from_sec(self.wait_time))
            if not finished:
                rospy.loginfo("Canceling goal due to timeout.")
                self.action_client.cancel_goal()
        

class ItemNode(Item):
    def __init__(self, item_yaml):
        self.name = item_yaml['display']
        
    def __str__(self):
        return ""
    
    def execute(self):
        rospy.loginfo("Executing node item")
        #execute


class ItemBag(Item):
    def __init__(self, item_yaml):
        self.name = item_yaml['display']
        self.topics = item_yaml['topic_names']
        self.bag_name = item_yaml['bag_name']
    
    def __str__(self):
        return ""

    def execute(self):
        rospy.loginfo("Executing bag item")
        #Execute

class ItemLaunch(Item):
    def __init__(self, item_yaml):
        self.name = item_yaml['display']
