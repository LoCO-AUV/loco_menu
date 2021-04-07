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
import rospkg
import rosnode
import roslaunch
import actionlib


from menu_node import MenuNode
from std_msgs.msg import String

import pydoc
import time

#Base class
class Item(MenuNode):
    def __init__(self, parent, item_yaml):
        super(Item, self).__init__(parent)
        self.name = item_yaml['name']
        self.pre_delay = item_yaml['pre_delay']
        self.post_delay = item_yaml['post_delay']
        self.background = item_yaml['background']
        self.kill_after_timeout = item_yaml['kill_after_timeout']
        self.timeout = item_yaml['timeout']


# Services for loco_menu
class ItemService(Item):
    def __init__(self, parent, item_yaml):
        super(ItemService, self).__init__(parent, item_yaml)
        
        self.service = item_yaml['service']
        self.service_class = pydoc.locate(item_yaml['service_class'])
        self.params = item_yaml['params']

        # rospy.wait_for_service(self.service)
        self.proxy = rospy.ServiceProxy(self.service, self.service_class)

    def __str__(self):
        return "Item: name=%r type=%r service=%r params=%r"%(self.name, "Service", self.service, self.params)

    def execute(self, display_pub):
        rospy.loginfo('Executings service item')
        msg = String()
        msg.data = "LIT; EXEC:%s"%(self.name)
        display_pub.publish(msg)

        if(self.pre_delay > 0):
            rospy.loginfo("Waiting for %r seconds prior to execution."%(self.pre_delay))
            rospy.sleep(self.pre_delay)
            
        self.proxy(**self.params)

        if(self.post_delay > 0):
            rospy.loginfo("Waiting for %r seconds after execution."%(self.post_delay))
            rospy.sleep(self.post_delay)

# Node killing for loco menu
class ItemKill(Item):
    def __init__(self, parent, item_yaml):
        super(ItemKill, self).__init__(parent, item_yaml)
        self.node_names = item_yaml['node_names']

    def __str__(self):
        return "Item: name=%r kill nodes=%r"%(self.name, self.node_names)

    def execute(self, display_pub):
        rospy.loginfo('Executing node kill item')
        msg = String()
        msg.data = "LIT; EXEC:%s"%(self.name)
        display_pub.publish(msg)

        if(self.pre_delay > 0):
            rospy.loginfo("Waiting for %r seconds prior to execution."%(self.pre_delay))
            rospy.sleep(self.pre_delay)    

        rosnode.kill_nodes(self.node_names)

        if(self.post_delay > 0):
            rospy.loginfo("Waiting for %r seconds after execution."%(self.post_delay))
            rospy.sleep(self.post_delay)


# Actions are provided by actionlib, and are similar to services, but have a goal, feedback in the meantime, 
class ItemAction(Item):
    def __init__(self, parent, item_yaml):
        super(ItemAction, self).__init__(parent, item_yaml)

        self.action = item_yaml['action']
        self.action_class = pydoc.locate(item_yaml['action_class'])
        self.goal_class = pydoc.locate(item_yaml['goal_class'])
        self.result_class = pydoc.locate(item_yaml['result_class'])
        self.feedback_class = pydoc.locate(item_yaml['feedback_class'])
        self.goal_data = item_yaml['goal_data']
        

        self.action_client = actionlib.SimpleActionClient(self.action, self.action_class)
        self.action_client.wait_for_server()

    def __str__(self):
         return "Item: name=%r type=%r action=%r"%(self.name, "Action", self.action)


    def execute(self, display_pub):
        rospy.loginfo("Executing action item")

        msg = String()
        msg.data = "LIT; EXEC:%s"%(self.name)
        display_pub.publish(msg)

        if(self.pre_delay > 0):
            rospy.loginfo("Waiting for %r seconds prior to execution."%(self.pre_delay))
            rospy.sleep(self.pre_delay)
            
        goal = self.goal_class()
        goal.diver_id = self.goal_data
        self.action_client.send_goal(goal)

        if ((not self.background) and (self.timeout > 0)):
            finished = self.action_client.wait_for_result(rospy.Duration.from_sec(self.timeout))
            if not finished and self.kill_after_timeout:
                rospy.loginfo("Canceling goal due to timeout.")
                self.action_client.cancel_goal()

        if(self.post_delay > 0):
            rospy.loginfo("Waiting for %r seconds after execution."%(self.post_delay))
            rospy.sleep(self.post_delay)

# Launch files for LoCO Menu
class ItemLaunch(Item):
    def __init__(self, parent, item_yaml):
        super(ItemLaunch, self).__init__(parent, item_yaml)

        self.package = item_yaml['package']
        self.launch_file = item_yaml['launch_file']
        self.args = item_yaml['args']


        rospack = rospkg.RosPack()
        package_path = rospack.get_path(self.package)
        self.lf_path = package_path + '/launch/' + self.launch_file

    def __str__(self):
        return "Item: name=%r type=%r pkg=%r file=%r"%(self.name, "Launch", self.package, self.launch_file)

    
    def execute(self, display_pub):
        rospy.loginfo('Executing launch item')
        msg = String()
        msg.data = "LIT; EXEC:%s"%(self.name)
        display_pub.publish(msg)

        # Pre-delay
        if(self.pre_delay > 0):
            rospy.loginfo("Waiting for %r seconds prior to execution."%(self.pre_delay))
            rospy.sleep(self.pre_delay)    

        # Prep and launch launch file.
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [self.lf_path])
        self.launch.start()

        # If we're doing foreground, do that here.
        if ((not self.background) and (self.timeout > 0)):
            rospy.sleep(self.timeout)
            if self.kill_after_timeout:
                self.launch.shutdown()

        # Post-delay
        if(self.post_delay > 0):
            rospy.loginfo("Waiting for %r seconds after execution."%(self.post_delay))
            rospy.sleep(self.post_delay)


class ItemBag(Item):
    def __init__(self, parent, item_yaml):
        super(ItemBag, self).__init__(parent, item_yaml)
        self.topics = item_yaml['topic_names']
        self.bag_name = item_yaml['bag_name']
    
    def __str__(self):
        return ""

    def execute(self):
        rospy.loginfo("Executing bag item")
        #Execute


class ItemNode(Item):
    def __init__(self, parent, item_yaml):
        super(ItemNode, self).__init__(parent, item_yaml)
        
    def __str__(self):
        return ""
    
    def execute(self):
        rospy.loginfo("Executing node item")
        #execute