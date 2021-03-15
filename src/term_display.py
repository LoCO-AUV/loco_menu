#! /usr/bin/python3

import rospy
import yaml

from std_msgs.msg import String


#import consolemenu # may use for final console menu
# from consolemenu import *
# from consolemenu.items import *


class Display(object):
    
    def __init__(self,):
        rospy.init_node("loco_display")
        self.rate = rospy.Rate(30)
        rospy.Subscriber("/loco/display", String, self.display_update_callback)

        with open(rospy.get_param('display_def_file')) as file:
            output = yaml.load(file, Loader=yaml.SafeLoader)
            self.lines = output['lines']


        
    def display_update_callback(self, data):
        #parse for newline characters
        lines = data.split("\n");

        print(chr(27) + "[2J")  # clear the menu
        for l in lines:         # print the menu
            rospy.loginfo(l)
        # TODO make this an actual menu

if __name__=='__main__':
    d = Display()

    while not rospy.is_shutdown():
        d.rate.sleep()