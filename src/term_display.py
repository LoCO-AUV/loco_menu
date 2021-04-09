#! /usr/bin/python3

import rospy
import yaml

from std_msgs.msg import String

import os, sys


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

        # create named pipe
        ros2ui_path = "loco_display_ros2ui"
        ui2ros_path = "loco_display_ui2ros"
        files_made = False
        try:
            os.mkfifo(ros2ui_path)
            os.mkfifo(ui2ros_path)
        except OSError as e:
            if e.errno == 17:
                files_made = True
                rospy.loginfo("Created named files.")
            else:
                rospy.logerr("Failed to create named pipe. (%s)", e)
        else:   
            files_made = True
            rospy.loginfo("Created named files.")

        if files_made:
            self.ros2ui_pipe = open(ros2ui_path, 'w')
            rospy.loginfo("Created ros2ui pipe.")
            self.ui2ros_pipe = open(ui2ros_path, 'r')
            rospy.loginfo("Created ui2ros pipe.")
            rospy.loginfo("Created named pipes.")

   
    def display_update_callback(self, data):
        self.ros2ui_pipe.write(data)
        rospy.loginfo("sent data ('{0}') to menu display.".format(data))

if __name__=='__main__':
    d = Display()

    while not rospy.is_shutdown():
        d.rate.sleep()

    # end tasks
        # close pipe : os.unlink(path)