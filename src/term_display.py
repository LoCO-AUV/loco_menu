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
        ui2ros_path = "/tmp/loco_display_ui2ros"
        files_made = False
        try:
            os.mkfifo(ros2ui_path)
            os.mkfifo(ui2ros_path)
        except OSError as e:
            if e.errno == 17:
                files_made = True
            else:
                rospy.logerr("ERROR: Failed to create named pipe. (%s)", e)
        else:   
            files_made = True

        if files_made:
            os.chmod(ros2ui_path, 755)
            os.chmod(ui2ros_path, 755)
            self.ros2ui_pipe = open(ros2ui_path, 'r')
            self.ui2ros_pipe = open(ui2ros_path, 'w')


        
    def display_update_callback(self, data):
        pass
        # os.write(self.pipe, data)
        # self.pipe.flush()
        # os.fsync(self.pipe.fileno())
        #parse for newline characters
        # lines = data.split("\n");

        # print(chr(27) + "[2J")  # clear the menu
        # for l in lines:         # print the menu
        #     rospy.loginfo(l)
        # TODO make this an actual menu

if __name__=='__main__':
    d = Display()

    while not rospy.is_shutdown():
        d.rate.sleep()

    # end tasks
        # close pipe : os.unlink(path)