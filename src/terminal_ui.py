#! /usr/bin/python3

import os, sys
class UI(object):


    def __init__(self,):
        # create named pipe
        ros2ui_path = "/tmp/loco_display_ros2ui"
        ui2ros_path = "/tmp/loco_display_ui2ros"
        files_made = False
        try:
            os.mkfifo(ros2ui_path)
            os.mkfifo(ui2ros_path)
        except OSError as e:
            if e.errno == 17:
                files_made = True
            else:
                print("ERROR: Failed to create named pipe. (%s)", e)
        else:   
            files_made = True

        if files_made:
            os.chmod(ros2ui_path, 755)
            os.chmod(ui2ros_path, 755)
            self.ros2ui_pipe = open(ros2ui_path, 'r')
            self.ui2ros_pipe = open(ui2ros_path, 'w')

    def updateMenu(self):
        menu_length = int(os.read(self.ros2ui_pipe,3)) # read in length of menu update
        menu_string = os.read(self.ros2ui_pipe,menu_length)
        print(menu_string)


if __name__=='__main__':
    u = UI()
    while (1):
       u.updateMenu()