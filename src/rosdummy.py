#! /usr/bin/python3

import os, sys
import curses
import yaml

class UI(object):

    def __del__(self):
        print("GOODBYE")
        #curses.endwin()
        #os.system('clear')

    def __init__(self,):
        # set up curses
        menu = {'title' : 'LOCO Menu',
        'type' : 'menu',
        'subtitle' : '(Terminal Version)',
        'items': []}

        # create named pipes
        self.ros2ui_path = "loco_display_ros2ui"
        self.ui2ros_path = "loco_display_ui2ros"
        self.files_made = False
        try:
            os.mkfifo(self.ros2ui_path)
            # os.mkfifo(self.ui2ros_path)
        except OSError as e:
            if e.errno == 17:
                self.files_made = True
            else:
                print("ERROR: Failed to create named pipe. (%s)", e)
        else:   
            self.files_made = True

        if self.files_made:
            self.ros2ui_pipe = open(self.ros2ui_path, 'w')
            # self.ui2ros_pipe = open(self.ui2ros_path, 'r')

        
        self.ros2ui_pipe.write("003")
        self.ros2ui_pipe.write("test1 test2 test3")
        print("Finished Writing!")
        # menu_string = self.ui2ros_pipe.read()
        # if len(menu_string) != 0:
        #     print("READ '{0}".format(menu_string))

    def get_menu_updates(self, menu_string):
        #menu_length = int(os.read(self.ros2ui_pipe,3)) # read in length of menu update
        #menu_string = os.read(self.ros2ui_pipe,menu_length)
        self.menu_items['items'] = menu_string.split()

    def send_menu_selection(self, selected_opt):
        if os.write(self.ui2ros_pipe, selected_opt) == len(selected_opt):
            # wrote everything
            print(":D")
        else:
            # uh oh
            print("oh no, didn't write everything")


if __name__=='__main__':
    u = UI()
    # while(1):
    # while (u.running):
        #u.display()
        #u.get_menu_updates("test1 test2 ")
    # del u