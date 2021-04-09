#! /usr/bin/python3

# http://adamlamers.com/post/FTPD9KNRA8CT

import os, sys
import curses
import yaml
import threading




class UI(object):

    def __del__(self):    
        curses.endwin()
        os.system('clear')
        print("GOODBYE SPACE COWBOY...")

    def __init__(self,):
        # set up curses
        menu = {'title' : 'LOCO Menu',
        'type' : 'menu',
        'subtitle' : '(Terminal Version)',
        'items': []}

        self.screen = curses.initscr()
        self.menu_items = menu
        self.selected_item = 0
        self._previously_selected_item = None
        self.highlighted_item = 0
        self._previously_highlighted_item = None
        self.running = True

        # pipe vars
        self.ros2ui_path = "loco_display_ros2ui"
        self.ui2ros_path = "loco_display_ui2ros"
        self.files_made = False
        self.ros2ui_status = "LOADING..."
        self.ui2ros_status = "LOADING..."

        #init curses and curses input
        curses.noecho()
        curses.cbreak()
        curses.start_color()
        curses.curs_set(0) #Hide cursor
        self.screen.keypad(1)

        #set up color pair for highlighted item
        curses.init_pair(1, curses.COLOR_BLACK, curses.COLOR_WHITE)
        self.hilite_color = curses.color_pair(1)
        self.normal_color = curses.A_NORMAL 

        #read in and set up menu
        with open("../menus/first_menu.yaml") as file:
            output = yaml.load(file, Loader=yaml.SafeLoader)
            for i in output['menu']:
                self.menu_items['items'].append(i['item']['display'])

        self._draw_menu()

        # start ros2ui thread
        self.ros2ui_thread = threading.Thread(target=self.ros2ui, daemon=True)
        self.ros2ui_thread.start()

        # start ui2ros thread
        self.ui2ros_thread = threading.Thread(target=self.ui2ros, daemon=True)
        self.ui2ros_thread.start()

    def prompt_selection(self, input_key, parent=None):
        # print(input_key)
        if parent is None:
            lastitem = "Exit"
        else:
            lastitem = "Return to previous menu ({})".format(parent['display'])

        item_count = len(self.menu_items['items'])

        ENTER_KEY = ord('\n')
        #while input_key != ENTER_KEY:
        if input_key != ENTER_KEY:
            self.selected_item = None

            if self.highlighted_item != self._previously_highlighted_item:
                self._previously_highlighted_item = self.highlighted_item

            #input_key = self.screen.getch()
            down_keys = [curses.KEY_DOWN, ord('j')]
            up_keys = [curses.KEY_UP, ord('k')]
            exit_keys = [ord('q')]

            if input_key in down_keys:
                if self.highlighted_item < item_count:
                    self.highlighted_item += 1
                else:
                    self.highlighted_item = 0

            if input_key in up_keys:
                if self.highlighted_item > 0:
                    self.highlighted_item -= 1
                else:
                    self.highlighted_item = item_count

            if input_key in exit_keys:
                self.highlighted_item = item_count #auto select exit and return
                #break

        if input_key == ENTER_KEY:
            self.selected_item = self.highlighted_item
        else:
            self.selected_item = None

        # self._draw_menu(input_key)
        #redraw screen
        self.screen.clear()
        self.screen.border(0)
        self._draw_title()
        for item in range(item_count):
            if self.highlighted_item == item:
                # print(item)
                self._draw_item(item, self.hilite_color)
            else:
                # print(item)
                self._draw_item(item, self.normal_color)

        if self.highlighted_item == item_count:
            # print(item_count)
            self.screen.addstr(5 + item_count, 4, "{:2} - {}".format(item_count+1,
               lastitem), self.hilite_color)
        else:
            # print(item_count)
            self.screen.addstr(5 + item_count, 4, "{:2} - {}".format(item_count+1,
                lastitem), self.normal_color)

        max_y, max_x = self.screen.getmaxyx()
        if input_key is not None:
            # print(input_key)
            self.screen.addstr(max_y-3, max_x - 5, "{:3}".format(self.highlighted_item))
        self.screen.refresh()

        return self.selected_item

    def _draw_menu(self, input_key = None, parent = None):
        if parent is None:
            lastitem = "Exit"
        else:
            lastitem = "Return to previous menu ({})".format(parent['display'])
        item_count = len(self.menu_items['items'])
        #redraw screen
        self.screen.clear()
        self.screen.border(0)
        self._draw_title()
        for item in range(item_count):
            if self.highlighted_item == item:
                # print(item)
                self._draw_item(item, self.hilite_color)
            else:
                # print(item)
                self._draw_item(item, self.normal_color)

        if self.highlighted_item == item_count:
            # print(item_count)
            self.screen.addstr(5 + item_count, 4, "{:2} - {}".format(item_count+1,
               lastitem), self.hilite_color)
        else:
            # print(item_count)
            self.screen.addstr(5 + item_count, 4, "{:2} - {}".format(item_count+1,
                lastitem), self.normal_color)

        max_y, max_x = self.screen.getmaxyx()
        if input_key is not None:
            self.screen.addstr(max_y-3, max_x - 5, "{:3}".format(self.highlighted_item))
        self.screen.refresh()

    def _draw_item(self, item_number, style):
        self.screen.addstr(5 + item_number,
                           4,
                           "{:2} - {}".format(item_number+1, self.menu_items['items'][item_number]),#"{:2} - {}".format(item_number+1, self.menu_items['items'][item_number]['display']),
                           style)

    def _draw_title(self):
        self.screen.addstr(2, 2, self.menu_items['title'], curses.A_STANDOUT)
        self.screen.addstr(4, 2, self.menu_items['subtitle'], curses.A_BOLD)

    def display(self):
        while (self.running):
            input_key = self.screen.getch()
            self.prompt_selection(input_key)
            i, _ = self.screen.getmaxyx()
            if self.selected_item != None:
                if self.selected_item >= len(self.menu_items['items']):
                    self.running = False
        curses.endwin()
        # self.ros2ui_thread.join()
        # self.ui2ros_thread.join()
        return

    def ros2ui(self):
        # set up pipe
        made_pipe = False
        try:
            os.mkfifo(self.ros2ui_path)
        except OSError as e:
            if e.errno == 17:
                made_pipe = True
            else:
                self.ros2ui_status = "ERROR"
        else:   
            made_pipe = True

        if made_pipe:
            self.ros2ui_pipe = open(self.ros2ui_path, 'r')
            self.ros2ui_status = "Good"
        self.menu_items['subtitle'] += " [ROS -> UI : '{0}'] [UI -> ROS : '{1}'] ".format(self.ros2ui_status, self.ui2ros_status)
        self._draw_menu()

        # loop and read
        while (self.running & made_pipe):
            menu_string = self.ros2ui_pipe.read()
            if len(menu_string) != 0:
                self.menu_items['items'] = menu_string.split()
                self._draw_menu()

    def ui2ros(self):
        # set up pipe
        made_pipe = False
        try:
            os.mkfifo(self.ui2ros_path)
        except OSError as e:
            if e.errno == 17:
                made_pipe = True
            else:
                self.ui2ros_status = "ERROR"
        else:   
            made_pipe = True

        if made_pipe:
            self.ui2ros_pipe = open(self.ui2ros_path, 'r')
            self.ui2ros_status = "Good"
        self.menu_items['subtitle'] += " [ROS -> UI : '{0}'] [UI -> ROS : '{1}'] ".format(self.ros2ui_status, self.ui2ros_status)
        self._draw_menu()


        # loop and write
        while (self.running & made_pipe):
            if self.selected_item != None:
                if os.write(self.ui2ros_pipe, self.selected_item) != len(self.selected_item):
                    print("oh no, didn't write everything")
                self.selected_item = None



if __name__=='__main__':
    u = UI()
    u.display()
    del u
