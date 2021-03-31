#! /usr/bin/python3

# http://adamlamers.com/post/FTPD9KNRA8CT

import os, sys
import curses
import yaml




class UI(object):

    def __del__(self):
        print("GOODBYE")
        curses.endwin()
        #os.system('clear')

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

    def _draw_item(self, item_number, style):
        self.screen.addstr(5 + item_number,
                           4,
                           "{:2} - {}".format(item_number+1, self.menu_items['items'][item_number]),#"{:2} - {}".format(item_number+1, self.menu_items['items'][item_number]['display']),
                           style)

    def _draw_title(self):
        self.screen.addstr(2, 2, self.menu_items['title'], curses.A_STANDOUT)
        self.screen.addstr(4, 2, self.menu_items['subtitle'], curses.A_BOLD)

    def display(self):
        input_key = self.screen.getch()
        selected_item = self.prompt_selection(input_key)
        i, _ = self.screen.getmaxyx()

        if (selected_item != None):
            if selected_item < len(self.menu_items['items']):
                selected_opt = self.menu_items['items'][selected_item]   
                return selected_opt
            else:
                self.running = False
                return {'title' : 'Exit', 'type' : 'exitmenu'}
        return None

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
    while (u.running):
        u.display()
        u.get_menu_updates("test1 test2 ")
    del u