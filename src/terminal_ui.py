#! /usr/bin/python3

import os, sys
import curses
class UI(object):
    def __del__(self):
        curses.endwin()
        os.system('clear')

    def __init__(self,):
        # # create named pipe
        # ros2ui_path = "/tmp/loco_display_ros2ui"
        # ui2ros_path = "/tmp/loco_display_ui2ros"
        # files_made = False
        # try:
        #     os.mkfifo(ros2ui_path)
        #     os.mkfifo(ui2ros_path)
        # except OSError as e:
        #     if e.errno == 17:
        #         files_made = True
        #     else:
        #         print("ERROR: Failed to create named pipe. (%s)", e)
        # else:   
        #     files_made = True

        # if files_made:
        #     os.chmod(ros2ui_path, 755)
        #     os.chmod(ui2ros_path, 755)
        #     self.ros2ui_pipe = open(ros2ui_path, 'r')
        #     self.ui2ros_pipe = open(ui2ros_path, 'w')

        # set up curses
        menu = {'title' : 'Curses Menu',
        'type' : 'menu',
        'subtitle' : 'A Curses menu in Python'}
        option_1 = {'title' : 'Hello World',
                    'type' : 'command',
                    'command' : 'echo Hello World!'}
        menu['options'] = [option_1]

        self.screen = curses.initscr()
        self.menu_options = menu
        self.selected_option = 0
        self._previously_selected_option = None
        self.running = True

        #init curses and curses input
        curses.noecho()
        curses.cbreak()
        curses.start_color()
        curses.curs_set(0) #Hide cursor
        self.screen.keypad(1)

        #set up color pair for highlighted option
        curses.init_pair(1, curses.COLOR_BLACK, curses.COLOR_WHITE)
        self.hilite_color = curses.color_pair(1)
        self.normal_color = curses.A_NORMAL

    def prompt_selection(self, parent=None):
        if parent is None:
            lastoption = "Exit"
        else:
            lastoption = "Return to previous menu ({})".format(parent['title'])

        option_count = len(self.menu_options['options'])

        input_key = None

        ENTER_KEY = ord('\n')
        while input_key != ENTER_KEY:
            if self.selected_option != self._previously_selected_option:
                self._previously_selected_option = self.selected_option

            self.screen.border(0)
            self._draw_title()
            for option in range(option_count):
                if self.selected_option == option:
                    self._draw_option(option, self.hilite_color)
                else:
                    self._draw_option(option, self.normal_color)

            if self.selected_option == option_count:
                self.screen.addstr(5 + option_count, 4, "{:2} - {}".format(option_count+1,
                    lastoption), self.hilite_color)
            else:
                self.screen.addstr(5 + option_count, 4, "{:2} - {}".format(option_count+1,
                    lastoption), self.normal_color)

            max_y, max_x = self.screen.getmaxyx()
            if input_key is not None:
                self.screen.addstr(max_y-3, max_x - 5, "{:3}".format(self.selected_option))
            self.screen.refresh()

            input_key = self.screen.getch()
            down_keys = [curses.KEY_DOWN, ord('j')]
            up_keys = [curses.KEY_UP, ord('k')]
            exit_keys = [ord('q')]

            if input_key in down_keys:
                if self.selected_option < option_count:
                    self.selected_option += 1
                else:
                    self.selected_option = 0

            if input_key in up_keys:
                if self.selected_option > 0:
                    self.selected_option -= 1
                else:
                    self.selected_option = option_count

            if input_key in exit_keys:
                self.selected_option = option_count #auto select exit and return
                break

        return self.selected_option

    def _draw_option(self, option_number, style):
        self.screen.addstr(5 + option_number,
                           4,
                           "{:2} - {}".format(option_number+1, self.menu_options['options'][option_number]['title']),
                           style)

    def _draw_title(self):
        self.screen.addstr(2, 2, self.menu_options['title'], curses.A_STANDOUT)
        self.screen.addstr(4, 2, self.menu_options['subtitle'], curses.A_BOLD)

    def display(self):
        selected_option = self.prompt_selection()
        i, _ = self.screen.getmaxyx()

        if selected_option < len(self.menu_options['options']):
            selected_opt = self.menu_options['options'][selected_option]
            return selected_opt
        else:
            self.running = False
            return {'title' : 'Exit', 'type' : 'exitmenu'}

    def updateMenu(self):
        menu_length = int(os.read(self.ros2ui_pipe,3)) # read in length of menu update
        menu_string = os.read(self.ros2ui_pipe,menu_length)
        print(menu_string)


if __name__=='__main__':
    u = UI()
    u.display()
    del u
    while (1):
        
        #u.updateMenu()