#! /usr/bin/python3

import rospy
import yaml

from menu_items import Item, ItemService, ItemKill
from ar_recog.msg import Tags, Tag
from std_msgs.msg import Header, String

class Menu(object): 

    def __init__(self):
        rospy.init_node("loco_menu")
        self.rate = rospy.Rate(30)
        rospy.Subscriber("/loco/tags", Tags, self.tag_callback)

        self.input = None
        self.input_time = None
        self.unhandled_input = False

        # reads in number of lines on display. 
        # can read in height/width here too.
        # reads in name of display publisher
        with open(rospy.get_param('display_def_file')) as file:
            display_output = yaml.load(file, Loader=yaml.SafeLoader)
            self.lines = display_output['lines']
            rospy.loginfo("Loaded number of lines: %d", self.lines)
            self.display_pub = rospy.Publisher(display_output['publisher_name'], String, queue_size= 10)
            rospy.loginfo("Created display publisher: %s", display_output['publisher_name'])

        # read in items 
        self.items = list()
        with open(rospy.get_param('~menu_def_file')) as file:
            output = yaml.load(file, Loader=yaml.SafeLoader)
            menu_data = output['menu']
            rospy.loginfo('Found %d items in the menu yaml', len(menu_data))

            for item in menu_data:
                item = item['item']
                if item['type'] == 'rosservice':
                    rospy.loginfo('Creating ServiceItem: %s', item['display'])
                    self.items.append(ItemService(item))
                elif item['type'] == 'kill':
                    rospy.loginfo('Creating KillItem: %s', item['display'])
                    self.items.append(ItemKill(item))
                elif item['type'] == 'bag':
                    rospy.loginfo('Creating BagItem: %s', item['display'])
                    self.items.append(ItemBag(item))
                elif item['type'] == 'launch':
                    rospy.loginfo('Creating LaunchItem: %s', item['display'])
                    self.items.append(ItemLaunch(item))
        rospy.loginfo("Finished Reading!")
    
    def tag_callback(self, data):
        if len(data.tags) > 0:
            self.input = data.tags[0]
            #self.input_time= data.header.stamp.sec

            rospy.loginfo('Recieved tag %d'%(self.input.id))

            if not self.unhandled_input:
                rospy.loginfo('No current input waiting in queue, so marking unhandled')
                self.unhandled_input = True
                    
    def menu_update(self):
        if self.unhandled_input:
            rospy.loginfo('Input waiting for handling, searching...')
            for idx, item in enumerate(self.items):
                rospy.loginfo('Index %d, Item: %r'%(idx, item))
                if idx == self.input.id:
                    rospy.loginfo('Item selected, executing')
                    item.execute()
                    self.unhandled_input = False
                    break


    def menu_graphical_update(self):
        menu_entries = ""
        for item in self.items:
            menu_entries += item.name + "\n"
        menu_entries = "item 1 \n item 2 \n item 3 \n" #debug
        self.display_pub.publish(menu_entries)  
        rospy.loginfo('Published to display.')

if __name__ == '__main__':
    m = Menu()

    while not rospy.is_shutdown():
        m.menu_update()
        m.menu_graphical_update()
        m.rate.sleep()
