#! /usr/bin/python

import rospy
import yaml

from menu_items import Item, ItemService
from ar_recog.msg import Tags, Tag
from std_msgs.msg import Header

class Menu(object): 

    def __init__(self):
        rospy.init_node("minibot_menu")
        self.rate = rospy.Rate(30)
        rospy.Subscriber("/minibot/tags", Tags, self.tag_callback)

        self.items = list()
        with open(rospy.get_param('~menu_def_file')) as file:
            output = yaml.load(file, Loader=yaml.SafeLoader)
            menu_data = output['menu']

            for item in menu_data:
                item = item['item']

                if item['type'] == 'rosservice':
                    self.items.append(ItemService(item))

            for item in self.items:
                print(item)
        
        self.input = None
        self.input_time = None
        self.unhandled_input = False

    
    def tag_callback(self, data):
        self.input = data.tags[0]
        self.input_time= data.header.stamp.sec

        rospy.loginfo('Recieved tag %d'%(input.id))

        if not self.unhandled_input:
            rospy.loginfo('No current input waiting in queue, so marking unhandled')
            self.unhandled_input = True

    def menu_update(self):
        if self.unhandled_input:
            rospy.loginfo('Input waiting for handling, searching...')
            for idx, item in self.items:
                rospy.loginfo('Index %d, Item: %r'%(idx, item))
                if idx == self.input.id:
                    rospy.loginfo('Item selected, executing')
                    item.execute()
                    self.unhandled_input = False
                    break


    def menu_graphical_update(self):
        pass    

if __name__ == '__main__':
    m = Menu()

    while not rospy.is_shutdown():
        m.menu_update()
        m.rate.sleep()
