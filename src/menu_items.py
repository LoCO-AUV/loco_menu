import rospy
import pydoc

class Item(object):
    def __init__(self, item_yaml):
        self.name = item_yaml['display']


class ItemService(Item):
    def __init__(self,item_yaml):
        self.name = item_yaml['display']
        self.service = item_yaml['service']
        self.service_class = pydoc.locate(item_yaml['service_class'])
	
        
        rospy.wait_for_service(self.service)
        self.proxy = rospy.ServiceProxy(self.service, self.service_class)

        self.params = item_yaml['params']

    def __str__(self):
        return "Item: name=%r type=%r service=%r params=%r"%(self.name, "Service", self.service, self.params)

    def execute(self):
        rospy.loginfo('Executing service item')
        self.proxy(**self.params)


class ItemLaunch(Item):
    def __init__(self, item_yaml):
        self.name = item_yaml['display']

class ItemBag(Item):
    def __init__(self, item_yaml):
        self.name = item_yaml['displ[ay']

    
