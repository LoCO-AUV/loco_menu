import rospy
import rosnode
import pydoc
import rosbag

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


class ItemKill(Item):
    def __init__(self, item_yaml):
        self.name = item_yaml['display']
        self.node_names = item_yaml['node_names']

    def __str__(self):
        return "Item: name=%r kill nodes=%r"%(self.name, self.node_names)

    def execute(self):
        rospy.loginfo('Executing node kill item')
        rosnode.kill_nodes(self.node_names)
    

class ItemBag(Item):
    def __init__(self, item_yaml):
        self.name = item_yaml['display']
        # bagmode, currently set to append

    def execute(self):
        self.bag = rosbag.Bag(self.name,'a')
        # subscribe to /rosout
        # write all messages

    #def rosout_callback(self)

class ItemLaunch(Item):
    def __init__(self, item_yaml):
        self.name = item_yaml['display']
    
    def execute(self):
        pass
