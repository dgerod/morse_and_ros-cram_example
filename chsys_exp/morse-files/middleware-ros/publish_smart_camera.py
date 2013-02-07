import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('roscpp'); roslib.load_manifest('rospy'); roslib.load_manifest('std_msgs'); roslib.load_manifest('smartcam_msgs');
import rospy
import std_msgs
import GameLogic
import math
import mathutils

from std_msgs.msg import String
from smartcam_msgs.msg import Object, ObjectArray
from geometry_msgs.msg import Pose

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    
    logger.info('######## SMART-CAM ros-publisher(object-msg) INIT ######## Begin')
        
    # Compose the name of the port, based on the parent and module names
    component_name = component_instance.blender_obj.name
    parent_name = component_instance.robot_parent.blender_obj.name

     # Add the new method to the component
    component_instance.output_functions.append(function)
 
    # Generate one publisher and one topic for each component that is a sensor and uses post_message
    self._topics.append(rospy.Publisher(parent_name + "/" + component_name, ObjectArray))
    
    logger.info('######## SMART-CAM ros-publisher(object-msg) INIT ######## End')

def post_object_msg(self, component_instance):
    """ Publish the data of the smart camera as a smartcam_msgs.

    """

    parent_name = component_instance.robot_parent.blender_obj.name
    ros_obj_list_msg = ObjectArray()                       

    # iterate through all objects of the component_instance and create one string
    
    obj_array = [];
    
    for obj in component_instance.local_data['visible_objects']:
        
        # prepare object   
        ros_obj = Object(name = str(obj['name']))
        
        ros_obj.type = str(obj['type'])
        ros_obj.description = str(obj['description'])
                
        ros_obj.pose = Pose(position=obj['position'], orientation=obj['orientation'])        
         
        # and add it to the array to be send
        obj_array.append(ros_obj)                  
    
    # publish the message on the correct topic
    
    ros_obj_list_msg.header.stamp = rospy.Time.now()
    ros_obj_list_msg.objects = obj_array
    
    active_topic = str("/" + parent_name + "/" + component_instance.blender_obj.name)

    for topic in self._topics:
        if str(topic.name) == active_topic:
            topic.publish(ros_obj_list_msg)
