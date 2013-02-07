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
    
    logger.info('######## SMART-CAM ros-publisher INIT ######## Begin')
        
    # Compose the name of the port, based on the parent and module names
    component_name = component_instance.blender_obj.name
    parent_name = component_instance.robot_parent.blender_obj.name

     # Add the new method to the component
    component_instance.output_functions.append(function)
 
    # Generate one publisher and one topic for each component that is a sensor and uses post_message
    self._topics.append(rospy.Publisher(parent_name + "/" + component_name, String))
    
    logger.info('######## SMART-CAM ros-publisher INIT ######## End')

    
def post_string_msg(self, component_instance):
    """ Publish the data of the semantic camera as a string-message with newlines (for better visualization in console).

    """
    parent_name = component_instance.robot_parent.blender_obj.name
    string = String()
               
    for topic in self._topics: 
        message = str("")
        #iterate through all objects of the component_instance and create one string
        for obj in component_instance.local_data['visible_objects']:
            #if object has no description, set to '-'
            if obj['description'] == '':
                description = '-'
            # Build string from name, description, location and orientation in the global world frame
            message = message + "[" + str(obj['name']) + ", " + description + ", " + str(obj['position']) + ", " + str(obj['orientation']) + " ]\n"    
            string.data = message
        # publish the message on the correct topic    
        if str(topic.name) == str("/" + parent_name + "/" + component_instance.blender_obj.name):
            topic.publish(string)

def post_object_msg(self, component_instance):
    """ Publish the data of the semantic camera as a string-message with newlines (for better visualization in console).

    """
    logger.info('######## ROS SMART-CAM PUBLISHER DATA ########')
    
    parent_name = component_instance.robot_parent.blender_obj.name
    ros_obj_list = ObjectArray()
               
    for topic in self._topics: 
        message = str("")
        #iterate through all objects of the component_instance and create one string
        for obj in component_instance.local_data['visible_objects']:
            
            ros_obj = Object()
            
            #if object has no description, set to '-'
            if obj['description'] == '':
                description = '-'
            
            # Build string from name, description, location and orientation in the global world frame
            message = message + "[" + str(obj['name']) + ", " + description + ", " + str(obj['position']) + ", " + str(obj['orientation']) + " ]\n"
            
            ros_obj.name = str(obj['name'])
            ros_obj.pose = Pose( obj['position'] )
            rob_obj.object_id = str("0");
            
            ros_obj_list.add(ros_obj);                            
            
        # publish the message on the correct topic    
        if str(topic.name) == str("/" + parent_name + "/" + component_instance.blender_obj.name):
            topic.publish(ros_obj_list)
