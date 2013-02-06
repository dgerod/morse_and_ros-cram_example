import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('roscpp'); roslib.load_manifest('rospy'); roslib.load_manifest('std_msgs')
import rospy
import std_msgs
import GameLogic
import math
import mathutils

from std_msgs.msg import String

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    # Compose the name of the port, based on the parent and module names
    component_name = component_instance.blender_obj.name
    parent_name = component_instance.robot_parent.blender_obj.name

     # Add the new method to the component
    component_instance.output_functions.append(function)
 
    # Generate one publisher and one topic for each component that is a sensor and uses post_message
    self._topics.append(rospy.Publisher(parent_name + "/" + component_name, String))
    
    logger.info('######## ROS BUMPER PUBLISHER INITIALIZED ########')

def post_bumper_msg(self, component_instance):
    """ Publish the data of the semantic camera as a string-message with newlines (for better visualization in console).

    """
    pass
