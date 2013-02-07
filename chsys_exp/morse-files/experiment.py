from math import pi
import random

from morse.builder import *
from morse_local_cfg import scenario_dir, robots_dir, objects_dir, mw_dir

from morse.builder.creator import SensorCreator

# Create another ATRV robot and set its name to CAT
# ----------------------------------------------------

cat = Robot('atrv')
cat.name = "CAT"
cat.properties(Object = True, Graspable = False, Label = cat.name)
cat.translate(x=-6.0, z=0.2)

pose = Sensor('pose')
pose.translate(x=-0.2000, z=0.9000)
cat.append(pose)

#front_bumper = SensorCreator('Bumper', "./" + robots_dir + "bumper", "BumperClass", 
#                          "./" + robots_dir + "bumper.blend")

front_bumper = Sensor(robots_dir + 'bumper')
front_bumper.translate(x=0.6, z=0.3)

front_bumper.properties(scan_window = 180)
front_bumper.properties(laser_range = 0.5)

cat.append(front_bumper)

# Add also a v, omega actuator that will make the robot move:
vw = Actuator('v_omega')
cat.append(vw)

# Next we make it controllable by the keyboard, using the correct actuator. 
# Also, we change the default speed, to make it more agile
keyb = Actuator('keyboard')
keyb.properties(Speed=3.0)
cat.append(keyb)

# Camera on the top of the scene
# --------------------------------------------------------

videoCam = Robot( './robots/' + 'vcam.blend')
videoCam.name = 'VCAM'
camera = Sensor('smart_camera')

videoCam.append(camera)

videoCam.translate(z=20)
videoCam.rotate(y=pi/2);

# We add some objects in the scene
# --------------------------------------------------------

for i in range(3):

    box = PassiveObject( objects_dir + 'boxes.blend','RedBox')    
    box.properties(Type = "BOX")
    
    box.translate( x=random.uniform(-10.0, 10.0),
                   y=random.uniform(-10.0, 10.0),
                   z=1.0000 )
    box.rotate( z=random.uniform(-pi,+pi) )
    
for i in range(3):

    cylinder = PassiveObject( objects_dir + 'cylinders.blend','GreenCylinder')
    cylinder.properties(Type = "CYLINDER")
            
    cylinder.translate( x=random.uniform(-10.0, 10.0),
                        y=random.uniform(-10.0, 10.0),
                        z=1.0000 )
    cylinder.rotate( z=random.uniform(-pi,+pi) )

# We export the sensors and actuators through ROS topics
# --------------------------------------------------------

vw.configure_mw('ros')
pose.configure_mw('ros')
front_bumper.configure_mw('ros',
                          ['ROS','post_bumper_msg',mw_dir + 'publish_bumper'])

camera.configure_mw('ros',
                    ['ROS','post_string_msg',mw_dir + 'publish_smart_camera'])
    
# And finally we complete the scene configuration:
# ----------------------------------------------------------

env = Environment( scenario_dir + 'empty_world.blend')

env.place_camera([10.0, -10.0, 10.0])
env.aim_camera([1.0470, 0, 0.7854])

