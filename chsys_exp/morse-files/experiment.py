from math import pi
import random

from morse.builder import *
from morse_local_cfg import scenario_dir, robots_dir, objects_dir 

# Create a new ATRV robot, and change its name to MOUSE
# --------------------------------------------------------

mouse = Robot('atrv')
mouse.name = "MOUSE"
mouse.translate (x=1.0, z=0.2)

Pose1 = Sensor('pose')
Pose1.translate(x=-0.2000, z=0.9000)
mouse.append(Pose1)

# Next we make it controllable by the keyboard, using the correct actuator. 
# Also, we change the default speed, to make it more agile
Keyb = Actuator('keyboard')
Keyb.properties(Speed=3.0)
mouse.append(Keyb)

# Add properties so the mouse can be recognized by semantic camera sensors
mouse.properties(Object = True, Graspable = False, Label = mouse.name)

# Create another ATRV robot and set its name to CAT
# ----------------------------------------------------

Cat = Robot('atrv')
Cat.name = "CAT"
Cat.translate(x=-6.0, z=0.2)

Pose2 = Sensor('pose')
Pose2.translate(x=-0.2000, z=0.9000)
Cat.append(Pose2)

# Add also a v, omega actuator that will make the robot move:
V_W = Actuator('v_omega')
Cat.append(V_W)

Cat.append(Keyb)

# Camera on the top of the scene
# --------------------------------------------------------

videoCam = Robot( robots_dir + '/' + 'vcam.blend')
videoCam.name = 'VCAM'
#camera = Sensor('video_camera')
camera = Sensor('semantic_camera')

videoCam.append(camera)

videoCam.translate(z=20)
videoCam.rotate(y=pi/2);

# We export the sensors and actuators 
# --------------------------------------------------------

Pose1.configure_mw('ros')

V_W.configure_mw('ros')
Pose2.configure_mw('ros')

camera.configure_mw('ros')

# We add some objects in the scene
# --------------------------------------------------------

for i in range(5):

    box = PassiveObject( objects_dir + '/' + 'boxes.blend','RedBox')    
    box.translate( x=random.uniform(-10.0, 10.0),
                     y=random.uniform(-10.0, 10.0),
                     z=1.0000 )
    box.rotate( z=random.uniform(-pi,+pi) )
    
    #box.properties(Object = True, Graspable = False, Label = "BOX")

# And finally we complete the scene configuration:
# ----------------------------------------------------------

env = Environment( scenario_dir + '/' + 'empty_world.blend')

env.place_camera([10.0, -10.0, 10.0])
env.aim_camera([1.0470, 0, 0.7854])

