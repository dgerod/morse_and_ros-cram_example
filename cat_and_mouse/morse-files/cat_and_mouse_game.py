from morse.builder import *

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

#mouse = Mouse();
#mouse.add_controller( Keyb )

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

# Next add two semantic cameras to the robot. This will provide us with 
# an easy way to follow our target. One is placed right and one left, to 
# provide a fake stereo vision

Semantic_R = Sensor('semantic_camera')
Semantic_R.translate(x=0.2, y=-0.3, z=0.9)
Semantic_R.name = 'Camera_R'
Cat.append(Semantic_R)

Semantic_L = Sensor('semantic_camera')
Semantic_L.translate(x=0.2, y=0.3, z=0.9)
Semantic_L.name = 'Camera_L'
Cat.append(Semantic_L)

# Add also a v, omega actuator that will make the robot move:
V_W = Actuator('v_omega')
Cat.append(V_W)

# We configure these two components to use the sockets middleware:
# --------------------------------------------------------

Pose1.configure_mw('ros')

V_W.configure_mw('ros')
Semantic_L.configure_mw('ros')
Semantic_R.configure_mw('ros')
Pose2.configure_mw('ros')

# And finally we complete the scene configuration:
# ----------------------------------------------------------

env = Environment('land-1/trees')
env.place_camera([10.0, -10.0, 10.0])
env.aim_camera([1.0470, 0, 0.7854])
 

