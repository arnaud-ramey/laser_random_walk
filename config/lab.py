""" Basic MORSE simulation scene for <my_first_sim> environment

Feel free to edit this template as you like!
"""
from morse.builder import *
# https://www.openrobots.org/morse/doc/stable/components_library.html
robot = ATRV()
robot.translate(4, 2, 0.0)
motion = MotionVW()
robot.append(motion)

keyboard = Keyboard()
robot.append(keyboard)
keyboard.properties(ControlType = 'Position')

pose = Pose()
robot.append(pose)

odometry = Odometry()
robot.append(odometry)

scan = Hokuyo()
robot.append(scan)
scan.properties(Visible_arc = True)
scan.properties(laser_range = 30.0)
scan.properties(resolution = 1.0)
scan.properties(scan_window = 180.0)
scan.create_laser_arc()

robot.add_interface('ros')
motion.add_interface('ros')
pose.add_stream('ros')
scan.add_stream('ros')
odometry.add_interface('ros')

# set 'fastmode' to True to switch to wireframe mode
# ls /usr/local/share/morse/data/environments/
#~ env = Environment('indoors-1/indoor-1', fastmode = False)
#~ env = Environment('sandbox', fastmode = False)
env = Environment('tum_kitchen/tum_kitchen', fastmode = True)
env.set_camera_location([4, 2, 15])
env.set_camera_rotation([0, 0, 0])
