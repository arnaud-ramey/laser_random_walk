#! /usr/bin/env morseexec

""" SLAM env """

from morse.builder import *
# add robot
#~ robot = Hummer()
robot = Pioneer3DX()
robot.translate(1.0, 0.0, 0.0)
# Add a motion controller
motion = MotionVW()
robot.append(motion)
# Add a keyboard controller to move the robot with arrow keys.
keyboard = Keyboard()
robot.append(keyboard)
keyboard.properties(ControlType = 'Position')
# Add a pose sensor that exports the current location and orientation
# of the robot in the world frame
pose = Pose()
robot.append(pose)
hokuyo = Hokuyo()
robot.append(hokuyo)
# To ease development and debugging, we add a socket interface to our robot.
motion.add_stream('ros')
pose.add_stream('ros')
hokuyo.add_stream('ros')
# set 'fastmode' to True to switch to wireframe mode
# ls /usr/local/share/morse/data/environments/
env = Environment('outdoors', fastmode = True)
env.set_camera_location([10.0, -10.0, 10.0])
env.set_camera_rotation([1.05, 0, 0.78])

