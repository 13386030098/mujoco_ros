#!/usr/local/anaconda3/envs/mujoco/bin/python3.5


from mujoco_py import load_model_from_path, MjSim, MjViewer, functions, cymj, MjSimState,const
from robot_msgs.msg import ik
import math
import time
import rospy
import os
import numpy as np



def listener():
    rospy.init_node('ur5', anonymous=True)

    rate = rospy.Rate(1000)
    t = 0
    while not rospy.is_shutdown():
        sim.data.ctrl[0] = math.cos(t / 10.) * 0.03
#        sim.data.ctrl[1] = math.cos(t / 10.) * 0.03

        t += 1
        sim.step()
        viewer.render()
        if t > 100 and os.getenv('TESTING') is not None:
            break
        rate.sleep()


if __name__ == '__main__':
    model = load_model_from_path("/home/zzz/mujoco_ros/ref/PR2/pr2_arm.xml")
    sim = MjSim(model)
    viewer = MjViewer(sim)
    try:
        listener()
    except rospy.ROSInterruptException: pass
