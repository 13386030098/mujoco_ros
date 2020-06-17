#!/usr/local/anaconda3/envs/mujoco/bin/python3.5
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
from tensorflow.keras.models import Model
from tensorflow.keras.layers import Input, Dense
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import scipy.signal
import functools

from mujoco_py import load_model_from_path, MjSim, MjViewer, functions, cymj, MjSimState,const
from robot_msgs.msg import ik
import math
import time
import rospy
import os


def callback_omega_1(data):

    sim.data.ctrl[1] = data.data[0]
    sim.data.ctrl[2] = data.data[1]
    sim.data.ctrl[3] = data.data[2]
    sim_state = sim.get_state()
    print(sim_state.qpos[0])

    sim.data.ctrl[4] = data.data[3]
    sim.data.ctrl[5] = data.data[4]

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/ik', ik, callback_omega_1)

    rate = rospy.Rate(1000)
    t = 0
    while not rospy.is_shutdown():
        # start = time.time()
        t += 1
        sim.step()
        viewer.render()
        sim_state = sim.get_state()

#        print("qpos")
#        print(sim_state.qpos[1])
#        print("get_sensor")
#        print(sim.data.get_sensor("roll2_joint"))
        if t > 100 and os.getenv('TESTING') is not None:
            break
        rate.sleep()
        # stop = time.time()
        # print(str((stop - start) * 1000) + "ms")

if __name__ == '__main__':
    model = load_model_from_path("/home/zzz/mujoco_ros/src/mujoco_description/robot_ver2.xml")
#    model = load_model_from_path("/home/zzz/mujoco_ros/src/mujoco_description/robot.xml")

    sim = MjSim(model)
    viewer = MjViewer(sim)
    try:
        listener()
    except rospy.ROSInterruptException: pass



























