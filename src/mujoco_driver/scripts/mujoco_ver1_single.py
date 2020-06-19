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
        if t > 100 and os.getenv('TESTING') is not None:
            break
        rate.sleep()
        # stop = time.time()
        # print(str((stop - start) * 1000) + "ms")

if __name__ == '__main__':
    model = load_model_from_path("/home/zzz/mujoco_ros/src/robot_description/robot_ver1_mujoco.xml")

    sim = MjSim(model)
    viewer = MjViewer(sim)
    try:
        listener()
    except rospy.ROSInterruptException: pass



























