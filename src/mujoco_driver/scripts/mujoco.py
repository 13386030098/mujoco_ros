#!/usr/local/anaconda3/envs/mujoco/bin/python3.5
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
from tensorflow.keras.models import Model
from tensorflow.keras.layers import Input, Dense
import matplotlib
import matplotlib.pyplot as plt
import numpy as np

from mujoco_py import load_model_from_path, MjSim, MjViewer, functions, cymj, MjSimState,const
from robot_msgs.msg import ik
import math
import time
import rospy
import os


class kalman:

    load_model = 0
    delta_t = 0.02

    def __init__(self):
        # for position prediction
        self.Q_P = 1e-3 # process variance
        self.R_P = 0.01**2 # estimate of measurement variance, change to see effect
        self.xhat_P = 0
        self.xhatminus_P = 0
        self.Pminus_P = 0
        self.K_P = 0
        self.P_P = 1.0

        # for velocity prediction
        self.Q_V = 1e-3
        self.R_V = 0.01**2
        self.xhat_V = 0
        self.xhatminus_V = 0
        self.Pminus_V = 0
        self.K_V = 0
        self.P_V = 1.0

        # for torque prediction
        self.Q_T = 1e-3
        self.R_T = 0.1**2
        self.xhat_T = 0
        self.xhatminus_T = 0
        self.Pminus_T = 0
        self.K_T = 0
        self.P_T = 1.0

        # for acceleration prediction
        self.A_A = np.mat([[1 , kalman.delta_t, 1/2*kalman.delta_t**2],
                           [0, 1, kalman.delta_t], [0, 0, 1]])
        self.H_A = np.mat([[1, 0, 0], [0, 1, 0]])

        self.P_A = np.mat([[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]])# error covarianc 3*3
        self.Q_A = np.mat([[0.5e-3, 0, 0], [0, 0.5e-3, 0], [0, 0, 2e-1]]) # process variance 3*3
        self.R_A = np.mat([[0.0001**2, 0], [0, 0.0001**2]]) #measurement variance 2*2

        self.xhatminus_A = np.mat(np.zeros((3,1)))
        self.Pminus_A = np.mat(np.zeros((3,3)))

        self.K_A= np.mat(np.zeros((3,2)))
        self.I_A = np.mat(np.identity(3))

        self.xhat_A = np.mat(np.zeros((3, 1)))

        test = np.array(
            [0.7706851, 0.3922136, -0.38806617, 0.11385617, 0.3412635,
             -0.34814566, -0.63279235, -1.1850903, 1.1281418])
        load_model = keras.models.load_model('predict_model.h5')
        pred_train = load_model.predict(test.reshape(1, 9))

    def __del__(self):
        class_name = self.__class__.__name__
        print(class_name, "delete")

    def position_predict(self, z):

        self.xhatminus_P = self.xhat_P
        self.Pminus_P = self.P_P + self.Q_P

        self.K_P = self.Pminus_P / (self.Pminus_P + self.R_P)
        self.xhat_P = self.xhatminus_P + self.K_P * (z - self.xhatminus_P)
        self.P_P = (1 - self.K_P) * self.Pminus_P

        return self.xhat_P


    def velocity_predict(self, z):

        self.xhatminus_V = self.xhat_V
        self.Pminus_V = self.P_V + self.Q_V

        self.K_V = self.Pminus_V / (self.Pminus_V + self.R_V)
        self.xhat_V = self.xhatminus_V + self.K_V * (z - self.xhatminus_V)
        self.P_V = (1 - self.K_V) * self.Pminus_V

        return self.xhat_V

    def torque_predict(self, z):

        self.xhatminus_T = self.xhat_T
        self.Pminus_T = self.P_T + self.Q_T

        self.K_T = self.Pminus_T / (self.Pminus_T + self.R_T)
        self.xhat_T = self.xhatminus_T + self.K_T * (z - self.xhatminus_T)
        self.P_T = (1 - self.K_T) * self.Pminus_T

        return self.xhat_T

    def acceleration_predict(self, z):

        self.xhatminus_A = self.A_A * self.xhat_A
        self.Pminus_A = self.A_A * self.P_A * self.A_A.T + self.Q_A

        self.K_A = self.Pminus_A * self.H_A.T * np.linalg.inv(self.H_A * self.Pminus_A * self.H_A.T + self.R_A)
        self.xhat_A = self.xhatminus_A + self.K_A * (z - self.H_A * self.xhatminus_A)
        self.P_A = (self.I_A - self.K_A * self.H_A) * self.Pminus_A


def callback_omega_1(data):
#    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    sim_state = sim.get_state()
#    print("qpos")
#    print(sim_state.qpos[1])
#    print("qvel")
#    print(sim_state.qvel[1])
#    sim.model.sensor_names
#    print("S_roll2_joint")
    print(sim.data.get_sensor("S_roll2_joint"))
#    print("Sjp_para2_joint")
#    print(sim.data.get_sensor("Sjp_para2_joint"))
#    print("S_instrument_joint")
#    print(sim.data.get_sensor("S_instrument_joint"))

#    print("qvel")
#    print(sim_state.qpos[1])

    sim.data.ctrl[0] = data.data[0]
    sim.data.ctrl[1] = data.data[1]
    sim.data.ctrl[2] = data.data[2]
    # sim.data.ctrl[3] = data.data[3]
    # sim.data.ctrl[4] = data.data[4]
    # sim.data.ctrl[5] = data.data[5]

#def acquisition(event):
#    sim_state = sim.get_state()
#    joint1_pos = sim_state.qpos[1]
#    joint2_pos = sim_state.qpos[2]
#    joint3_pos = sim_state.qpos[3]

#    joint1_vel = sim_state.qvel[1]
#    joint2_vel = sim_state.qvel[2]
#    joint3_vel = sim_state.qvel[3]

#    joint1_tor = sim.data.get_sensor("S_roll2_joint")
#    joint2_tor = sim.data.get_sensor("S_para2_joint")
#    joint3_tor = sim.data.get_sensor("S_instrument_joint")

#    fo=open('data.dat','a+')
#    fo.write(str(joint1_pos) +' '+ str(joint2_pos)+' '+str(joint3_pos)+' '+str(joint1_vel)+' '+str(joint2_vel)+' '+
#    str(joint3_vel)+' '+str(joint1_tor)+' '+str(joint2_tor)+' '+str(joint3_tor)+'\n')
#    fo.close()

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/ik', ik, callback_omega_1)

#    rospy.Timer(rospy.Duration(0.02), acquisition)#250hz

    rate = rospy.Rate(1000)
    t = 0
    while not rospy.is_shutdown():
        t += 1
        sim.step()
        viewer.render()
        if t > 100 and os.getenv('TESTING') is not None:
            break
        rate.sleep()

if __name__ == '__main__':
    model = load_model_from_path("/home/zzz/mujoco_ros/src/mujoco_description/robot.xml")

    sim = MjSim(model)
    viewer = MjViewer(sim)
    try:
        listener()
    except rospy.ROSInterruptException: pass



























