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

# from mujoco_py import load_model_from_path, MjSim, MjViewer, functions, cymj, MjSimState,const
# from robot_msgs.msg import ik
import math
import time
# import rospy
import os


class kalman:

    load_model = 0
    delta_t = 0.02

    def __init__(self):
        # for position prediction
        # for roll
        self.Q_P_roll = 1e-3 # process variance
        self.R_P_roll = 0.01**2 # estimate of measurement variance, change to see effect
        self.xhat_P_roll = 0
        self.xhatminus_P_roll = 0
        self.Pminus_P_roll = 0
        self.K_P_roll = 0
        self.P_P_roll = 1.0

        # for link
        self.Q_P_link = 1e-3 # process variance
        self.R_P_link = 0.01**2 # estimate of measurement variance, change to see effect
        self.xhat_P_link = 0
        self.xhatminus_P_link = 0
        self.Pminus_P_link = 0
        self.K_P_link = 0
        self.P_P_link = 1.0

        # for slide
        self.Q_P_slide = 1e-3 # process variance
        self.R_P_slide = 0.01**2 # estimate of measurement variance, change to see effect
        self.xhat_P_slide = 0
        self.xhatminus_P_slide = 0
        self.Pminus_P_slide = 0
        self.K_P_slide = 0
        self.P_P_slide = 1.0


        # for velocity prediction
        # for roll
        self.Q_V_roll = 1e-3
        self.R_V_roll = 0.01**2
        self.xhat_V_roll = 0
        self.xhatminus_V_roll = 0
        self.Pminus_V_roll = 0
        self.K_V_roll = 0
        self.P_V_roll = 1.0

        # for link
        self.Q_V_link = 1e-3
        self.R_V_link = 0.01**2
        self.xhat_V_link = 0
        self.xhatminus_V_link = 0
        self.Pminus_V_link = 0
        self.K_V_link = 0
        self.P_V_link = 1.0

        # for slide
        self.Q_V_slide = 1e-3
        self.R_V_slide = 0.01**2
        self.xhat_V_slide = 0
        self.xhatminus_V_slide = 0
        self.Pminus_V_slide = 0
        self.K_V_slide = 0
        self.P_V_slide = 1.0


        # for torque prediction
        # for roll
        self.Q_T_roll = 1e-3
        self.R_T_roll = 0.1**2
        self.xhat_T_roll = 0
        self.xhatminus_T_roll = 0
        self.Pminus_T_roll = 0
        self.K_T_roll = 0
        self.P_T_roll = 1.0

        # for link
        self.Q_T_link = 1e-3
        self.R_T_link = 0.1**2
        self.xhat_T_link = 0
        self.xhatminus_T_link = 0
        self.Pminus_T_link = 0
        self.K_T_link = 0
        self.P_T_link = 1.0

        # for slide
        self.Q_T_slide = 1e-3
        self.R_T_slide = 0.1**2
        self.xhat_T_slide = 0
        self.xhatminus_T_slide = 0
        self.Pminus_T_slide = 0
        self.K_T_slide = 0
        self.P_T_slide = 1.0

        # for acceleration prediction
        # for roll
        self.A_A_roll = np.mat([[1 , kalman.delta_t, 1/2*kalman.delta_t**2],
                           [0, 1, kalman.delta_t], [0, 0, 1]])
        self.H_A_roll = np.mat([[1, 0, 0], [0, 1, 0]])

        self.P_A_roll = np.mat([[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]])# error covarianc 3*3
        self.Q_A_roll = np.mat([[0.5e-3, 0, 0], [0, 0.5e-3, 0], [0, 0, 2e-1]]) # process variance 3*3
        self.R_A_roll = np.mat([[0.0001**2, 0], [0, 0.0001**2]]) #measurement variance 2*2

        self.xhat_A_roll = np.mat(np.zeros((3, 1)))
        self.xhatminus_A_roll = np.mat(np.zeros((3,1)))
        self.Pminus_A_roll = np.mat(np.zeros((3,3)))

        self.K_A_roll = np.mat(np.zeros((3,2)))
        self.I_A_roll = np.mat(np.identity(3))

        # for link
        self.A_A_link = np.mat([[1 , kalman.delta_t, 1/2*kalman.delta_t**2],
                           [0, 1, kalman.delta_t], [0, 0, 1]])
        self.H_A_link = np.mat([[1, 0, 0], [0, 1, 0]])

        self.P_A_link = np.mat([[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]])# error covarianc 3*3
        self.Q_A_link = np.mat([[0.5e-3, 0, 0], [0, 0.5e-3, 0], [0, 0, 2e-1]]) # process variance 3*3
        self.R_A_link = np.mat([[0.0001**2, 0], [0, 0.0001**2]]) #measurement variance 2*2

        self.xhat_A_link = np.mat(np.zeros((3, 1)))
        self.xhatminus_A_link = np.mat(np.zeros((3,1)))
        self.Pminus_A_link = np.mat(np.zeros((3,3)))

        self.K_A_link = np.mat(np.zeros((3,2)))
        self.I_A_link = np.mat(np.identity(3))

        # for slide
        self.A_A_slide = np.mat([[1 , kalman.delta_t, 1/2*kalman.delta_t**2],
                           [0, 1, kalman.delta_t], [0, 0, 1]])
        self.H_A_slide = np.mat([[1, 0, 0], [0, 1, 0]])

        self.P_A_slide = np.mat([[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]])# error covarianc 3*3
        self.Q_A_slide = np.mat([[0.5e-3, 0, 0], [0, 0.5e-3, 0], [0, 0, 2e-1]]) # process variance 3*3
        self.R_A_slide = np.mat([[0.0001**2, 0], [0, 0.0001**2]]) #measurement variance 2*2

        self.xhat_A_slide = np.mat(np.zeros((3, 1)))
        self.xhatminus_A_slide = np.mat(np.zeros((3,1)))
        self.Pminus_A_slide = np.mat(np.zeros((3,3)))

        self.K_A_slide = np.mat(np.zeros((3,2)))
        self.I_A_slide = np.mat(np.identity(3))


        test = np.array(
            [0.7706851, 0.3922136, -0.38806617, 0.11385617, 0.3412635,
             -0.34814566, -0.63279235, -1.1850903, 1.1281418])
        kalman.load_model = keras.models.load_model('/home/zzz/mujoco_ros/src/mujoco_driver/scripts/predict_model.h5')
        pred_train = kalman.load_model.predict(test.reshape(1, 9))
        print(pred_train)

    def __del__(self):
        class_name = self.__class__.__name__
        print(class_name, "delete")

    # for position prediction
    # for roll
    def roll_position_predict(self, z):

        self.xhatminus_P_roll = self.xhat_P_roll
        self.Pminus_P_roll = self.P_P_roll + self.Q_P_roll

        self.K_P_roll = self.Pminus_P_roll / (self.Pminus_P_roll + self.R_P_roll)
        self.xhat_P_roll = self.xhatminus_P_roll + self.K_P_roll * (z - self.xhatminus_P_roll)
        self.P_P_roll = (1 - self.K_P_roll) * self.Pminus_P_roll

        return self.xhat_P_roll

    # for link
    def link_position_predict(self, z):

        self.xhatminus_P_link = self.xhat_P_link
        self.Pminus_P_link = self.P_P_link + self.Q_P_link

        self.K_P_link = self.Pminus_P_link / (self.Pminus_P_link + self.R_P_link)
        self.xhat_P_link = self.xhatminus_P_link + self.K_P_link * (z - self.xhatminus_P_link)
        self.P_P_link = (1 - self.K_P_link) * self.Pminus_P_link

        return self.xhat_P_link

    # for slide
    def slide_position_predict(self, z):

        self.xhatminus_P_slide = self.xhat_P_slide
        self.Pminus_P_slide = self.P_P_slide + self.Q_P_slide

        self.K_P_slide = self.Pminus_P_slide / (self.Pminus_P_slide + self.R_P_slide)
        self.xhat_P_slide = self.xhatminus_P_slide + self.K_P_slide * (z - self.xhatminus_P_slide)
        self.P_P_slide = (1 - self.K_P_slide) * self.Pminus_P_slide

        return self.xhat_P_slide


    # for velocity prediction
    # for roll
    def roll_velocity_predict(self, z):

        self.xhatminus_V_roll = self.xhat_V_roll
        self.Pminus_V_roll = self.P_V_roll + self.Q_V_roll

        self.K_V_roll = self.Pminus_V_roll / (self.Pminus_V_roll + self.R_V_roll)
        self.xhat_V_roll = self.xhatminus_V_roll + self.K_V_roll * (z - self.xhatminus_V_roll)
        self.P_V_roll = (1 - self.K_V_roll) * self.Pminus_V_roll

        return self.xhat_V_roll

    # for link
    def link_velocity_predict(self, z):

        self.xhatminus_V_link = self.xhat_V_link
        self.Pminus_V_link = self.P_V_link + self.Q_V_link

        self.K_V_link = self.Pminus_V_link / (self.Pminus_V_link + self.R_V_link)
        self.xhat_V_link = self.xhatminus_V_link + self.K_V_link * (z - self.xhatminus_V_link)
        self.P_V_link = (1 - self.K_V_link) * self.Pminus_V_link

        return self.xhat_V_link

    # for slide
    def slide_velocity_predict(self, z):

        self.xhatminus_V_slide = self.xhat_V_slide
        self.Pminus_V_slide = self.P_V_slide + self.Q_V_slide

        self.K_V_slide = self.Pminus_V_slide / (self.Pminus_V_slide + self.R_V_slide)
        self.xhat_V_slide = self.xhatminus_V_slide + self.K_V_slide * (z - self.xhatminus_V_slide)
        self.P_V_slide = (1 - self.K_V_slide) * self.Pminus_V_slide

        return self.xhat_V_slide

    # for torque prediction
    # for roll
    def roll_torque_predict(self, z):

        self.xhatminus_T_roll = self.xhat_T_roll
        self.Pminus_T_roll = self.P_T_roll + self.Q_T_roll

        self.K_T_roll = self.Pminus_T_roll / (self.Pminus_T_roll + self.R_T_roll)
        self.xhat_T_roll = self.xhatminus_T_roll + self.K_T_roll * (z - self.xhatminus_T_roll)
        self.P_T_roll = (1 - self.K_T_roll) * self.Pminus_T_roll

        return self.xhat_T_roll

    # for link
    def link_torque_predict(self, z):

        self.xhatminus_T_link = self.xhat_T_link
        self.Pminus_T_link = self.P_T_link + self.Q_T_link

        self.K_T_link = self.Pminus_T_link / (self.Pminus_T_link + self.R_T_link)
        self.xhat_T_link = self.xhatminus_T_link + self.K_T_link * (z - self.xhatminus_T_link)
        self.P_T_link = (1 - self.K_T_link) * self.Pminus_T_link

        return self.xhat_T_link

    # for slide
    def slide_torque_predict(self, z):

        self.xhatminus_T_slide = self.xhat_T_slide
        self.Pminus_T_slide = self.P_T_slide + self.Q_T_slide

        self.K_T_slide = self.Pminus_T_slide / (self.Pminus_T_slide + self.R_T_slide)
        self.xhat_T_slide = self.xhatminus_T_slide + self.K_T_slide * (z - self.xhatminus_T_slide)
        self.P_T_slide = (1 - self.K_T_slide) * self.Pminus_T_slide

        return self.xhat_T_slide

    # for acceleration prediction
    # for roll
    def roll_acceleration_predict(self, z):

        self.xhatminus_A_roll = self.A_A_roll * self.xhat_A_roll
        self.Pminus_A_roll = self.A_A_roll * self.P_A_roll * self.A_A_roll.T + self.Q_A_roll

        self.K_A_roll = self.Pminus_A_roll * self.H_A_roll.T * np.linalg.inv(self.H_A_roll
                                                                             * self.Pminus_A_roll * self.H_A_roll.T + self.R_A_roll)
        self.xhat_A_roll = self.xhatminus_A_roll + self.K_A_roll * (z - self.H_A_roll * self.xhatminus_A_roll)
        self.P_A_roll = (self.I_A_roll - self.K_A_roll * self.H_A_roll) * self.Pminus_A_roll

        return self.xhat_A_roll[2]

    # for link
    def link_acceleration_predict(self, z):

        self.xhatminus_A_link = self.A_A_link * self.xhat_A_link
        self.Pminus_A_link = self.A_A_link * self.P_A_link * self.A_A_link.T + self.Q_A_link

        self.K_A_link = self.Pminus_A_link * self.H_A_link.T * np.linalg.inv(self.H_A_link
                                                                             * self.Pminus_A_link * self.H_A_link.T + self.R_A_link)
        self.xhat_A_link = self.xhatminus_A_link + self.K_A_link * (z - self.H_A_link * self.xhatminus_A_link)
        self.P_A_link = (self.I_A_link - self.K_A_link * self.H_A_link) * self.Pminus_A_link

        return self.xhat_A_link[2]

    # for slide
    def slide_acceleration_predict(self, z):

        self.xhatminus_A_slide = self.A_A_slide * self.xhat_A_slide
        self.Pminus_A_slide = self.A_A_slide * self.P_A_slide * self.A_A_slide.T + self.Q_A_slide

        self.K_A_slide = self.Pminus_A_slide * self.H_A_slide.T * np.linalg.inv(self.H_A_slide
                                                                             * self.Pminus_A_slide * self.H_A_slide.T + self.R_A_slide)
        self.xhat_A_slide = self.xhatminus_A_slide + self.K_A_slide * (z - self.H_A_slide * self.xhatminus_A_slide)
        self.P_A_slide = (self.I_A_slide - self.K_A_slide * self.H_A_slide) * self.Pminus_A_slide

        return self.xhat_A_slide[2]

# data = np.loadtxt('/home/zzz/mujoco_ros/src/mujoco_driver/scripts/data.txt')
#
# col_1 = data[:,0]
# col_2 = data[:,1]
# col_3 = data[:,2]
#
# col_4 = data[:,3]
# col_5 = data[:,4]
# col_6 = data[:,5]
#
# col_7 = data[:,6]
# col_8 = data[:,7]
# col_9 = data[:,8]
#
# data_plot_1_filter = scipy.signal.savgol_filter(col_1, 51, 10)
# data_plot_2_filter = scipy.signal.savgol_filter(col_2, 51, 10)
# data_plot_3_filter = scipy.signal.savgol_filter(col_3, 51, 10)
#
# data_plot_4_filter = scipy.signal.savgol_filter(col_4, 51, 10)
# data_plot_5_filter = scipy.signal.savgol_filter(col_5, 51, 10)
# data_plot_6_filter = scipy.signal.savgol_filter(col_6, 51, 10)
#
# data_plot_7_filter = scipy.signal.savgol_filter(col_7, 51, 10)
# data_plot_8_filter = scipy.signal.savgol_filter(col_8, 51, 10)
# data_plot_9_filter = scipy.signal.savgol_filter(col_9, 51, 10)
#
# yhat_acc_1 = scipy.signal.savgol_filter(data_plot_4_filter, 51, 8, 1, 0.02) # window size 51, polynomial order 3
# yhat_acc_2 = scipy.signal.savgol_filter(data_plot_5_filter, 51, 8, 1, 0.02) # window size 51, polynomial order 3
# yhat_acc_3 = scipy.signal.savgol_filter(data_plot_6_filter, 51, 8, 1, 0.02) # window size 51, polynomial order 3
#
#
# pos_1_filter = np.zeros(col_1.shape[0])
# pos_2_filter = np.zeros(col_1.shape[0])
# pos_3_filter = np.zeros(col_1.shape[0])
#
# vel_1_filter = np.zeros(col_1.shape[0])
# vel_2_filter = np.zeros(col_1.shape[0])
# vel_3_filter = np.zeros(col_1.shape[0])
#
# tor_1_filter = np.zeros(col_1.shape[0])
# tor_2_filter = np.zeros(col_1.shape[0])
# tor_3_filter = np.zeros(col_1.shape[0])
#
# acc_1_filter = np.zeros(col_1.shape[0])
# acc_2_filter = np.zeros(col_1.shape[0])
# acc_3_filter = np.zeros(col_1.shape[0])
#
# n_iter = col_1.shape[0]
#
# object_test = kalman()
#
# for k in range(n_iter):
#     pos_1_filter[k] = object_test.roll_position_predict(col_1[k])
#     pos_2_filter[k] = object_test.link_position_predict(col_2[k])
#     pos_3_filter[k] = object_test.slide_position_predict(col_3[k])
#
#     vel_1_filter[k] = object_test.roll_velocity_predict(col_4[k])
#     vel_2_filter[k] = object_test.link_velocity_predict(col_5[k])
#     vel_3_filter[k] = object_test.slide_velocity_predict(col_6[k])
#
#     tor_1_filter[k] = object_test.roll_torque_predict(col_7[k])
#     tor_2_filter[k] = object_test.link_torque_predict(col_8[k])
#     tor_3_filter[k] = object_test.slide_torque_predict(col_9[k])
#
# pos_vel_1 = np.concatenate((pos_1_filter.reshape(1, -1), vel_1_filter.reshape(1, -1)), 0)
# pos_vel_2 = np.concatenate((pos_2_filter.reshape(1, -1), vel_2_filter.reshape(1, -1)), 0)
# pos_vel_3 = np.concatenate((pos_3_filter.reshape(1, -1), vel_3_filter.reshape(1, -1)), 0)
#
# pos_vel_1 = np.mat(pos_vel_1)
# pos_vel_2 = np.mat(pos_vel_2)
# pos_vel_3 = np.mat(pos_vel_3)
#
#
# for k in range(n_iter):
#     acc_1_filter[k] = object_test.roll_acceleration_predict(pos_vel_1[:,k])
#     acc_2_filter[k] = object_test.link_acceleration_predict(pos_vel_2[:,k])
#     acc_3_filter[k] = object_test.slide_acceleration_predict(pos_vel_3[:,k])
#
# # plt.figure()
# # l1,= plt.plot(data_plot_7_filter[:2000])
# # l2,= plt.plot(tor_1_filter[:2000])
# # plt.legend(handles=[l1, l2], labels=['S-G', 'kalman_predict'],loc='best')
# # plt.show()
#
# pos_1_filter = pos_1_filter.reshape(-1, 1)
# pos_2_filter = pos_2_filter.reshape(-1, 1)
# pos_3_filter = pos_3_filter.reshape(-1, 1)
#
# vel_1_filter = vel_1_filter.reshape(-1, 1)
# vel_2_filter = vel_2_filter.reshape(-1, 1)
# vel_3_filter = vel_3_filter.reshape(-1, 1)
#
# acc_1_filter = acc_1_filter.reshape(-1, 1)
# acc_2_filter = acc_2_filter.reshape(-1, 1)
# acc_3_filter = acc_3_filter.reshape(-1, 1)
#
# input_data  = np.concatenate((pos_1_filter, pos_2_filter, pos_3_filter, vel_1_filter, vel_2_filter, vel_3_filter,
#                               acc_1_filter, acc_2_filter, acc_3_filter), 1)
#
#
# pre_train = kalman.load_model.predict(input_data)
#
#
# plt.figure()
# l1,= plt.plot(tor_1_filter[58000:])
# l2,= plt.plot(pre_train[58000:,0])
# plt.legend(handles=[l1, l2], labels=['kalman_filter', 'kalman_predict'],loc='best')
#
# plt.figure()
# l1,= plt.plot(tor_2_filter[58000:])
# l2,= plt.plot(pre_train[58000:,1])
# plt.legend(handles=[l1, l2], labels=['kalman_filter', 'kalman_predict'],loc='best')
#
# plt.figure()
# l1,= plt.plot(tor_3_filter[58000:])
# l2,= plt.plot(pre_train[58000:,2])
# plt.legend(handles=[l1, l2], labels=['kalman_filter', 'kalman_predict'],loc='best')
# #
# plt.show()





def callback_omega_1(data):

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



























