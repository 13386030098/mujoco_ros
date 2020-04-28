#!/usr/local/anaconda3/envs/mujoco/bin/python3.5
from mujoco_py import load_model_from_path, MjSim, MjViewer, functions, cymj, MjSimState,const
from robot_msgs.msg import ik
import math
import time
import rospy
import os
import numpy as np

def callback_omega_1(data):
#    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    sim_state = sim.get_state()
#    print("qpos")
#    print(sim_state.qpos[1])
#    print("qvel")
#    print(sim_state.qvel[1])
#    sim.model.sensor_names
#    print("S_roll2_joint")
#    print(sim.data.get_sensor("S_roll2_joint"))
#    print("Sjp_para2_joint")
#    print(sim.data.get_sensor("Sjp_para2_joint"))
#    print("S_instrument_joint")
#    print(sim.data.get_sensor("S_instrument_joint"))

#    print("qvel")
#    print(sim_state.qpos[1])

    sim.data.ctrl[0] = data.data[0]
    sim.data.ctrl[1] = data.data[1]
    sim.data.ctrl[2] = data.data[2]
    sim.data.ctrl[3] = data.data[3]
    sim.data.ctrl[4] = data.data[4]
    sim.data.ctrl[5] = data.data[5]

def callback_omega_2(data):

    sim.data.ctrl[6]  = data.data[0]
    sim.data.ctrl[7]  = data.data[1]
    sim.data.ctrl[8]  = data.data[2]
    sim.data.ctrl[9]  = data.data[3]
    sim.data.ctrl[10] = data.data[4]
    sim.data.ctrl[11] = data.data[5]

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
    rospy.Subscriber('omega1/ik', ik, callback_omega_1)
    rospy.Subscriber('omega2/ik', ik, callback_omega_2)

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
#    model = load_model_from_path("/home/zzz/mujoco_ros/src/mujoco_description/robot.xml")
    model = load_model_from_path("/home/zzz/mujoco_ros/src/mujoco_description/robot_dual.xml")

    sim = MjSim(model)
    viewer = MjViewer(sim)
    try:
        listener()
    except rospy.ROSInterruptException: pass



























