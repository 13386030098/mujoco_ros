#!/usr/local/anaconda3/envs/mujoco/bin/python3.5
from mujoco_py import load_model_from_path, MjSim, MjViewer, cymj
from robot_msgs.msg import ik
import math
import time
import rospy
import os

#while True:
#    sim.data.ctrl[0] =  1.570796
    # sim_state = sim.get_state()
    # print(sim_state)
    # print(sim_state.qpos[1])

    # print("get_joint_qpos")

    # print(cymj.PyMjData.get_joint_qpos(p))

    # print("qvel")
    # print(sim_state.qvel[1])

    # print(sim_state.actuator_length[0])

def callback(data):
#    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    sim_state = sim.get_state()
    print("state")
#    print(sim_state.qpos[1])
    print("true")
    print(data.data[0])
    sim.data.ctrl[0] = data.data[0]
    sim.data.ctrl[1] = data.data[1]
    sim.data.ctrl[2] = data.data[2]
#    sim.data.ctrl[3] = data.data[3]
#    sim.data.ctrl[4] = data.data[4]
#    sim.data.ctrl[5] = data.data[5]

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/ik', ik, callback)
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



























