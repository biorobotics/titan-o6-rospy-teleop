#! /usr/bin/python

import time
import matplotlib.pyplot as plt
import rospy
from sensor_msgs.msg import JointState

def callback(data):
    for pl in range(0, 18):
        ax.scatter(time.time()-timeStart, msg.position[pl], color=(float(pl%5)/4, float(pl%11+5)/17, float(pl%6+2)/8), marker='o')
        plt.draw()
        plt.pause(0.005)

def jointPositionGraph():
    global timeStart = time.time()
    fig, ax = plt.subplots()
    rospy.init_node('jointPositionGraph')
    rospy.Subscriber("/titan/joint_cmd", JointState, callback)
    rospy.spin()

if __name__ == "__main__":
    jointPositionGraph()