#! /usr/bin/python
from sensor_msgs.msg import JointState
import rospy
import copy
import numpy as np

def collapse(array):
    retval = []
    for i in array:
        retval = retval + i
    return retval

if __name__ == "__main__":
    print('start')
    jointAddresses = [
            ['16', '10', '0E', '01', '0D', '14'],
            ['05', '09', '11', '0F', '08', '04'], 
            ['13', '0A', '07', '02', '03', '0C']
        ]
    set_pos = np.array([
    [0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0]
    ])
    increment = np.array([
    [0.1, 0, 0, 0, 0, 0],
    [0.1, 0, 0, 0, 0, 0],
    [0.1, 0, 0, 0, 0, 0]
    ])

    increment
    rospy.init_node('scripted_motion')
    rate = rospy.Rate(20)
    jointCommandPublisher = rospy.Publisher('titan/joint_cmd', JointState, queue_size=1)

    while not rospy.is_shutdown():
        js = JointState()
        print(set_pos)   
        js.name = collapse(jointAddresses)
        js.position = set_pos.flatten()
        js.velocity = [float('nan')]*18
        js.effort = [float('nan')]*18

        jointCommandPublisher.publish(js)
        print('About to Add')
        print(set_pos)
        print('mid')
        set_pos = np.add(set_pos, increment)
        print(set_pos)
        print('Added')
        rate.sleep()


