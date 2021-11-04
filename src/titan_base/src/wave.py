#! /usr/bin/python

from math import sin, pi
import numpy as np
import rospy
import copy
from sensor_msgs.msg import JointState
from titanCodeBase import titan

def waveSimple(titan, runtime):
    #Simple Wave Gait based off of 'eigenbot_hexapod_test.py'
    period = 10.0

    #Difference in Phases between Modules
    phase = [
    [0, pi, 0, 0, pi, 0],
    [-pi/2, pi/2, -pi/2, pi/2, -pi/2, pi/2],
    [-pi/2, pi/2, -pi/2, pi/2, -pi/2, pi/2]
    ]

    #Amplitude of joint motion
    amp = [[pi/48]*6, [pi/48]*6, [pi/48]*6]

    center3d = titan.jointCenters.reshape(3,6)
    
    '''set_pos = np.array([
    [0, 0, 0, 0, 0, 0],
    [-0.8, -0.8, -0.8, -0.8, -0.8, -0.8],
    [0, 0, 0, 0, 0, 0]
    ], dtype='f')
    '''
    set_pos = copy.deepcopy(np.reshape(titan.realCenters, (3, 6)))
    realCenters = copy.deepcopy(np.reshape(titan.realCenters, (3,6)))
    t_start = rospy.Time.now()

    kill = False

    while not kill and not rospy.is_shutdown():
        print('wave')
        t_now = rospy.Time.now()
        t_elapsed = t_now - t_start

        if t_elapsed < runtime:
            for row in range(len(set_pos)):
                
                #for col in range(len(set_pos[row])):
                col = 0
                
                settingValue = realCenters[row, col] + amp[row][col]*sin(((2.0*pi*t_elapsed.to_sec())/period)+phase[row][col])
                #np.put(set_pos, [row][col], settingValue)
                set_pos[row][col] = copy.deepcopy(settingValue)
                print('settingValue')
                print(settingValue)
                print(set_pos)
                    #:set_pos[2, col] = -( -2 + center3d[2][col] + amp[2][col]*sin(((2.0*pi*t_elapsed.to_sec())/period)+phase[2][col]))
                #print('SENT')
                #print(set_pos[2, col])
        else:
            #set_pos = copy.deepcopy(center3d)
            kill = True

        titan.publishJointCommands(set_pos.flatten(), np.array([float('nan')]*18), np.array([float('nan')]*18))
        titan.rate.sleep()
    
    
    


#def waveTrue():

    #rospy.Subscriber('/titan/controller', JointState, titan.command_cb)
    
if __name__ == "__main__":
    titan = titan("bot_params.txt")
    #print('linjointspace')
    #titan.linJointSpaceTraj( rospy.Duration(3000), titan.jointCenters)#Standing up
        #Joint positions should be at 'center'
    print('wave')
    zeros = np.array([
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0
        ], dtype='f')
    titan.linJointSpaceTraj( rospy.Duration(20), zeros, zeros)
    #waveSimple(titan, rospy.Duration(10))
    #titan.linJointSpaceTraj( rospy.Duration(3), titan.sentJointPositions, zeros)    
        #Joint positions should be at 'center'
    #titan.linJointSpaceTraj( rospy.Duration(3), titan.jointCenters)#centering Joint Positions
    #titan.linJointSpaceTraj( rospy.Duration(3), titan.jointZero)#Sitting down
    
