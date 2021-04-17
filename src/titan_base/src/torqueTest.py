#! /usr/bin/python

from math import sin, cos, pi
import numpy as np
import rospy
import copy
from sensor_msgs.msg import JointState
from titanCodeBase import titan

def waveSimple(titan, runtime):
    set_pos = copy.deepcopy(np.reshape(titan.sentJointPositions, (3, 6)))

    t_start = rospy.Time.now()
    legNum = 0;
    kill = False
    waistStart = set_pos[0][legNum]
    hipStart = set_pos[1][legNum]
    kneeStart = set_pos[2][legNum]
    while not kill and not rospy.is_shutdown():
        t_now = rospy.Time.now()
        t_elapsed = t_now - t_start

        if t_elapsed < runtime:
            print('waving')
            completion = 5*(t_elapsed/runtime)
            set_pos[0][legNum] = waistStart + 0.5*(0.3 * sin(2*pi*completion - (pi/2)) +0.3)
            #set_pos[1][0] = hipStart + 0.3 * sin(2*pi*completion + pi)
            set_pos[2][legNum] = kneeStart - 0.1 * cos(4*pi*completion - (pi)) -0.1
        else:
            #set_pos = copy.deepcopy(center3d)
            kill = True

        titan.publishJointCommands(set_pos.flatten(), np.array([float('nan')]*18), np.array([float('nan')]*18))
        titan.rate.sleep()
    
    
def rockSimple(titan, runtime):
    set_pos = copy.deepcopy(np.reshape(titan.sentJointPositions, (3, 6)))

    t_start = rospy.Time.now()
    legs = [0, 1, 2, 3, 4, 5];
    kill = False
    waistStart = set_pos[0]
    hipStart = set_pos[1]
    kneeStart = set_pos[2]
    while not kill and not rospy.is_shutdown():
        t_now = rospy.Time.now()
        t_elapsed = t_now - t_start

        if t_elapsed < runtime:
            print('rock')
            for leg in range(5):
                completion = 1*(t_elapsed/runtime)
                #print(waistStart[leg])
                set_pos[0][leg] = waistStart[leg] + 0.25*(0.3 * sin(2*pi*completion - (pi/2)) +0.3)
                #set_pos[1][0] = hipStart + 0.3 * sin(2*pi*completion + pi)
                #set_pos[2][legNum] = kneeStart - 0.1 * cos(4*pi*completion - (pi)) -0.1
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
    
    zeros = np.array([
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0
        ], dtype='f')
    legsUp = np.array([
        0, 0, 0, 0, 0, 0,
        -0.6, -0.6, -0.6, -0.6, -0.6, -0.6,
        0, 0, 0, 0, 0, 0
        ])
    legsUpEffort = np.array([
        0, 0, 0, 0, 0, 0,
        -0.6, -0.6, -0.6, -0.6, -0.6, -0.6,
        0, 0, 0, 0, 0, 0
        ])

    
    titan.linJointSpaceTraj( rospy.Duration(5), zeros, legsUp)#Stand
    
    titan.linJointSpaceTraj( rospy.Duration(5), legsUp, legsUp)#pause
    i = 1
    while(i < 100):
        i+= 1
        titan.publishJointCommands(legsUp, np.array([float('nan')]*18), legsUpEffort)
    
    #titan.linJointSpaceTraj( rospy.Duration(5), ThreelegStand, ThreelegStand)
    #titan.linJointSpaceTraj( rospy.Duration(5), ThreelegStand, Stand)
    #titan.linJointSpaceTraj( rospy.Duration(3), Stand, ThreelegStand)#Get into waving stance
    
    #waveSimple(titan, rospy.Duration(8))#Give a wave
    
    #titan.linJointSpaceTraj( rospy.Duration(3), ThreelegStand, Stand)

    #PAUSE
    #titan.linJointSpaceTraj( rospy.Duration(3), Stand, Stand)

    
    
        #Joint positions should be at 'center'
    #titan.linJointSpaceTraj( rospy.Duration(3), titan.jointCenters)#centering Joint Positions
    #titan.linJointSpaceTraj( rospy.Duration(3), titan.jointZero)#Sitting down
    
