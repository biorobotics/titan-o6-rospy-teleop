#! /usr/bin/python
#Simple Tripod Gait Implementation
#Be careful when switching between statesw
    #any initialization time period needed

#Behavior trees?

#ONly class variables 

# use rospy.time instead of time.time
# Make this a state Machine
#send ROS messages, create something to go to the simulator
#lucidchart

from math import sin, cos, acos, atan, atan2, sqrt, isnan, pi
import numpy as np
import rospy
import copy
from sensor_msgs.msg import JointState
from std_msgs.msg import *


class titan(object):
    def __init__(self, filename):
        self.STATE = "MOVE"
        rospy.init_node('titanObject')
        self.jointCommandPublisher = rospy.Publisher('/titan/joint/cmd', JointState, queue_size=1)
        self.jointCommandPublisherSIM = rospy.Publisher('/titan/joint_cmd', JointState, queue_size=1)
        #rospy.Subscriber('/titan/control', std_msgs.msg.Float32MultiArray, self.command_cb)
        self.rate = rospy.Rate(30)
        self.sentJointPositions = np.array([
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0
        ], dtype='f')
        
        self.jointAddresses = np.array([
            '16', '14', '11', '04', '07', '0C',
            '10', '0D', '09', '08', '0A', '0B', 
            '0E', '01', '05', '0F', '13', '02'
        ])
        self.jointCenters = np.array([
        0, 0, 0, 0, 0, 0,
        -pi/4, -pi/4, -pi/4, -pi/4, -pi/4, -pi/4,
        3*pi/4, 3*pi/4, 3*pi/4, 3*pi/4, 3*pi/4, 3*pi/4
        ], dtype='f')
        self.realCenters = np.array([
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        ])
        self.jointZero = np.array([
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0
        ], dtype='f')
        """ 
        f = open(filename, "r")
        f_lines = f.readlines()
        
        for i in range(0, len(f_lines), 2):
            if(f_lines[i] == "l1\n"):
                self.l1 = float(f_lines[i+1])
            elif(f_lines[i] == "l2\n"):
                self.l2 = float(f_lines[i+1])
            elif(f_lines[i] == "ang_offs\n"):
                str_offs = f_lines[i+1].split(',')
                str_offs[2] = str_offs[2][0:-1]
                self.ang_offs = np.array(str_offs).astype(np.float)
            elif(f_lines[i] == "all_l\n"):
                all_l = f_lines[i+1].split(',')
                all_l[len(all_l)-1] = all_l[len(all_l)-1][0:-1]
        
        self.center_rad = 0.2 #distance from center of body to each leg base
        self.ref_pos = np.zeros(18)
        self.pos = np.zeros(18)
        self.i_con = np.zeros([3, 6]) #xyz pos of each ee
        self.update_init()"""




#   have this be a state
#   Standstill state?



    def command_cb(self, data):
        #Interpretds data from command node
        print('commandcb')
    def fb_cb(self, msg):
        #interperets feedback daata
        for i in range(0, len(msg.name)):
            this_name = msg.name[i];
        for j in range(0, 18):
            if(this_name == self.all_l[j] and not isnan(msg.position[i])):
                self.ref_pos[j] = msg.position[i]
    """ 
    def t2xyz(self, t1, t2, t3): #FK 
        xyz = [0, 0, 0]
        t1 = t1+self.ang_offs[0]
        t2 = t2+self.ang_offs[1]
        t3 = t3+self.ang_offs[2]

        reach = self.l1*cos(t2) + self.l2*cos(t2+t1)
        xyz[0] = reach*cos(t3)
        xyz[1] = reach*sin(t3)
        xyz[2] = self.l1*sin(t2) + self.l2*sin(t2+t1)
        return xyz

    def xyz2t(self, x, y, z, leg): #IK
        t1=atan(y/x)

        w = sqrt(x*x+y*y)

        l1 = self.l1
        l2 = self.l2
        t3 = -1*acos((z*z+w*w-l1*l1-l2*l2)/(2*l1*l2))
        t2 = -1*atan2(z, w) + atan2(l2*sin(t3), l1+l2*cos(t3))

        t_arr = [t3, -1*t2, t1]

        for i in range(0, 3):
            t_arr[i] = t_arr[i] - self.ang_offs[i]

        t_ind = 0
        l = (leg-1)*3

        while l < (leg-1)*3+3: #Checks validity of IK solution
            if(abs(t_arr[t_ind] - self.ref_pos[l]) < 0.2 or OUTPUT == 'SIM'):
                #if(OUTPUT == 'PLT'):
                #    self.ref_pos[l] = t_arr[t_ind] #Faking perfect control in simulation case
                self.pos[l] = t_arr[t_ind]
            elif(abs(t_arr[t_ind] - self.ref_pos[l]) > 5.5):
                offset = 2*pi*round((self.ref_pos[l] - t_arr[t_ind])/(2*pi))
                self.pos[l] = t_arr[t_ind] + offset
            else:
                print('invalid command to joint: ' + all_l[l])
            l = l + 1
            t_ind = t_ind + 1
    def smoothSlide(self, init, fin, prog): #smooth (velocity at ends are 0) interpolation from init to fin
        #NEEDS: ramp_por k2, m2,
        x = 0

        if(prog < self.ramp_por):
            x = init + self.k2*(fin-init)*prog*prog
        elif(prog < (1-self.ramp_por)):
            x = init + (fin-init)*(self.m2*prog+self.k2*self.ramp_por*self.ramp_por-self.m2*self.ramp_por)
        else:
            x = fin- self.k2*(fin-init)*(1-prog)*(1-prog)
        return x

    def stepAngles(self, init_pos, last_pos, leg, up, prog): #given a initial and final pos, returns joint values for interpolated points inbetween
        #NEEDS: stand_h stand_rad
        
        p_sym = abs(0.5-prog)
        z = -1*self.stand_h + init_pos[2]
        x = self.stand_rad
        y = 0

        if(up):
            if(prog < 0.5):
                z = self.smoothSlide(-1*self.stand_h+init_pos[2], -1*self.stand_h+self.step_h, prog*2)
            else:
                z = self.smoothSlide(-1*self.stand_h+self.step_h, -1*self.stand_h+last_pos[2], 2*(prog-0.5))

        x = self.smoothSlide(self.stand_rad+init_pos[0], self.stand_rad+last_pos[0], prog)
        y = self.smoothSlide(init_pos[1], last_pos[1], prog)

        self.xyz2t(x, y, z, leg)

        return([x-self.stand_rad, y, z+self.stand_h])
    
    def update_init(self): #Updating EE positions
        for l in range(0, 6):
            self.i_con[:, l] = self.t2xyz(self.ref_pos[3*l], self.ref_pos[3*l+1], self.ref_pos[3*l+2])

    def t2inter(self, t2, t3): #FK for knee joint
        xyz = [0, 0, 0]
        t2 = t2+self.ang_offs[1]
        t3 = t3+self.ang_offs[2]

        reach = self.l1*cos(t2)
        xyz[0] = reach*cos(t3)
        xyz[1] = reach*sin(t3)
        xyz[2] = self.l1*sin(t2)
        return xyz

    

    def leg2cb(self, l_xyz, leg): #XYZ in in leg space (origin at leg base) to XYZ in center body frame
        leg_angs = self.getleg_ang(leg)
        yaw = leg_angs[0]
        plane_vec = np.array([[l_xyz[0]+self.center_rad, l_xyz[1]]]).T
        rot_mat = np.array([[cos(yaw), -1*sin(yaw)], [sin(yaw), cos(yaw)]])
        new_plan = np.matmul(rot_mat, plane_vec)
    
        merged_res = [new_plan[0][0], new_plan[1][0], l_xyz[2]]
        return merged_res
    def getleg_ee(self, leg): #leg is zero indexed
        ee_pos = self.t2xyz(self.ref_pos[leg*3], self.ref_pos[leg*3+1], self.ref_pos[leg*3+2])
        return ee_pos

    def getleg_knee(self, leg):
        knee_pos = self.t2inter(self.ref_pos[leg*3+1], self.ref_pos[leg*3+2])
        return knee_pos

    def getleg_ang(self, leg):
        ang_mat = [0, 0, 0]
        ang_mat[0] = pi/6+leg*pi/3 + self.ang_offs[2] - self.ref_pos[leg*3+2] #yaw
        ang_mat[1] = self.ang_offs[1] + self.ref_pos[leg*3+1] #roll intermed
        ang_mat[2] = ang_mat[1] + self.ang_offs[0] + self.ref_pos[leg*3] #roll fin

        return ang_mat

    def check_coll(self, leg):
        coll_threshold = 0.05
        if(self.ref_pos[leg*3+1] - self.pos[leg*3+1] > coll_threshold):
            return True
        else:
            return False """

    def linJointSpaceTraj(self, runtime, start, goal):
        #Moves joints between currentPosition and goal pos in linear joint space


        set_pos = np.array([
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0
        ], dtype='f')
        

        t_start = rospy.Time.now()

        kill = False

        while not kill:#not rospy.is_shutdown()
            t_now = rospy.Time.now()
            t_elapsed = t_now - t_start
            completionRatio = t_elapsed.to_sec()/runtime.to_sec() #A number from 0 to 1 that describes how far the stand up process is 
            if t_elapsed < runtime:
                sentPos = copy.deepcopy(self.sentJointPositions)
                centers = copy.deepcopy(self.jointCenters)
                #Might need to use np for this
                set_pos = start + (completionRatio*(np.subtract(goal, start)))

            else:
                kill = True
            print(set_pos.flatten())
            self.publishJointCommands(set_pos.flatten(), np.array([float('nan')]*18), np.array([float('nan')]*18))
            self.rate.sleep()
        return

    def publishJointCommands(self, position, velocity, effort):
        #if not self.Mstop:
            js = JointState()
            js.name = self.jointAddresses.flatten()
            js.position = position.flatten()
            js.velocity = velocity.flatten()
            js.effort = effort.flatten()
            self.jointCommandPublisher.publish(js)
            self.jointCommandPublisherSIM.publish(js)

            #print('PUBLISHED')
            #print(position)
            self.sentJointPositions = copy.deepcopy(position)
            return
