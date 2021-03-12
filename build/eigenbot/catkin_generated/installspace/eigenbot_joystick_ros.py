
'''
TODO:
make version without pybullet dependency
consider how to reduce delay and set rates
check what happens when order of modules is random, ports for limbs might need to be sorted

# initialize joystick
# assemble urdf, drop in


note to self for later: if no 90deg present, add 45 deg offsets to the final two joints
            if module_types[child_index] == '90deg':
                limb_has_90deg = True

                ...

    const_offsets = np.array([[-1,0,1,-1,0,1], 
        [0,0,0,0,0,0], [0,0,0,0,0,0]])*np.pi/4
    for i in range(6):
        if limb_has_90deg[i]:
            const_offsets[1:,i] += np.pi/4


'''

import vrjoystick

from description_assembler_3 import description_assemble 
import numpy as np
import pybullet as p
import pybullet_data
import time
import itertools
import os
pi= np.pi
cwd = os.path.dirname(os.path.realpath(__file__))


def wrap_to_pi(angle):
    return np.remainder(angle + np.pi,  np.pi*2) - np.pi

def np2str(input, precision=2):
    return np.array2string(input,precision=2)

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import rosnode

RUN_SIM_GUI = False # if true, shows gui and steps sim, else does not
# RUN_SIM_GUI = True # if true, shows gui and steps sim, else does not
SEND_COMMANDS = False # if true, publishes ROS commands
SEND_COMMANDS = True # if true, publishes ROS commands

# car test
# robot_name = 'car'
module_types_test = ['Eigenbody',  'Wheel_module', 
 'Wheel_module',  'Wheel_module',  'Wheel_module',
  ]
graph_edges_test = [
[0, 1, 0, 0], # from node, to node, port, mount 
[0, 2, 2, 0 ],
[0, 3, 3, 4 ],
[0, 4, 5, 4 ]
]


module_serials_test = [str(foo) for foo in range(len(module_types_test))]


def joint_state_talker(module_types=module_types_test,
                       graph_edges = graph_edges_test,
                       module_serials=module_serials_test):

    joy = vrjoystick.init_joystick()


    description_assemble(module_types, graph_edges, module_serials)

    # Go through the modules and identify which modules are involved in 
    # legs vs wheels and what their orientations is
    print('Finding limb types...')
    n_chassis_ports = 6
    limb_types = [None]*n_chassis_ports
    wheel_joint_ids = [None]*n_chassis_ports
    leg_joint_ids = [list() for i in range(n_chassis_ports)]
    leg_joint_orns = [None]*n_chassis_ports
    # for each limb 1-6, keep an open set of modules to look at the ports.
    open_set = [list() for i in range(n_chassis_ports)] 
    limb_info = []
    # go through edges to find out which modules are attached to the body
    for edge in graph_edges:
        if module_types[edge[0]] == 'Eigenbody' and edge[2]<6: # for legs on ports 0-5
            open_set[edge[2]].append(edge[1:4]) # a child to open

        # if module_types[edge[0]] == 'Eigenbody' and 1<=edge[2]<=6: # for legs on ports 1-6
            # open_set[edge[2]-1].append(edge[1:4]) # a child to open, ports index 1-6
            # add to open set child module and its port and mount 

    for chassis_port in range(n_chassis_ports):
        joints_ids_branch = []
        joint_orn_branch = []
        # print('Port ' + str(chassis_port))
        while len(open_set[chassis_port])>0:
            # pop an edge to look at
            child_index, child_port, child_mount = open_set[chassis_port].pop()
            child_type = module_types[child_index]
            
            # If the module is a wheel, label this limb as a wheel
            if child_type == 'Wheel_module':
                # print('Found a wheel')
                limb_types[chassis_port] = 'wheel_limb'
                wheel_joint_ids[chassis_port] = module_serials[child_index]
                open_set[chassis_port] = []
            
            # If the module is a leg, label this limb as a leg
            elif child_type == 'Foot_module':
                # print('Found a foot')
                limb_types[chassis_port] = 'foot_limb'
                #   track which joints are in the leg
                leg_joint_ids[chassis_port] = joints_ids_branch
                leg_joint_orns[chassis_port] = joint_orn_branch
                open_set[chassis_port] = []

            # For modules that are not terminal, look for their children
            else:
                # keep special care for bendys, 
                if child_type == 'Bendy_module':
                    # keep a list of their IDs and orientations for later use
                    joints_ids_branch.append(module_serials[child_index])
                    joint_orn_branch.append(child_mount)

                # go through edges to identify the children
                for edge in graph_edges:
                    if edge[0] == child_index:
                        open_set[chassis_port].append(edge[1:4])
                        
        
    print('limb_types ' + str(limb_types)) 
    print('wheel_joint_ids ' + str( wheel_joint_ids))
    print('leg_joint_ids ' + str( leg_joint_ids))   
    print('leg_joint_orns ' + str( leg_joint_orns))    

    # keep track of which orientation the joint axes are in
    # This appears to work properly for the first and possibly second joint on each leg,
    # but fails for the last joint. Not sure why.
    leg_joint_orns_cumulative = [None]*n_chassis_ports
    leg_joint_multiplier = [None]*n_chassis_ports
    for i in range(6):
     # the orientation accumulates with each module added. Find the total orn change

        if leg_joint_orns[i] is not None:
            leg_joint_orns_cumulative[i] = wrap_to_pi(np.cumsum(leg_joint_orns[i])*np.pi/4)
            leg_joint_multiplier[i] = []

            # first joint, if it exists
            if len(leg_joint_orns_cumulative[i])>0:
                if  -(np.pi/2 - 0.1) < leg_joint_orns_cumulative[i][0] < (np.pi/2 - 0.1):
                    leg_joint_multiplier[i].append(1)
                elif np.abs(leg_joint_orns_cumulative[i][0]) > (np.pi/2 + 0.1):
                    leg_joint_multiplier[i].append(-1)
                else:
                    leg_joint_multiplier[i].append(0)

    #         # second joint, if it exists
            if len(leg_joint_orns_cumulative[i])>1:
                if 0.1 < leg_joint_orns_cumulative[i][1] < (np.pi - 0.1):
                    leg_joint_multiplier[i].append(-1)
                elif (-np.pi + 0.1) < leg_joint_orns_cumulative[i][1] < -0.1:
                    leg_joint_multiplier[i].append(1)
                else:
                    leg_joint_multiplier[i].append(0)

    #         # third joint, if it exists
            if len(leg_joint_orns_cumulative[i])>2:
                if np.abs(leg_joint_orns_cumulative[i][2]) > (np.pi-0.1):
                    leg_joint_multiplier[i].append(-1)
                elif np.abs(leg_joint_orns_cumulative[i][2]) < 0.1:
                    leg_joint_multiplier[i].append(1)
                else:
                    leg_joint_multiplier[i].append(0)

    
                          
    print('leg_joint_orns_cumulative')
    print(str(leg_joint_orns_cumulative))        
    print('leg_joint_multiplier ')
    print(str(leg_joint_multiplier))        



    cwd = os.path.dirname(os.path.realpath(__file__))

    # setup world
    if RUN_SIM_GUI:
        physicsClient = p.connect(p.GUI)# p.DIRECT for non-graphical version
    else:
        physicsClient = p.connect(p.DIRECT)# p.DIRECT for non-graphical version
    p.configureDebugVisualizer(p.COV_ENABLE_GUI,0,physicsClientId=physicsClient)
    p.resetDebugVisualizerCamera(1,0,-65,[0,0,0],physicsClientId=physicsClient) # I like this view

    p.resetSimulation(physicsClient) # remove all objects from the world and reset the world to initial conditions. (not needed here but kept for example)
    p.setGravity(0,0,-9.81,physicsClientId=physicsClient)
    planeId = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane100.urdf"),physicsClientId=physicsClient)



    # add robot.
    # TODO: make this machine independent
    urdf_name = '/home/cobracommander/catkin_ws/src/eigenbot/urdf/autoXACRO.urdf'
    parent_folder = os.path.dirname(cwd)
    urdf_name = os.path.join(parent_folder, 'urdf/autoXACRO.urdf'),

    startOrientation = p.getQuaternionFromEuler([0,-np.pi/2,0])

    # Use pybullet to get the number of joints, limits, and other info.
    # Not ideal-- this could probably be replaced by xml parsing.
    print('loading urdf...')
    # robotID = p.loadURDF(urdf_name,
    #     basePosition=[0,0,0.5], baseOrientation=startOrientation,
    #                                flags= (p.URDF_MAINTAIN_LINK_ORDER | 
    #                                p.URDF_USE_SELF_COLLISION | 
    #                                p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES), # "The following flags can be combined using a bitwise  OR, |"
    #                                physicsClientId=physicsClient)


    # maintain link order seems to break it... order of modules might not be the same
    robotID = p.loadURDF(urdf_name,
        basePosition=[0,0,0.5], baseOrientation=startOrientation,
                                   physicsClientId=physicsClient)
    num_joints_total = p.getNumJoints(robotID,physicsClientId=physicsClient)
    moving_joint_inds = []
    joint_names = []
    moving_joint_names = []
    moving_joint_types = []
    moving_joint_limits = []
    moving_joint_centers = []
    moving_joint_max_torques = []
    moving_joint_max_velocities = []
    # getJointInfo 8 9 are jointLowerLimit and jointUpperLimit 

    # collect joint info, center, maxes, min, etc
    for j_ind in range(num_joints_total):
        j_info = p.getJointInfo(robotID, j_ind, physicsClientId=physicsClient)
        joint_names.append(j_info[1])
        if j_info[2] != (p.JOINT_FIXED):
            moving_joint_inds.append(j_ind)
            j_limits = [j_info[8], j_info[9]]
            j_center = (j_info[8] + j_info[9])/2
            if j_limits[1]<=j_limits[0]:
                j_limits = [-np.inf, np.inf]
                j_center = 0
            moving_joint_limits.append(j_limits)
            moving_joint_centers.append(j_center)

            moving_joint_names.append(j_info[1])
            moving_joint_max_torques.append(j_info[10])
            moving_joint_max_velocities.append(j_info[11])
            moving_joint_types.append(j_info[2])
    num_joints = len(moving_joint_inds)
    moving_joint_max_velocities = np.array(moving_joint_max_velocities)
    print('Moving joints:')
    print('moving_joint_names')
    print(moving_joint_names)
    print('moving_joint_limits') # NOTE: if limits[1]<limits[0] then pybullet ignores the limits.
    print(moving_joint_limits) # NOTE: if limits[1]<limits[0] then pybullet ignores the limits.
    print('moving_joint_centers')
    print(moving_joint_centers)
    print('moving_joint_max_torques')
    print(moving_joint_max_torques)
    print('moving_joint_max_velocities')
    print(moving_joint_max_velocities)
    print('moving_joint_types')
    print(moving_joint_types)

    lower_limits = []
    upper_limits = []
    for m in moving_joint_limits:
        lower_limits.append(m[0])
        upper_limits.append(m[1])

    # collect joint serial numbers for use in ROS joint commands
    joint_serials =[]
    extra_joint_inds = []
    for i_mj_bytes in range(num_joints):
        mj_bytes = moving_joint_names[i_mj_bytes]
        mj = mj_bytes.decode('utf-8')
        # print(mj)
        serial_index = mj.find('_S')
        joint_serial = mj[serial_index+2:]
        joint_serials.append(joint_serial)

         # get joint names of those joints not in the legs or wheels
        if (joint_serial not in wheel_joint_ids) and (joint_serial not in 
            itertools.chain(*leg_joint_ids)):
            extra_joint_inds.append(i_mj_bytes)

    print('joint serials: ' + str(joint_serials) )
    print('Indexes not in legs or wheels:' + str(extra_joint_inds))
    print('Names not in legs or wheels:' + str(
        [moving_joint_names[i_extra] for i_extra in extra_joint_inds] ))
    extra_joint_inds = np.array(extra_joint_inds)

    if RUN_SIM_GUI:
        # set sim to a centered initial joint angle
        for i in range(num_joints):
            center = moving_joint_centers[i]
            jind = moving_joint_inds[i]
            p.resetJointState( bodyUniqueId=robotID, 
                jointIndex = jind,
                targetValue=center, physicsClientId=physicsClient )

    # parameters for alternating tripod
    amplitude_max = pi/8
    amplitudes = amplitude_max*np.ones([3,6])
    amplitudes[1,:] = amplitudes[1,:] + pi/16 # boost step height
    amplitudes[2,:] = amplitudes[1,:] + pi/16 # boost step height
    period = 1.5
    const_offsets = np.array([[-1,0,1,-1,0,1], 
        [0,0,0,0,0,0], [0,0,0,0,0,0]])*np.pi/4
    phase_offsets = np.array([[0.5,-0.5,0.5,-0.5,0.5,-0.5], # forward # NOTE: python2 needs to force float division
        [1,0,1,0,1,0], [0,1,0,1,0,1]])*np.pi



    # for test without 90deg module in hexapod
    const_offsets[1,:] -= np.pi/4
    const_offsets[2,:] += np.pi/4



### MAIN LOOP
    if SEND_COMMANDS:
        pub = rospy.Publisher('/eigenbot/joint_cmd', JointState, queue_size=1)
    ros_nodes = rosnode.get_node_names()
    print('Nodes running: ' + str(ros_nodes))
    if not ('/description_listener' in ros_nodes):
        # the "description_listener" makes the node
        rospy.init_node('description_listener') 
    continuous_joint_serials = []
    continuous_joint_inds = []
    for j_ind in range(num_joints):
        if np.isinf(moving_joint_limits[j_ind][1]):
            continuous_joint_serials.append(joint_serials[j_ind])
            continuous_joint_inds.append(j_ind)

    # for the purposes of ease, count the "extra" joints in the same way
    for ind in extra_joint_inds:
        continuous_joint_serials.append(joint_serials[ind])
        continuous_joint_inds.append(ind)

    if SEND_COMMANDS:
        # we need to get the continuous joint angles at the start
        # but the message is in a different order than the topology was a few seconds ago...

        continuous_joint_init_pos = [np.nan]*len(continuous_joint_serials)
        while np.any(np.isnan(continuous_joint_init_pos)):
            joint_fb = rospy.wait_for_message("/eigenbot/joint_fb", JointState)
            joint_pos_init = []
            print('Topology names:')
            print(joint_serials)
            print('Initial fb names:')
            print(joint_fb.name)
            # print('Initial fb positions:')
            # print(joint_fb.position)
            for i_serial in range(len(continuous_joint_serials)):
                serial = continuous_joint_serials[i_serial]
                if serial in joint_fb.name:
                    ind = joint_fb.name.index(serial)
                    pos = joint_fb.position[ind]
                    continuous_joint_init_pos[i_serial] = pos
        print('Got continuous joint intial positions ' + str(continuous_joint_init_pos))
    else:
        continuous_joint_init_pos = [0]*len(continuous_joint_serials)

    # rate_hz = 20 # too fast
    rate_hz = 13
    rate = rospy.Rate(rate_hz) # 10hz
    dt = 1./rate_hz
    dt_sim = 1./240 # default of pybullet
    n_steps_per_rate = dt/dt_sim
    n_steps_per_rate = int(np.round(n_steps_per_rate))
    joint_state = JointState()
    t=0
    j_positions= np.zeros(num_joints)
    # subtract out the initial positions of continuous joints
    for j in range(len(continuous_joint_inds)):
        j_ind = continuous_joint_inds[j]
        j_positions[j_ind] = continuous_joint_init_pos[j]

    wave_phase = 0

    print(' ----------- starting main loop -----------')
    while not rospy.is_shutdown():

        # joint_state.header.stamp = 'Eigenbot joint state'
        joint_state.header.stamp = rospy.Time.now()

        axes, buttons, povs = vrjoystick.read(joy)

        # button 9-12 ends it all
        if np.any(buttons[8:10]):
            break


        forward_cmd = axes[1]
        turn_cmd = axes[0]

        amplitudes[0,0:3] = amplitude_max*np.ones(3)*(-forward_cmd+turn_cmd)
        amplitudes[0,3:6] = amplitude_max*np.ones(3)*(forward_cmd+turn_cmd)
        amplitudes[0,:] = np.clip(amplitudes[0,:], -amplitude_max, amplitude_max)

        wave_phase = wrap_to_pi(wave_phase)
        if np.linalg.norm(axes[0:2])>0.01:
            wave_phase += dt
        # TODO: if we are not moving, interpolate to wave phase zero
        # elif np.abs(wave_phase) >= dt: 
        #     if (-pi/2 < wave_phase < 0) or (pi/2 < wave_phase < pi):
        #         wave_phase += dt
        #     else:
        #         wave_phase -= dt


        # given a joystick command, convert it into wheel velocities setpoints
        j_velocities = np.zeros(num_joints)
        for i in range(n_chassis_ports):
            if wheel_joint_ids[i] is not None:
                if i<3:
                    cmd = (-forward_cmd + turn_cmd)
                elif i>=3:
                    cmd = (forward_cmd + turn_cmd)

                index_of_wheel = joint_serials.index(wheel_joint_ids[i])
                # j_velocities[index_of_wheel] = cmd*moving_joint_max_velocities[index_of_wheel]
                j_velocities[index_of_wheel] = cmd*moving_joint_max_velocities[index_of_wheel]


        # given a joystick command, convert it into leg position setpoints
        for i in range(6):
            leg_angles_i = amplitudes[:,i]*np.sin(wave_phase*2*pi/period - phase_offsets[:,i]) 
            leg_angles_i[1:] = np.clip(leg_angles_i[2:], -np.inf,0) # convert up-down motion to up-flat motion
            leg_angles_i[2] = -leg_angles_i[2]
            leg_angles_i += const_offsets[:,i]

            leg_joint_ids_i = leg_joint_ids[i]
            if leg_joint_ids_i is not None:
                for j in range(len(leg_joint_ids_i)):
                    leg_joint_id = leg_joint_ids_i[j]
                    index_of_joint = joint_serials.index(leg_joint_id)
                    j_positions[index_of_joint] = leg_angles_i[j] * leg_joint_multiplier[i][j]

        j_positions += j_velocities*dt



        # Use buttons 1-8 as "clutch" to operate the other joints one at a time
        n_extra_joints = len(extra_joint_inds)
        n_extra_joints = min(n_extra_joints, 8)
        extra_buttons = buttons[:n_extra_joints]
        if np.any(extra_buttons):
            to_move = extra_joint_inds[np.where(extra_buttons)[0]]
            # print('Extra move: ' + str(to_move))
            j_positions[to_move] += axes[3]*dt*moving_joint_max_velocities[to_move]

        j_positions = np.clip(j_positions, 
            np.array(lower_limits), np.array(upper_limits))
       
        if RUN_SIM_GUI:

            p.setJointMotorControlArray(robotID, moving_joint_inds, 
                controlMode=p.POSITION_CONTROL,
                targetPositions = j_positions,
                forces = moving_joint_max_torques, physicsClientId=physicsClient)

            for tt in range(n_steps_per_rate):
                p.stepSimulation(physicsClientId=physicsClient)

        t+=dt



        # get serial numbers out of joint name fields 
        joint_state.name = []
        for joint_name in moving_joint_names:
            joint_serial = joint_name[joint_name.find('_S')+2:]
            joint_state.name.append(joint_serial)
        # Note: eigenbot driver requires all pos, vel, effort to be same length.
        joint_state.position = j_positions
        if np.any(buttons[10:]):
            joint_state.position =np.zeros(num_joints)

        joint_state.velocity = [float('nan')]*num_joints
        joint_state.effort = [float('nan')]*num_joints
        # print(np2str(joint_state.position))
        if SEND_COMMANDS:
            pub.publish(joint_state)
        rate.sleep()

if __name__ == '__main__':
    try:
        joint_state_talker()
    except rospy.ROSInterruptException:
        pass


    ############################## misc bits ###############'



# # hexapod test
robot_name = 'hexapod'
module_types_test = ['Eigenbody',
            'Bendy_module', 
            'Bendy_module',
            'Bendy_module', 
            'Bendy_module',
            'Bendy_module',
            'Bendy_module',
        'Bendy_module', 
        'Bendy_module',
        'Bendy_module', 
        'Bendy_module',
        'Bendy_module',
        'Bendy_module',
            'Bendy_module', 
            'Bendy_module',
            'Bendy_module', 
            'Bendy_module',
            'Bendy_module',
            'Bendy_module',
        'Static_90deg_module',
        'Static_90deg_module',
        'Static_90deg_module',
        'Static_90deg_module',
        'Static_90deg_module',
        'Static_90deg_module',
            'Foot_module',
            'Foot_module',
            'Foot_module',
            'Foot_module',
            'Foot_module',
            'Foot_module',
      ]
graph_edges_test = [
    [0, 1, 0, 0 ], # from node, to node, port, mount 
    [0, 2, 1, 0 ],
    [0, 3, 2, 0 ],
    [0, 4, 3, 0 ],
    [0, 5, 4, 0 ],
    [0, 6, 5, 0 ],
    [1, 7, 0, 6 ],
    [2, 8, 0, 6 ],
    [3, 9, 0, 6 ],
    [4, 10, 0, 2 ],
    [5, 11, 0, 2 ],
    [6, 12, 0, 2 ],
    [7, 13, 0, 0 ],
    [8, 14, 0, 0 ],
    [9, 15, 0, 0 ],
    [10, 16, 0, 0 ],
    [11, 17, 0, 0 ],
    [12, 18, 0, 0 ],
    [13, 19, 0, 0 ],
    [14, 20, 0, 0 ],
    [15, 21, 0, 0 ],
    [16, 22, 0, 0 ],
    [17, 23, 0, 0 ],
    [18, 24, 0, 0 ],
    [19, 25, 0, 0 ],
    [20, 26, 0, 0 ],
    [21, 27, 0, 0 ],
    [22, 28, 0, 0 ],
    [23, 29, 0, 0 ],
    [24, 30, 0, 0 ],
]

# flipped joints
# graph_edges_test = [
#     [0, 1, 0, 4 ], # from node, to node, port, mount 
#     [0, 2, 1, 4 ],
#     [0, 3, 2, 4 ],
#     [0, 4, 3, 0 ],
#     [0, 5, 4, 0 ],
#     [0, 6, 5, 0 ],
#         [1, 7, 0, 2 ],
#         [2, 8, 0, 2 ],
#         [3, 9, 0, 2 ],
#         [4, 10, 0, 2 ],
#         [5, 11, 0, 2 ],
#         [6, 12, 0, 2 ],
#         [7, 13, 0, 0 ],
#         [8, 14, 0, 0 ],
#         [9, 15, 0, 0 ],
#         [10, 16, 0, 0 ],
#         [11, 17, 0, 0 ],
#         [12, 18, 0, 0 ],
#         [13, 19, 0, 0 ],
#         [14, 20, 0, 0 ],
#         [15, 21, 0, 0 ],
#         [16, 22, 0, 0 ],
#         [17, 23, 0, 0 ],
#         [18, 24, 0, 0 ],
#         [19, 25, 0, 0 ],
#         [20, 26, 0, 0 ],
#         [21, 27, 0, 0 ],
#         [22, 28, 0, 0 ],
#         [23, 29, 0, 0 ],
#         [24, 30, 0, 0 ],
#     ]

# # hexapod test flipped joints
# robot_name_test = 'hexapod_mod'
# module_types_test = ['Eigenbody',
#             'Bendy_module', 
#             'Bendy_module',
#             'Bendy_module', 
#             'Bendy_module',
#             'Bendy_module',
#             'Bendy_module',
#         'Bendy_module', 
#         'Bendy_module',
#         'Bendy_module', 
#         'Bendy_module',
#         'Bendy_module',
#         'Bendy_module',
#             'Bendy_module', 
#             'Bendy_module',
#             'Bendy_module', 
#             'Bendy_module',
#             'Bendy_module',
#             'Bendy_module',
#         'Static_90deg_module',
#         'Static_90deg_module',
#         'Static_90deg_module',
#         'Static_90deg_module',
#         'Static_90deg_module',
#         'Static_90deg_module',
#             'Foot_module',
#             'Foot_module',
#             'Foot_module',
#             'Foot_module',
#             'Foot_module',
#             'Foot_module',
#       ]
# graph_edges_test = [
#     [0, 1, 0, 4 ], # from node, to node, port, mount 
#     [0, 2, 1, 4 ],
#     [0, 3, 2, 4 ],
#     [0, 4, 3, 4 ],
#     [0, 5, 4, 0 ],
#     [0, 6, 5, 0 ],
#         [1, 7, 0, 2 ],
#         [2, 8, 0, 2 ],
#         [3, 9, 0, 2 ],
#         [4, 10, 0, 6 ],
#         [5, 11, 0, 2 ],
#         [6, 12, 0, 2 ],
#         [7, 13, 0, 4 ],
#         [8, 14, 0, 0 ],
#         [9, 15, 0, 0 ],
#         [10, 16, 0, 0 ],
#         [11, 17, 0, 0 ],
#         [12, 18, 0, 0 ],
#         [13, 19, 0, 4 ],
#         [14, 20, 0, 0 ],
#         [15, 21, 0, 0 ],
#         [16, 22, 0, 0 ],
#         [17, 23, 0, 0 ],
#         [18, 24, 0, 0 ],
#         [19, 25, 0, 0 ],
#         [20, 26, 0, 0 ],
#         [21, 27, 0, 0 ],
#         [22, 28, 0, 0 ],
#         [23, 29, 0, 0 ],
#         [24, 30, 0, 0 ],
#     ]

# # wheel-hexapod test
# robot_name = 'hexapod-car'
# module_types_test = ['Eigenbody', # 0
#         'Wheel_module',# 1
#         'Bendy_module',  # 2
#             'Bendy_module',
#             'Bendy_module',
#             'Static_90deg_module',
#             'Foot_module',
#         'Bendy_module', # 7
#             'Bendy_module',
#             'Bendy_module',
#             'Static_90deg_module',
#             'Foot_module',
#         'Wheel_module', #12
#         'Bendy_module', # 13
#             'Bendy_module',
#             'Bendy_module',
#             'Static_90deg_module',
#             'Foot_module',
#         'Bendy_module', #18
#             'Bendy_module',
#             'Bendy_module',
#             'Static_90deg_module',
#             'Foot_module',
#       ]
# graph_edges_test = [# from node, to node, port, mount 
#     [0, 1, 0, 1], # wheel
#     [0, 2, 1, 4 ], # leg
#     [2, 3, 0, 2 ],
#     [3, 4, 0, 0 ],
#     [4, 5, 0, 0 ],
#     [5, 6, 0, 0 ],
#     [0, 7, 2, 4 ], # leg
#     [7, 8, 0, 2 ],
#     [8, 9, 0, 0 ],
#     [9, 10, 0, 0 ],
#     [10, 11, 0, 0 ],
#     [0, 12, 3, 3], # wheel
#     [0, 13, 4, 0 ], # leg
#     [13, 14, 0, 2 ],
#     [14, 15, 0, 0 ],
#     [15, 16, 0, 0 ],
#     [16, 17, 0, 0 ],
#     [0, 18, 5, 0 ], # leg
#     [18, 19, 0, 2 ],
#     [19, 20, 0, 0 ],
#     [20, 21, 0, 0 ],
#     [21, 22, 0, 0 ]   
#     ]

    # # Left side legs
    # for i in range(3):
    #     if leg_joint_orns[i] is not None:
    #         # the orientation accumulates with each module added. Find the total orn change
    #         leg_joint_orns_cumulative[i] = wrap_to_pi(np.cumsum(leg_joint_orns[i])*np.pi/4)
    #         leg_joint_multiplier[i] = [] # the orn causes the joint axes to flip from nominal

    #         # first joint, if it exists
    #         if len(leg_joint_orns_cumulative[i])>0:
    #             if np.abs(leg_joint_orns_cumulative[i][0]) <= np.pi/2 :
    #                 leg_joint_multiplier[i].append(1)
    #             else:
    #                 leg_joint_multiplier[i].append(-1)

    #         # second joint, if it exists
    #         if len(leg_joint_orns_cumulative[i])>1:
    #             if np.abs(leg_joint_orns_cumulative[i][1]) > np.pi/2:
    #                 leg_joint_multiplier[i].append(1)
    #             else:
    #                 leg_joint_multiplier[i].append(-1)

    #         # third joint, if it exists
    #         if len(leg_joint_orns_cumulative[i])>2:
    #             if np.abs(leg_joint_orns_cumulative[i][2]) > np.pi/2:
    #                 leg_joint_multiplier[i].append(1)
    #             else:
    #                 leg_joint_multiplier[i].append(-1)
    
    # # Right side legs
    # for i in range(3,6):
    #     if leg_joint_orns[i] is not None:
    #         leg_joint_orns_cumulative[i] = wrap_to_pi(np.cumsum(leg_joint_orns[i])*np.pi/4)
    #         leg_joint_multiplier[i] = []

    #         if len(leg_joint_orns_cumulative[i])>0:
    #             if np.abs(leg_joint_orns_cumulative[i][0]) <= np.pi/2 :
    #                 leg_joint_multiplier[i].append(1)
    #             else:
    #                 leg_joint_multiplier[i].append(-1)

    #         if len(leg_joint_orns_cumulative[i])>1:
    #             if np.abs(leg_joint_orns_cumulative[i][1]) > np.pi/2:
    #                 leg_joint_multiplier[i].append(-1)
    #             else:
    #                 leg_joint_multiplier[i].append(1)

    #         if len(leg_joint_orns_cumulative[i])>2:
    #             if np.abs(leg_joint_orns_cumulative[i][2]) > np.pi/2:
    #                 leg_joint_multiplier[i].append(-1)
    #             else:
    #                 leg_joint_multiplier[i].append(1)
