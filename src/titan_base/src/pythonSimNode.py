#! /usr/bin/python
#Interperets /titan/joint_cmd and sends commands to the simulator
import rospy
from sensor_msgs.msg import *

class pythonSimNode(object):
    def __init__(self):
        rospy.init_node('pythonSimNode')
        
        self.b1 = rospy.Publisher('/o6/base1_position_controller/command', std_msgs.msg.Float64, queue_size=1)
        self.b2 = rospy.Publisher('/o6/base2_position_controller/command', std_msgs.msg.Float64, queue_size=1)
        self.b3 = rospy.Publisher('/o6/base3_position_controller/command', std_msgs.msg.Float64, queue_size=1)
        self.b4 = rospy.Publisher('/o6/base4_position_controller/command', std_msgs.msg.Float64, queue_size=1)
        self.b5 = rospy.Publisher('/o6/base5_position_controller/command', std_msgs.msg.Float64, queue_size=1)
        self.b6 = rospy.Publisher('/o6/base6_position_controller/command', std_msgs.msg.Float64, queue_size=1)

        self.e1 = rospy.Publisher('/o6/elbow1_position_controller/command', std_msgs.msg.Float64, queue_size=1)
        self.e2 = rospy.Publisher('/o6/elbow2_position_controller/command', std_msgs.msg.Float64, queue_size=1)
        self.e3 = rospy.Publisher('/o6/elbow3_position_controller/command', std_msgs.msg.Float64, queue_size=1)
        self.e4 = rospy.Publisher('/o6/elbow4_position_controller/command', std_msgs.msg.Float64, queue_size=1)
        self.e5 = rospy.Publisher('/o6/elbow5_position_controller/command', std_msgs.msg.Float64, queue_size=1)
        self.e6 = rospy.Publisher('/o6/elbow6_position_controller/command', std_msgs.msg.Float64, queue_size=1)

        self.s1 = rospy.Publisher('/o6/shoulder1_position_controller/command', std_msgs.msg.Float64, queue_size=1)
        self.s2 = rospy.Publisher('/o6/shoulder2_position_controller/command', std_msgs.msg.Float64, queue_size=1)
        self.s3 = rospy.Publisher('/o6/shoulder3_position_controller/command', std_msgs.msg.Float64, queue_size=1)
        self.s4 = rospy.Publisher('/o6/shoulder4_position_controller/command', std_msgs.msg.Float64, queue_size=1)
        self.s5 = rospy.Publisher('/o6/shoulder5_position_controller/command', std_msgs.msg.Float64, queue_size=1)
        self.s6 = rospy.Publisher('/o6/shoulder6_position_controller/command', std_msgs.msg.Float64, queue_size=1)
def callback(data):
    baseOff = 0
    hipOff = -0.76
    kneeOff = 2.16

    b1.publish(data.position[0]+baseOff)
    b2.publish(data.position[1]+baseOff)
    b3.publish(data.position[2]+baseOff)
    b4.publish(data.position[3]+baseOff)
    b5.publish(data.position[4]+baseOff)
    b6.publish(data.position[5]+baseOff)
    s1.publish(data.position[6]+hipOff)
    s2.publish(data.position[7]+hipOff)
    s3.publish(data.position[8]+hipOff)
    s4.publish(data.position[9]+hipOff)
    s5.publish(data.position[10]+hipOff)
    s6.publish(data.position[11]+hipOff)
    e1.publish(data.position[12]+kneeOff)
    e2.publish(data.position[13]+kneeOff)
    e3.publish(data.position[14]+kneeOff)
    e4.publish(data.position[15]+kneeOff)
    e5.publish(data.position[16]+kneeOff)
    e6.publish(data.position[17]+kneeOff)

    
if __name__ == "__main__":
    rospy.init_node('pythonSimNode')
    
    global b1 
    b1 = rospy.Publisher('/o6/base1_position_controller/command', std_msgs.msg.Float64, queue_size=1)
    global b2 
    b2 = rospy.Publisher('/o6/base2_position_controller/command', std_msgs.msg.Float64, queue_size=1)
    global b3 
    b3 = rospy.Publisher('/o6/base3_position_controller/command', std_msgs.msg.Float64, queue_size=1)
    global b4 
    b4 = rospy.Publisher('/o6/base4_position_controller/command', std_msgs.msg.Float64, queue_size=1)
    global b5 
    b5 = rospy.Publisher('/o6/base5_position_controller/command', std_msgs.msg.Float64, queue_size=1)
    global b6 
    b6 = rospy.Publisher('/o6/base6_position_controller/command', std_msgs.msg.Float64, queue_size=1)

    global e1 
    e1 = rospy.Publisher('/o6/elbow1_position_controller/command', std_msgs.msg.Float64, queue_size=1)
    global e2 
    e2 = rospy.Publisher('/o6/elbow2_position_controller/command', std_msgs.msg.Float64, queue_size=1)
    global e3 
    e3 = rospy.Publisher('/o6/elbow3_position_controller/command', std_msgs.msg.Float64, queue_size=1)
    global e4 
    e4 = rospy.Publisher('/o6/elbow4_position_controller/command', std_msgs.msg.Float64, queue_size=1)
    global e5 
    e5 = rospy.Publisher('/o6/elbow5_position_controller/command', std_msgs.msg.Float64, queue_size=1)
    global e6 
    e6 = rospy.Publisher('/o6/elbow6_position_controller/command', std_msgs.msg.Float64, queue_size=1)

    global s1 
    s1 = rospy.Publisher('/o6/shoulder1_position_controller/command', std_msgs.msg.Float64, queue_size=1)
    global s2 
    s2 = rospy.Publisher('/o6/shoulder2_position_controller/command', std_msgs.msg.Float64, queue_size=1)
    global s3 
    s3 = rospy.Publisher('/o6/shoulder3_position_controller/command', std_msgs.msg.Float64, queue_size=1)
    global s4 
    s4 = rospy.Publisher('/o6/shoulder4_position_controller/command', std_msgs.msg.Float64, queue_size=1)
    global s5 
    s5 = rospy.Publisher('/o6/shoulder5_position_controller/command', std_msgs.msg.Float64, queue_size=1)
    global s6 
    s6 = rospy.Publisher('/o6/shoulder6_position_controller/command', std_msgs.msg.Float64, queue_size=1)
    print('INITIALIZED')
    
    rospy.Subscriber('/titan/joint_cmd', JointState, callback)
    rospy.spin()