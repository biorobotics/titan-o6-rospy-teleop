#import bagpy
#from bagpy import bagreader
#import pandas as pd
#import seaborn as sea
#import matplotlib.pyplot as plt
#import numpy as np
#import rosbag
#from std_msgs.msg import Int32, String
#bag = rosbag.Bag('test.bag', 'w')

import rosbag
bag = rosbag.Bag(/home/biorobotics/bags/2020-12-10-22-20-59.bag)
for topic, msg, t in bag.read_messages(topics=['chatter', 'numbers']):
    print(msg)
bag.close()



#wget https://data.ouster.io/sample-data-1.13/OS1-64/OS1-64_city1.bag

#b = bagreader('OS1-64_city1.bag') 




#import bagpy
#import bagreader
#from bagpy import bagreader
#from .bagreader import bagreader
#import pandas as pd
#from std_msgs.msg import Int32, String

#bag = rosbag.Bag('test.bag', 'w')
#b = bagreader('test.bag')

#try:
#    s = String()
#    s.data = 'foo'
#    i = Int32()
#    i.data = 42
#    bag.write('chatter', s)
#    bag.write('numbers', i)
#finally:
#    bag.close()



#POSITION_CMD = b.message_by_topic('some path')

#POSITION_CMD

#df_position = pd.read_csv(POSITION_CMD)
#df_position
