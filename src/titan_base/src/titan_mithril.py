import time
from math import sin, cos, asin, acos, atan2, atan, sqrt, pi, isnan
import numpy as np
import matplotlib.pyplot as plt
import rospy
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Joy
import matplotlib.colors as mcolors

OUTPUT = "ROS"

first_fb = False

pos = np.zeros(18)
ref_pos = np.zeros(18)

all_l = ['16', '10', '0E', '01', '0D', '14', '05', '09', '11', '0F', '08', '04', '13', '0A', '07', '02', '03', '0C']

con_mode = "STAND"
old_mode = "STAND"

l1 = 0.5
l2 = 1.2

joy_cmd = [0, 0, 0]
joy_but = [0, 0, 0]

ang_offs = [-2.1642, 0.767945, 0]

z_con = [-1*l2, -1*l2, -1*l2, -1*l2, -1*l2, -1*l2]
x_con = [l1, l1, l1, l1, l1, l1]
y_con = [0, 0, 0, 0, 0, 0]

i_z_con = [-1*l2, -1*l2, -1*l2, -1*l2, -1*l2, -1*l2]
i_x_con = [l1, l1, l1, l1, l1, l1]
i_y_con = [0, 0, 0, 0, 0, 0]

stand_h = 1.1
stand_rad = 0.63

#Trapezoidal definition parameters
step_dis = 0.2
step_h = 0.15

ramp_por = 0.15
m = 1/(0.5-ramp_por)
b = step_h/2+step_h*m/4
k = m/(2*ramp_por)
m2 = 1/(1-ramp_por)
k2 = m2/(2*ramp_por)


def t2xyz(t1, t2, t3):
  xyz = [0, 0, 0]
  t1 = t1+ang_offs[0]
  t2 = t2+ang_offs[1]
  t3 = t3+ang_offs[2]

  reach = l1*cos(t2) + l2*cos(t2+t1)
  xyz[0] = reach*cos(t3)
  xyz[1] = reach*sin(t3)

  xyz[2] = l1*sin(t2) + l2*sin(t2+t1)
  return xyz

def xyzS2t(x, y, z, leg):
  t1=atan(y/x)

  w = sqrt(x*x+y*y)

  t3 = -1*acos((z*z+w*w-l1*l1-l2*l2)/(2*l1*l2))
  t2 = -1*atan2(z, w) + atan2(l2*sin(t3), l1+l2*cos(t3))

  t_arr = [t3, -1*t2, t1]

  for i in range(0, 3):
    t_arr[i] = t_arr[i] - ang_offs[i]

  t_ind = 0
  l = (leg-1)*3

  while l < (leg-1)*3+3:
    if(abs(t_arr[t_ind] - ref_pos[l]) < 0.4):
      if(OUTPUT == 'PLT'):
        ref_pos[l] = t_arr[t_ind] #Faking perfect control in simulation case
      pos[l] = t_arr[t_ind]
    elif(abs(t_arr[t_ind] - ref_pos[l]) > 5.5):
      offset = 2*pi*round((ref_pos[l] - pos[l])/(2*pi))
      pos[l] = t_arr[t_ind] + offset
    else:
      print('invalid command to joint: ' + all_l[l])
    l = l + 1
    t_ind = t_ind + 1

def smoothSlide(init, fin, prog):
  x = 0

  if(prog < ramp_por):
    x = init + k2*(fin-init)*prog*prog
  elif(prog < (1-ramp_por)):
    x = init + (fin-init)*(m2*prog+k2*ramp_por*ramp_por-m2*ramp_por)
  else:
    x = fin- k2*(fin-init)*(1-prog)*(1-prog)

  return x;


def stepAngles(init_pos, last_pos, leg, up, prog):
  p_sym = abs(0.5-prog)
  z = -1*stand_h
  x = stand_rad
  y = 0

  if(up):
    if(p_sym < ramp_por):
      z = -1*stand_h + step_h - k * step_h * p_sym*p_sym
    elif(p_sym < (0.5-ramp_por)):
      z = -1*stand_h + b - m*step_h*p_sym
    else:
      z = -1*stand_h + k*step_h*(0.5-p_sym)*(0.5-p_sym)
  x = smoothSlide(stand_rad+init_pos[0], stand_rad+last_pos[0], prog)
  y = smoothSlide(init_pos[1], last_pos[1], prog)

  xyzS2t(x, y, z, leg)


def joy_cb(msg):
  for i in range(0, 3):
    joy_cmd[i] = msg.axes[i]
  for i in range(0, 3):
    joy_but[i] = msg.buttons[i]

def fb_cb(msg):
  global first_fb
  for i in range(0, len(msg.name)):
    this_name = msg.name[i];

    for j in range(0, 18):
      if(this_name == all_l[j] and not isnan(msg.position[i])):
        ref_pos[j] = msg.position[i]

    if(first_fb):
      for leg in range(0, 6):
        t3 = t2xyz(ref_pos[leg], ref_pos[leg+6], ref_pos[leg+12])

        i_x_con[leg] = t3[0]
        i_y_con[leg] = t3[1]
        i_z_con[leg] = t3[2]
  first_fb = False

def mstop_cb(event):
  tog_mstop()

def tog_mstop():
  global con_mode
  global old_mode
  if(con_mode != 'MSTOP'):
    print('MSTOP -ed')
    old_mode = con_mode
    con_mode = 'MSTOP'
  else:
    con_mode = old_mode
    old_mode = 'MSTOP'


def wave(): #period, amplitude, y_offset, x_offset=0):
  d_wait = 0
  d_test = 60

  t_start = time.time()
  t_now = t_start
  t_pause_com = time.time()

  rospy.init_node('scripted_motion')
  rate = rospy.Rate(20)
  pub = rospy.Publisher('titan/joint/cmd', JointState, queue_size=1)

  rospy.Subscriber("joy", Joy, joy_cb)
  rospy.Subscriber("/titan/joint/fb", JointState, fb_cb)

  fig, ax = plt.subplots()
  fig.canvas.mpl_connect('key_press_event', mstop_cb)

  kill = False

  for l in range(0, 6):
    xyz = t2xyz(ref_pos[3*l], ref_pos[3*l+1], ref_pos[3*l+2])
    i_x_con[l] = xyz[0];  
    i_y_con[l] = xyz[1];  
    i_z_con[l] = xyz[2];

  print(i_z_con)

  global con_mode
  stand_prog = 0
  curr_inc = 0
  inc_per = 30

  #Gait phase params
  phase = 0
  phase_leg = [[1, 0, 0, 1, 0, 0], [0, 1, 0, 0, 1, 0], [0, 0, 1, 0, 0, 1]]
  #phase_leg = [[1, 0, 1, 0, 1, 0], [0, 1, 0, 1, 0, 1]]
  num_phase = 3

  last_leg_arr = [[0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0]]
  next_leg_arr = [[0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0]]
 
  steps = 0
  maxsteps = 10

  while not kill and not rospy.is_shutdown():
      #print(ref_pos)
      dt = time.time() - t_now
      t_now = time.time()
      t_elapsed = t_now - t_start

      #Check for mstop press from controller
      if(joy_but[0]):
        tog_mstop()

      if con_mode == 'STAND':
        #stand_prog = stand_prog + joy_cmd[0]*0.003;
        stand_prog = ((t_elapsed-5)/20)


        stand_prog = max(min(stand_prog, 1.0), 0.0);

        if(stand_prog < 0.0001):
          for l in range(0, 6):
            xyz = t2xyz(ref_pos[3*l], ref_pos[3*l+1], ref_pos[3*l+2])
            i_x_con[l] = xyz[0];  
            i_y_con[l] = xyz[1];  
            i_z_con[l] = xyz[2];

        for leg in range(0, 6):
          if(stand_prog < 0.5):
            x_con[leg] = smoothSlide(i_x_con[leg], stand_rad+0.05, stand_prog*2)
            y_con[leg] = smoothSlide(i_y_con[leg], 0, stand_prog*2)
            z_con[leg] = smoothSlide(i_z_con[leg], -0.61, stand_prog*2)
          elif (stand_prog >= 0.5) and (stand_prog < 1):
            x_con[leg] = smoothSlide(stand_rad+0.05, stand_rad, (stand_prog-0.5)*2)
            y_con[leg] = 0
            z_con[leg] = smoothSlide(-0.61, -1*stand_h, (stand_prog-0.5)*2)
          else:
            t_start = time.time()
            con_mode = 'PAUSE'

        for leg in range(0, 6):
          xyzS2t(x_con[leg], y_con[leg], z_con[leg], leg+1);

      elif con_mode == 'PAUSE':
        if(t_elapsed > 4):
          t_start = time.time()
          con_mode = 'STEP'
          for l in range(0, 6):
            xyz = t2xyz(ref_pos[3*l], ref_pos[3*l+1], ref_pos[3*l+2])
            i_x_con[l] = xyz[0];  
            i_y_con[l] = xyz[1];  
            i_z_con[l] = xyz[2];
        
      elif con_mode == 'STEP':
        tv = joy_cmd[1];
        rv = joy_cmd[0];

        #if(OUTPUT=='PLT'):
        tv = 1
      
        if(curr_inc >= inc_per):
          #if(joy_cmd[2] != 0):
          steps = steps + 1
          if(steps >= maxsteps):
            con_mode = 'SIT'
            t_start = time.time()

            for l in range(0, 6):
              xyz = t2xyz(ref_pos[3*l], ref_pos[3*l+1], ref_pos[3*l+2])
              i_x_con[l] = xyz[0];  
              i_y_con[l] = xyz[1];  
              i_z_con[l] = xyz[2];

          phase = (phase+1)%num_phase;
          curr_inc = 0;

          for leg in range(0, 6):
            last_leg_arr[leg][0] = next_leg_arr[leg][0]
            last_leg_arr[leg][1] = next_leg_arr[leg][1]

            phi_lin = leg*pi/3 + pi/6;

            if(phase_leg[phase][leg]):
              next_leg_arr[leg][0] = (num_phase-1)*step_dis/num_phase*(tv*cos(phi_lin))
              next_leg_arr[leg][1] = (num_phase-1)*step_dis/num_phase*(tv*sin(phi_lin) + rv)
            else:
              next_leg_arr[leg][0] = -1*step_dis/num_phase*(tv*cos(phi_lin));
              next_leg_arr[leg][1] = -1*step_dis/num_phase*(tv*sin(phi_lin)+rv);
        
        prog = (float(curr_inc))/inc_per

        for leg in range(0, 6):
          stepAngles([last_leg_arr[leg][0], last_leg_arr[leg][1]], [next_leg_arr[leg][0], next_leg_arr[leg][1]], leg+1, phase_leg[phase][leg], prog)
        
        if (curr_inc > 0 or tv != 0):
          curr_inc = curr_inc + 1


      elif con_mode == 'SIT':
        #stand_prog += joy_cmd[2]*0.0015
        stand_prog = ((20-t_elapsed)/20)
        print(stand_prog)
        stand_prog = max(min(stand_prog, 1.0), 0)
        
        for leg in range(0, 6):
          x_con[leg] = i_x_con[leg]
          y_con[leg] = i_y_con[leg]
          z_con[leg] = smoothSlide(-0.75, i_z_con[leg], stand_prog)
          xyzS2t(x_con[leg], y_con[leg], z_con[leg], leg+1)

      elif con_mode == 'MSTOP':
        #continue freezing
        t_start = t_start + dt

      else:
        print("Unrecognized control mode")

      if OUTPUT == "ROS" and time.time() - t_pause_com > 2:
          js = JointState()
          js.name = all_l
          js.position = pos.tolist()
          js.velocity = [float('nan')]*len(pos)
          js.effort = [float('nan')]*len(pos)
          pub.publish(js)
      
      #time.sleep(0.005)
      if OUTPUT == "PLT" or OUTPUT == "ROS":
          for pl in range(0, 18):
            ax.scatter(t_now, pos[pl], color=(float(pl%5)/4, float(pl%11+5)/17, float(pl%6+2)/8), marker='o')
          plt.draw()
          plt.pause(0.005)
      if OUTPUT == "ROS":
          rate.sleep()

if __name__ == "__main__":
  wave()
  #time.sleep(3)
