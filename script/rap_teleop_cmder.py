#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist

from math import atan2,acos,sqrt,pi,sin,cos,tan

import sys, select, termios, tty

TOW_CAR_LENGTH = 0.93 # Length between two cars

msg = """
Control Your Turtlebot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly
y : toggle mode, crab mode or differental mode

CTRL-C to quit
"""

moveBindings = {
        'i':(1,0),
        'o':(1,-1),
        'j':(0,1),
        'l':(0,-1),
        'u':(1,1),
        ',':(-1,0),
        '.':(-1,1),
        'm':(-1,-1),
           }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
          }

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

speed = .2
turn = 0.2

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

def normalize_angle(angle):
    '''
    [-pi, pi]
    '''
    sign = None 
    if angle >= 0:
        sign = 1
    else: 
        sign = -1 

    ans = angle % (2* pi * sign)

    if ans < -pi: # [-2pi, -pi]
        ans += 2*pi
    elif ans > pi: # [pi, 2pi]
        ans -= 2*pi
    return ans

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('naive_teleop')
    '''
    # GLOBAL paramters loading from rosparam server, to determinant topic name of cmd_vel
    is_parameters_set = False
    while (not is_parameters_set and not rospy.is_shutdown() ):
        try:
            robot_name = rospy.get_param("/unique_parameter/robot_name") # Find paramters in ros server
            is_parameters_set = True
        except:
            rospy.loginfo("robot_name are not found in rosparam server, keep on trying...")
            rospy.sleep(0.2) # Sleep 0.2 seconds for waiting the parameters loading
            continue
    '''
    pub_cmd_car1 = rospy.Publisher("car1/naive_cmd", Twist, queue_size=5)
    pub_cmd_car2 = rospy.Publisher("car2/naive_cmd", Twist, queue_size=5)
    
    x = 0
    th = 0
    status = 0
    count = 0
    acc = 0.1
    target_speed = 0
    target_turn = 0
    control_speed = 0
    control_turn = 0
    mode = "crab" # "diff"
    ref_ang = 0
    try:
        print msg
        print vels(speed,turn)
        while True:
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
                count = 0
            elif key in speedBindings.keys():# Change speed
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                count = 0

                print vels(speed,turn)
                if (status == 14):
                    print msg
                status = (status + 1) % 15
            elif key == ' ' or key == 'k': # Stop immediately
                x = 0
                th = 0
                control_speed = 0
                control_turn = 0
                ref_ang = 0
            elif key == 'y': # toggle mode 
                if mode == "crab":
                    mode = "diff"
                    print ("[teleop_naive_controller] Switch to DIFF mode")
                elif mode == "diff":
                    mode = "crab"
                    print ("[teleop_naive_controller] Switch to CRAB mode")
                x = 0
                th = 0
                control_speed = 0
                control_turn = 0
                ref_ang = 0
                continue
            else:
                count += 1
                if count > 4:
                    x = 0
                    th = 0
                if (key == '\x03'):# terminate
                    break
            
            target_speed = speed * x
            target_turn = turn * th


            if target_speed > control_speed:
                control_speed = min( target_speed, control_speed + 0.02 )
            elif target_speed < control_speed:
                control_speed = max( target_speed, control_speed - 0.02 )
            else:
                control_speed = target_speed

            if target_turn > control_turn:
                control_turn = min( target_turn, control_turn + 0.1 )
            elif target_turn < control_turn:
                control_turn = max( target_turn, control_turn - 0.1 )
            else:
                control_turn = target_turn

            if mode == "crab":
                ref_ang += control_turn
                twist = Twist()
                twist.linear.x = control_speed
                twist.linear.y = 0
                twist.linear.z = 0
                twist.angular.x = normalize_angle(ref_ang)
                twist.angular.y = 0 # Set mode to crab
                twist.angular.z = 0
                
                pub_cmd_car1.publish(twist)
                pub_cmd_car2.publish(twist)
                #print("loop: {0}".format(count))
                #print("target: vx: {0}, wz: {1}".format(target_speed, target_turn))
                #print("publihsed: vx: {0}, wz: {1}".format(twist.linear.x, twist.angular.z))
            if mode == "diff":
                ref_ang += control_turn
                '''
                if ref_ang == 0.0:# go stairght
                    R = 0
                else:
                    R = TOW_CAR_LENGTH / (2*tan(ref_ang)) # rotation center radius
                if abs(ref_ang) <= 1.047 :# smaller then 60 degree
                    twist = Twist()
                    twist.linear.x = control_speed
                    twist.linear.y = 0
                    twist.linear.z = 0
                    twist.angular.x = normalize_angle(ref_ang)
                    twist.angular.y = 1 # Set mode to diff
                    if R == 0.0:
                        twist.angular.z = 0
                    else:
                        twist.angular.z = control_speed / R
                else: 
                    twist = Twist()
                    twist.linear.x = R * control_speed
                    twist.linear.y = 0
                    twist.linear.z = 0
                    twist.angular.x = normalize_angle(ref_ang)
                    twist.angular.y = 1 # Set mode to diff
                    twist.angular.z = control_speed
                '''
                twist = Twist()
                twist.linear.x = control_speed
                twist.linear.y = 0
                twist.linear.z = 0
                twist.angular.x = 0
                twist.angular.y = 1 # Set mode to diff
                twist.angular.z = control_turn
                pub_cmd_car1.publish(twist)
                pub_cmd_car2.publish(twist)
                #print("loop: {0}".format(count))
                #print("target: vx: {0}, wz: {1}".format(target_speed, target_turn))
                #print("publihsed: vx: {0}, wz: {1}".format(twist.linear.x, twist.angular.z))

    except Exception as e:
        print e

    finally:
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        pub_cmd_car1.publish(twist)
        pub_cmd_car2.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
