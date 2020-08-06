#!/usr/bin/env python
import rospy
import sys
import tf2_ros
import tf # conversion euler
import random
from geometry_msgs.msg import Twist # topic /cmd_vel
from math import atan2,acos,sqrt,pi,sin,cos,tan
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker, MarkerArray # Debug drawing
from geometry_msgs.msg import Point

import time # for testing 

#########################
### Global parameters ###
#########################
TOW_CAR_LENGTH = 0.93 # meter, Length between two cars
V_MAX = 0.3 # m/s, Max velocity
W_MAX = 0.8 # rad/s, MAX angular velocity
KP_crab = 0.8 # KP for crab mode, the bigger the value, the faster it will chase ref_ang
KP_diff = 1.5 # KP fro diff mode
KI = 0
# Global variable
MARKER_LINE = MarkerArray()# Line markers show on RVIZ

class Rap_controller():
    def __init__(self,robot_name, role, sim, ctl_frequency):
        # Subscriber
        rospy.Subscriber("/"+robot_name+"/"+"naive_cmd", Twist, self.cmd_cb)
        # Publisher
        self.pub_cmd_vel = rospy.Publisher("/"+robot_name+'/cmd_vel', Twist,queue_size = 1,latch=False)
        self.pub_marker_line = rospy.Publisher("/"+robot_name+'/rap_angle_marker_line', MarkerArray,queue_size = 1,latch=False)
        # Parameters
        self.robot_name = robot_name
        self.role = role
        self.sim = sim
        self.dt = 1.0 / ctl_frequency
        # Tf listner
        self.tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tfBuffer)
        self.base_link_xyt = None # (x,y,theta)
        self.big_car_xyt = None  # (x,y,theta)
        self.theta = None
        # Kinematics 
        self.Vc = 0
        self.Vy = 0
        self.Wc = 0
        self.ref_ang = 0
        # Output 
        self.v_out = None
        self.w_out = None
        # Flags
        self.mode = "diff" # "crab"
        # PID
        self.cmd_last = 0
        self.error_last = 0 
        self.sum_term = 0
    
    def sim_theta_cb(self, data):
        '''
        Get theta from topics
        '''
        self.base_link_xyt = (None, None, data.data)

    def cmd_cb(self,data):
        '''
        Topic /<robot_name>/cmd_vel callback function
        Argument: 
            data - geometry_msgs/Twist
            geometry_msgs/Vector3 linear
                float64 x
                float64 y
                float64 z
            geometry_msgs/Vector3 angular
                float64 x - ref_ang
                float64 y - mode, 1 means differtial mode, 0 means crab mode
                float64 z
        '''
        self.Vc = data.linear.x
        self.Vy = data.linear.y
        self.Wc = data.angular.z
        self.ref_ang = data.angular.x
        if data.angular.y == 0: # TODO get rid of mode 
            self.mode = "diff"
        else: 
            self.mode = "crab"
    
    def normalize_angle(self,angle):
        '''
        Make angle inside range [-pi, pi]
        Arguments:
            angle - flaot
        Return:
            float
        '''
        ans = (abs(angle) % (2*pi))*self.sign(angle)
        if ans < -pi: # [-2pi, -pi]
            ans += 2*pi
        elif ans > pi: # [pi, 2pi]
            ans -= 2*pi
        return ans
    
    def sign(self, value):
        if value >= 0:
            return 1 
        if value < 0:
            return -1

    def pi_controller(self, kp, ki,error):
        '''
        '''
        # Yulin's suggestion
        #cmd = self.cmd_last + error*kp + (kp-ki*DT)*self.error_last
        #self.cmd_last = cmd
        #self.error_last = error
        
        # Doris suggestion
        # self.sum_term = kp*ki*DT*error * 0.05 + self.sum_term * 0.95
        self.sum_term += kp*ki*self.dt*error
        cmd = kp*error + self.sum_term
        return cmd
    
    def crab_controller(self,vx,vy,error):
        '''
        Return leader crab controller result
        '''
        v_con = self.sign(vx) * sqrt(vx**2 + vy**2) * abs(cos(error))
        # w = KP_crab*error
        w_con = self.pi_controller(KP_crab, KI, error)
        return (v_con, w_con)
    
    def diff_controller(self,vx,wz,error,ref_ang):
        '''
        Return leader crab controller result
        '''
        print ("diff_controller")
        R = self.get_radius_of_rotation(vx, wz)
        if R == float("inf"):# Go straight
            v_con = vx
            w_con = self.pi_controller(KP_diff, KI, error)
        else:
            v_con = (sqrt(R**2 + (TOW_CAR_LENGTH/2.0)**2)*wz) *abs(cos(error))
            if abs(R) < 0.1:  # TODO
                w_con = wz*abs(cos(error)) + self.pi_controller(KP_diff, KI, error)
                if ref_ang < 0: # ref_ang == -pi/2
                    v_con *= -1
            else:
                if not self.is_same_sign(vx,v_con):
                    v_con *= -1
                w_con = self.sign(vx)*wz*abs(cos(error)) + self.pi_controller(KP_diff, KI, error)
        return (v_con, w_con)
    
    def rota_controller(self,wz,error,ref_ang):
        '''
        Inplace rotation controller
        '''
        print ("rota_controller")
        v_con = (TOW_CAR_LENGTH/2.0)*wz*abs(cos(error)) # TODO test
        # v_con = (sqrt(R**2 + (TOW_CAR_LENGTH/2.0)**2)*wz) *abs(cos(error))
        w_con = wz*abs(cos(error)) + self.pi_controller(KP_diff, KI, error)
        if ref_ang < 0: # ref_ang == -pi/2
            v_con = -v_con
        return (v_con, w_con)

    def get_radius_of_rotation(self,v,w):
        try:
            radius = v / w
        except ZeroDivisionError:
            return float("inf")
        else:
            return radius 

    def run_once(self):
        '''
        call by main loop, execute every loop.
        Return:
            True - Calculate successfully, need publish
            False - Can't finish calculation, don't publish
        '''
        # Update tf
        if self.sim:
            t_base_link = self.get_tf("map", self.robot_name)
            t_big_car   = self.get_tf("map", "base_link")
        else:
            t_base_link = self.get_tf(self.robot_name + "/map",
                          self.robot_name + "/base_link")
            t_big_car   = self.get_tf(self.robot_name + "/map",
                          self.robot_name + "/center_big_car")
        if t_base_link != None:
            self.base_link_xyt = t_base_link
        if t_big_car != None:
            self.big_car_xyt = t_big_car
        
        if self.base_link_xyt == None or self.big_car_xyt == None: #tf is invalid
            return False
        
        # Get current theta
        self.theta = self.normalize_angle(self.normalize_angle(self.base_link_xyt[2])
                                        - self.normalize_angle(self.big_car_xyt[2]))
        
        #################
        ### DIFF MODE ###
        #################
        if self.mode == "diff":
            # Get refenrence angle
            R = self.get_radius_of_rotation(self.Vc, self.Wc)
            self.ref_ang = atan2(TOW_CAR_LENGTH/2.0, abs(R))
            if not self.is_same_sign(R, self.Vc):
                self.ref_ang *= -1
            print ("R = "  + str(R))
            # if self.Vc == 0:# In-place rotation
            
            if abs(R) < 0.1:  # TODO 
                # Choose a nearest ref_ang to prsue, two possibility: (-pi/2, pi/2)
                if  abs(self.ref_ang - self.theta) > abs(-self.ref_ang - self.theta) and\
                    self.theta < -0.2:
                    self.ref_ang *= -1
            
            # Get error
            if self.role == "follower":
                self.ref_ang = self.normalize_angle(pi - self.ref_ang)
            error_theta = self.normalize_angle(self.ref_ang - self.theta)

            # Get v_out, w_out
            # if self.Vc != 0:
            (self.v_out, self.w_out) = self.diff_controller(self.Vc, self.Wc, error_theta, self.ref_ang)
            '''
            if abs(R) < 0.1:  # TODO 
                (self.v_out, self.w_out) = self.rota_controller(self.Wc, error_theta, self.ref_ang)
            else:
                (self.v_out, self.w_out) = self.diff_controller(self.Vc, self.Wc, error_theta)
            '''
            # Reverse follower heading velocity
            if self.role == "follower":
                self.v_out = -self.v_out
        
        #################
        ### CRAB MODE ###
        #################
        elif self.mode == "crab":
            # Get refenrence angle
            if self.Vc >= 0: # Go forward
                self.ref_ang = atan2(self.Vy, self.Vc)
            elif self.Vc < 0: # Go backward
                self.ref_ang = self.normalize_angle(atan2(self.Vy, self.Vc) + pi)
            
            # Get error
            if self.role == "follower":
                self.ref_ang += pi
            error_theta = self.normalize_angle(self.ref_ang - self.theta)

            # Get v_out, w_out
            (self.v_out, self.w_out) = self.crab_controller(self.Vc, self.Vy, error_theta)
            
            # Reverse follower heading velocity
            if self.role == "follower":
                self.v_out = -self.v_out

        # Saturation velocity, for safty
        self.v_out = self.saturation(self.v_out, V_MAX)
        self.w_out = self.saturation(self.w_out, W_MAX)
        
        # Set marker line
        # Reference ang
        p1 = self.base_link_xyt[:2]
        p2 = (p1[0] + TOW_CAR_LENGTH/2.0 * cos(self.ref_ang + self.big_car_xyt[2]),
              p1[1] + TOW_CAR_LENGTH/2.0 * sin(self.ref_ang + self.big_car_xyt[2]))
        set_line([p1, p2], RGB = (0,0,255), size = 0.02 ,id = 0)
        # Current ang
        p2 = (p1[0] + TOW_CAR_LENGTH/2.0 * cos(self.base_link_xyt[2]),
              p1[1] + TOW_CAR_LENGTH/2.0 * sin(self.base_link_xyt[2]))
        set_line([p1, p2], RGB = (102,178,255), size = 0.02 ,id = 1)
        
        # Set publish flag
        return True

    def saturation(self, value, maximum):
        '''
        Let input value can't exceed maximum
        Arguments:
            value - float
            maximum - float
        Return:
            float : saturation value
        '''
        try:
            if abs(value) > maximum:
                if value > 0: # positive
                    value = maximum
                else: # nagative
                    value = -maximum
        except TypeError:
            pass
        finally:
            return value

    def is_same_sign(self, a, b):
        '''
        Check whether a,b are same sign 
        arguments:
            a - float/int
            b - float/int
        Return: 
            Bool - True means a,b have same sign, False means they don't
        '''
        if (a >= 0 and b >= 0) or (a < 0 and b < 0):
            return True
        else:
            return False

    def get_tf(self,frame_id, child_frame_id):
        '''
        get tf frame_id -> child_frame_id
        Arguments:
            frame_id(str): e.g: "map", "odom"
            child_frame_id(str): e.g: "base_link"
        Return:
            (x,y,theta)
            None, if tf is unvaliable
        '''
        try:
            t = self.tfBuffer.lookup_transform(frame_id,
                                               child_frame_id,
                                               rospy.Time(),
                                               rospy.Duration(0.1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("[rap_controller] Can't get tf frame: " + frame_id + " -> " + child_frame_id)
            return None
        else:
            quaternion = (
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            return (t.transform.translation.x, t.transform.translation.y, euler[2])

#######################
### Global function ###
#######################
def set_line(points , RGB = None , size = 0.2, id = 0):
    '''
    Set line at MarkArray
    Input : 
        points = [p1,p2....]
        RGB - tuple : (255,255,255)
        size - float: width of line
        id - int
    '''
    global MARKER_LINE
    marker = Marker()
    marker.header.frame_id = "map"
    marker.id = id
    marker.ns = "tiles"
    marker.header.stamp = rospy.get_rostime()
    marker.type = marker.LINE_STRIP
    marker.action = marker.ADD
    marker.scale.x = size
    marker.scale.y = size
    marker.scale.z = size
    marker.color.a = 1.0
    if RGB == None : 
        marker.color.r = random.randint(0,255) / 255.0
        marker.color.g = random.randint(0,255) / 255.0
        marker.color.b = random.randint(0,255) / 255.0
    else: 
        marker.color.r = RGB[0]/255.0
        marker.color.g = RGB[1]/255.0
        marker.color.b = RGB[2]/255.0
    marker.pose.orientation.w = 1.0
    for i in points:
        p = Point()
        p.x = i[0]
        p.y = i[1]
        marker.points.append(p)
    MARKER_LINE.markers.append(marker)

def main(args):
    global MARKER_LINE
    rospy.init_node('rap_controller',anonymous=False)
    # Get global parameters
    ROBOT_NAME = rospy.get_param(param_name="~robot_name", default="car1")
    ROLE = rospy.get_param(param_name="~role", default="leader")
    SIM  = rospy.get_param(param_name="~sim", default="false")
    REVERSE_OMEGA = rospy.get_param(param_name="~reverse_omega", default="false")
    CONTROL_FREQ = rospy.get_param(param_name="~ctl_frequency", default="10")
    
    # Init naive controller
    rap_controller = Rap_controller(ROBOT_NAME, ROLE,SIM, CONTROL_FREQ)
    
    rate = rospy.Rate(CONTROL_FREQ)
    while not rospy.is_shutdown():
        if rap_controller.run_once():
            cmd_vel = Twist()
            cmd_vel.linear.x  = rap_controller.v_out
            cmd_vel.angular.z = rap_controller.w_out
            if REVERSE_OMEGA: # This is for weird simulation bug
                cmd_vel.angular.z = -cmd_vel.angular.z
            
            rap_controller.pub_cmd_vel.publish(cmd_vel)
            # Publish marker, for debug
            rap_controller.pub_marker_line.publish(MARKER_LINE)
            # Debug print
            rospy.logdebug(rap_controller.role + " : V=" + str(round(rap_controller.v_out, 3)) +
                                                 ", W=" + str(round(rap_controller.w_out, 3)))
            MARKER_LINE = MarkerArray()
        rate.sleep()

if __name__ == '__main__':
    try:
       main(sys.argv)
    except rospy.ROSInterruptException:
        pass
