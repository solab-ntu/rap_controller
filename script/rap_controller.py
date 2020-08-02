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

#########################
### Global parameters ###
#########################
TOW_CAR_LENGTH = 0.93 # meter, Length between two cars
V_MAX = 0.3 # m/s, Max velocity
W_MAX = 0.8 # rad/s, MAX angular velocity
KP_crab = 0.8 # KP for crab mode, the bigger the value, the faster it will chase ref_ang
KP_diff = 1.5 # KP fro diff mode
KI = 0
DT = 0.1 # sec

# Global variable
MARKER_LINE = MarkerArray()# Line markers show on RVIZ

class Rap_controller():
    def __init__(self,robot_name, role, sim):
        # Subscriber
        rospy.Subscriber("/"+robot_name+"/"+"naive_cmd", Twist, self.cmd_cb)
        #if sim:
        #    rospy.Subscriber("/"+robot_name+"/"+"theta", Float64, self.sim_theta_cb)
        # Publisher
        self.pub_cmd_vel = rospy.Publisher("/"+robot_name+'/cmd_vel', Twist,queue_size = 1,latch=False)
        self.pub_marker_line = rospy.Publisher("/"+robot_name+'/marker_line', MarkerArray,queue_size = 1,latch=False)
        # Parameters
        self.robot_name = robot_name
        self.role = role
        self.sim = sim
        # Tf listner
        self.tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tfBuffer)
        self.base_link_xyt = None # (x,y,theta)
        self.big_car_xyt = None  # (x,y,theta)
        self.theta = None
        # Kinematics 
        self.Vc = None
        self.Vy = None
        self.Wc = None
        self.ref_ang = None
        # Output 
        self.v_out = None
        self.w_out = None
        # Flags
        self.mode = "diff" # "crab"
        self.is_need_publish = False
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
        if data.angular.y == 0:
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
    
    def nearest_error(self, error):# TODO merge with normalize angle?
        '''
        Make error inside range [-pi, pi]
        '''
        if error > pi:
            error -= 2*pi
        elif error < -pi:
            error += 2*pi
        return error
    
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
        self.sum_term = kp*ki*DT*error
        cmd = kp*error + self.sum_term
        return cmd
    
    def crab_controller(self,vx,vy,error):
        '''
        Return leader crab controller result
        '''
        v_out = self.sign(vx) * sqrt(vx**2 + vy**2) * abs(cos(error))
        # w = KP_crab*error
        w_out = self.pi_controller(KP_crab, KI, error)
        return (v_out, w_out)
    
    def diff_controller(self,vx,wz,error):
        '''
        Return leader crab controller result
        '''
        R = self.get_radius_of_rotation(vx, wz)
        if R == float("inf"):
            v_con = vx
            w_con = self.pi_controller(KP_diff, KI, error)
        else:
            v_con = (sqrt(R**2 + (TOW_CAR_LENGTH/2.0)**2)*wz) *abs(cos(error))
            if not self.is_same_sign(vx,v_con):
                v_con *= -1
            if vx >= 0:
                w_con = wz*abs(cos(error)) + self.pi_controller(KP_diff, KI, error)
            else:
                w_con = -wz*abs(cos(error)) + self.pi_controller(KP_diff, KI, error)
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
        '''
        # Do nothing if command is not set yet.
        if  self.Vc == None or\
            self.Vy == None or\
            self.Wc == None or\
            self.ref_ang == None:
            return

        # Get tf
        if not self.sim:
            self.base_link_xyt = self.get_tf(self.robot_name + "/map",
                                 self.robot_name + "/base_link")
            self.big_car_xyt   = self.get_tf(self.robot_name + "/map",
                                 self.robot_name + "/center_big_car")
        else:
            self.base_link_xyt = self.get_tf("map", self.robot_name)
            self.big_car_xyt   = self.get_tf("map", "base_link")
        if self.base_link_xyt == None or self.big_car_xyt == None: # If tf is invalid
            return
        
        # Get current theta
        self.theta = self.normalize_angle(self.normalize_angle(self.base_link_xyt[2])
                                        - self.normalize_angle(self.big_car_xyt[2]))
        
        # Get refenrence angle
        if self.mode == "diff":
            # Get radius
            R = self.get_radius_of_rotation(self.Vc, self.Wc)
            if R == float("inf"):
                self.ref_ang = 0

            self.ref_ang = atan2(TOW_CAR_LENGTH/2.0, abs(R))
            if R >= 0: # Center is at LHS
                if self.Vc >= 0: #  Go forward
                    pass 
                else: # Go backward
                    self.ref_ang *= -1
            else: # Center is at RHS
                if self.Vc >= 0: #  Go forward
                    self.ref_ang *= -1
                else: # Go backward
                    pass
                        
        elif self.mode == "crab":
            if self.Vc >= 0: # Go forward
                self.ref_ang = atan2(self.Vy, self.Vc)
            elif self.Vc < 0: # Go backward
                self.ref_ang = self.normalize_angle(atan2(self.Vy, self.Vc) + pi)
        
        # Calculate error of angle
        if self.role == "leader":
            pass
        elif self.role == "follower":
            if self.mode == "crab":
                self.ref_ang += pi
            elif self.mode == "diff":
                self.ref_ang = pi - self.ref_ang
        error_theta = self.nearest_error(self.ref_ang - self.theta)
        
        # Get v_out, w_out
        if self.mode == "crab":
            (self.v_out, self.w_out) = self.crab_controller(self.Vc, self.Vy, error_theta)
        elif self.mode == "diff":
            (self.v_out, self.w_out) = self.diff_controller(self.Vc, self.Wc, error_theta)

        # Reverse follower heading velocity
        if self.role == "follower":
            self.v_out *= -1.0
        
        # Saturation velocity, for safty
        self.v_out = self.saturation(self.v_out, V_MAX)
        self.w_out = self.saturation(self.w_out, W_MAX)
        
        # Set marker line
        # Reference ang
        p1 = self.base_link_xyt[:2]
        p2 = (p1[0] + TOW_CAR_LENGTH/2.0 * cos(self.ref_ang + self.big_car_xyt[2]),
              p1[1] + TOW_CAR_LENGTH/2.0 * sin(self.ref_ang + self.big_car_xyt[2]))
        set_line([p1, p2], RGB = (255,138,189), size = 0.02 ,id = 0)

        # Current ang
        p2 = (p1[0] + TOW_CAR_LENGTH/2.0 * cos(self.base_link_xyt[2]),
              p1[1] + TOW_CAR_LENGTH/2.0 * sin(self.base_link_xyt[2]))
        set_line([p1, p2], RGB = (255,0,0), size = 0.02 ,id = 1)
        
        # Set publish flag
        self.is_need_publish = True

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
            t = self.tfBuffer.lookup_transform(frame_id, child_frame_id, rospy.Time())
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
    rospy.init_node('rap_controller',anonymous=False)
    # Get global parameters
    ROBOT_NAME = rospy.get_param(param_name="~robot_name", default="car1")
    ROLE = rospy.get_param(param_name="~role", default="leader")
    SIM  = rospy.get_param(param_name="~sim", default="false")
    REVERSE_OMEGA = rospy.get_param(param_name="~reverse_omega", default="false")
    
    # Init naive controller
    rap_controller = Rap_controller(ROBOT_NAME, ROLE,SIM)
    
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        rap_controller.run_once()
        if  rap_controller.is_need_publish and\
            rap_controller.v_out != None   and\
            rap_controller.w_out != None:
            cmd_vel = Twist()
            cmd_vel.linear.x  = rap_controller.v_out
            cmd_vel.angular.z = rap_controller.w_out
            if REVERSE_OMEGA: # This is for weird simulation bug
                cmd_vel.angular.z = -cmd_vel.angular.z
            
            rap_controller.pub_cmd_vel.publish(cmd_vel)
            # Publish marker, for debug
            rap_controller.pub_marker_line.publish(MARKER_LINE)
            # Debug print
            rospy.loginfo(rap_controller.role + " : V=" + str(round(rap_controller.v_out, 3)) +
                                                 ", W=" +str(round(rap_controller.w_out, 3)))
            rap_controller.is_need_publish = False 
        rate.sleep()

if __name__ == '__main__':
    try:
       main(sys.argv)
    except rospy.ROSInterruptException:
        pass
