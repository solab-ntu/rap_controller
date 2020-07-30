#!/usr/bin/env python
import rospy
import sys
import tf2_ros
import tf # conversion euler
from geometry_msgs.msg import Twist # topic /cmd_vel
from math import atan2,acos,sqrt,pi,sin,cos,tan

#########################
### Global parameters ###
#########################
TOW_CAR_LENGTH = 0.93 # meter, Length between two cars
V_MAX = 0.3 # m/s, Max velocity
W_MAX = 0.6 # rad/s, MAX angular velocity
KP_crab = 0.8 # KP for crab mode, the bigger the value, the faster it will chase ref_ang
KP_diff = 1.5 # KP fro diff mode
DT = 0.1 # sec

# variable 

class Navie_controller():
    def __init__(self,robot_name, role):
        # Subscriber
        rospy.Subscriber("naive_cmd", Twist, self.cmd_cb)
        # Publisher
        self.pub_cmd_vel = rospy.Publisher("/"+robot_name+'/cmd_vel', Twist,queue_size = 1,latch=False)
        # Parameters
        self.robot_name = robot_name
        self.role = role
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
        self.V_leader = None
        self.W_leader = None
        self.V_follower = None
        self.W_follower = None
        # Flags
        self.mode = "crab" # "diff"
        self.is_need_publish = False
        # PID
        self.cmd_last = 0
        self.error_last = 0 
    
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
            self.mode = "crab"
        else: 
            self.mode = "diff"
    
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
        cmd = self.cmd_last + error*kp + (kp-ki*DT)*self.error_last
        self.cmd_last = cmd
        self.error_last = error
        return cmd
    
    def crab_controller(self,vx,vy,error):
        '''
        Return leader crab controller result
        '''
        v = self.sign(vx) * sqrt(vx**2 + vy**2) * abs(cos(error))
        w = KP_crab*error
        # w = self.pi_controller(KP_crab, 0.001, error)
        return (v,w)
    
    def diff_controller(self,vx,w,R,error):
        '''
        Return leader crab controller result
        '''
        v_out = (vx - sqrt(R**2 + (TOW_CAR_LENGTH/2.0)**2)*w) *abs(cos(error))
        if abs(error) > 0.2617993877991494:
            w_out = KP_diff*error
            # w = self.pi_controller(KP_diff, 0.001, error)
        else:
            # w = w*abs(cos(error)) + self.pi_controller(KP_diff, 0.001, error)
            w_out = w*abs(cos(error)) + KP_diff*error
        return (v_out,w_out)
    
    def iterate_once(self):
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
        self.get_base_link()
        self.get_big_car()
        if self.base_link_xyt == None or self.big_car_xyt == None: # If tf is invalid
            return
        #
        self.theta = self.normalize_angle(self.normalize_angle(self.base_link_xyt[2]) - self.normalize_angle(self.big_car_xyt[2]))
       
        # Get refenrence angle
        if self.mode == "diff":
            try: 
                R = self.Vc / self.Wc
            except ZeroDivisionError:
                R = 99999
                self.ref_ang = 0
            else:
                self.ref_ang = -atan2(TOW_CAR_LENGTH/2.0, R)
        elif self.mode == "crab":
            if self.Vc >= 0: # Go forward
                self.ref_ang = atan2(self.Vy, self.Vc)
            elif self.Vc < 0: # Go backward
                self.ref_ang = self.normalize_angle(atan2(self.Vy, self.Vc) + pi)
        
        # Leader
        error_leader = self.nearest_error(self.ref_ang - self.theta)
        if self.mode == "crab":
            (self.V_leader, self.W_leader) = self.crab_controller(self.Vc, self.Vy, error_leader)

        elif self.mode == "diff":
            (self.V_leader, self.W_leader) = self.diff_controller(self.Vc, self.Wc, R,error_leader)
            #self.V_leader = (self.Vc - sqrt(R**2 + (TOW_CAR_LENGTH/2.0)**2)*self.Wc) *abs(cos(error_leader))
            #if abs(error_leader) > 0.2617993877991494:
            #    self.W_leader = KP_diff*error_leader
            #else:
            #    self.W_leader = self.Wc*abs(cos(error_leader)) + KP_diff*error_leader

        # Follower
        if self.mode == "crab":
            error_follower = self.nearest_error(pi + self.ref_ang - self.theta)
            (self.V_follower, self.W_follower) = self.crab_controller(self.Vc, self.Vy, error_follower)
            #self.V_follower = self.sign(self.Vc) * -sqrt(self.Vc**2 + self.Vy**2) * abs(cos(error_follower))
            #self.W_follower = KP_crab*error_follower
        elif self.mode == "diff":
            error_follower = self.nearest_error(pi - self.ref_ang - self.theta)
            (self.V_follower, self.W_follower) = self.diff_controller(self.Vc, self.Wc, R,error_follower)
            #self.V_follower = -( self.Vc - sqrt(R**2 + (TOW_CAR_LENGTH/2.0)**2)*self.Wc) * abs(cos(error_follower) )
            #if abs(error_leader) > 0.2617993877991494:
            #    self.W_follower = KP_diff*error_follower
            #else:
            #    self.W_follower =  self.Wc*abs( cos(error_follower)) + KP_diff*error_follower
        
        self.V_follower *= -1.0
        # Saturation velocity, for safty
        self.V_leader = self.saturation(self.V_leader, V_MAX)
        self.W_leader = self.saturation(self.W_leader, W_MAX)
        self.V_follower = self.saturation(self.V_follower, V_MAX)
        self.W_follower = self.saturation(self.W_follower, W_MAX)
        
        # Debug print
        # rospy.loginfo("[naive controller] W_Leader = "+str(KP)+"*(" + str(self.ref_ang) + " - " + str(self.theta))
        if self.role == "leader":
            rospy.loginfo("[naive controller] Error_leader: "+ str(error_leader)+", ref_ang: " + str(self.ref_ang) + ", theta:" + str(self.theta))
            rospy.loginfo("[naive controller] Leader: "+ str(error_leader)+" (" + str(self.V_leader) + ", " +str(self.W_leader) + ")")
        elif self.role == "follower":
            rospy.loginfo("[naive controller] Follower: "+ str(error_follower)+" (" + str(self.V_follower) + ", " +str(self.W_follower) + ")")
        
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
    
    def get_base_link(self):
        '''
        Get tf, map -> base_link 
        '''
        try:
            t = self.tfBuffer.lookup_transform(self.robot_name + "/map", self.robot_name + "/base_link", rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("[rap_controller] Can't get tf frame: " + self.robot_name + "/map -> " + self.robot_name + "/base_link")
        else:
            quaternion = (
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            self.base_link_xyt = (t.transform.translation.x, t.transform.translation.y, euler[2])
    
    def get_big_car(self):
        '''
        Get tf, map -> center_big_car 
        '''
        try:
            t = self.tfBuffer.lookup_transform(self.robot_name + "/map", self.robot_name + "/center_big_car", rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("[rap_controller] Can't get tf frame: " + self.robot_name + "/map -> " + self.robot_name + "/center_big_car")
        else:
            quaternion = (
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            self.big_car_xyt = (t.transform.translation.x, t.transform.translation.y, euler[2])

def main(args):
    rospy.init_node('rap_controller',anonymous=False)
    # Get global parameters
    robot_name = rospy.get_param(param_name="~robot_name", default="car1")
    role = rospy.get_param(param_name="~role", default="leader")
    '''
    is_parameters_set = False
    
    while (not is_parameters_set and not rospy.is_shutdown() ):
        try:
            robot_name = rospy.get_param("/unique_parameter/robot_name") # Find paramters in ros server
            role       = rospy.get_param("/unique_parameter/role") # Find paramters in ros server
            is_parameters_set = True
        except:
            rospy.logwarn("[rap controller] robot_name are not found in rosparam server, keep on trying...")
            rospy.sleep(0.2) # Sleep 0.2 seconds for waiting the parameters loading
            continue
    '''
    # Init naive controller
    navie_controller = Navie_controller(robot_name, role)
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        navie_controller.iterate_once()
        if navie_controller.is_need_publish:
            if role == "leader" and navie_controller.V_leader != None and navie_controller.W_leader != None:
                cmd_vel = Twist()
                cmd_vel.linear.x  = navie_controller.V_leader
                cmd_vel.angular.z = navie_controller.W_leader
                navie_controller.pub_cmd_vel.publish(cmd_vel)
            elif role == "follower" and navie_controller.V_follower != None and navie_controller.W_follower != None:
                cmd_vel = Twist()
                cmd_vel.linear.x  = navie_controller.V_follower
                cmd_vel.angular.z = navie_controller.W_follower
                navie_controller.pub_cmd_vel.publish(cmd_vel)
            navie_controller.is_need_publish = False 
        rate.sleep()

if __name__ == '__main__':
    try:
       main(sys.argv)
    except rospy.ROSInterruptException:
        pass
