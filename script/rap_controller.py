#!/usr/bin/env python
import rospy
import sys
import tf2_ros
import tf # conversion euler
from math import atan2,acos,sqrt,pi,sin,cos,tan
from std_msgs.msg import Float64
from geometry_msgs.msg import Point, Twist# topic /cmd_vel
from lucky_utility.ros.rospy_utility import get_tf,normalize_angle,\
                                            sign,is_same_sign, Marker_Manager, send_tf
from visualization_msgs.msg import MarkerArray
import time # for testing
#########################
### Global parameters ###
#########################
TOW_CAR_LENGTH = 0.93 # meter, Length between two cars
V_MAX = 0.3 # m/s, Max velocity
W_MAX = 0.8 # rad/s, MAX angular velocity
KP_crab = 0.8 # KP for crab mode, the bigger the value, the faster it will chase ref_ang
KP_diff = 1.5 # KP fro diff mode
KP_rota_abs = 5.0
KI = 0

# If Radius small then this value, switch to rota controller
INPLACE_ROTATION_R = 0.1  # meter
# How precise transition it needs to be.
TRANSITION_ANG_TOLERANCE = 10 # degree
TRANSITION_ANG_TOLERANCE *= pi/180.0

ROTA_ABS_TOLERANCE = 5 # degree
ROTA_ABS_TOLERANCE *= pi/180.0
# Crab dead zone, car1, car2 heading can't go here
DEAD_ZONE_ANG = pi/2

class Rap_controller():
    def __init__(self, 
                 robot_name,
                 # role,
                 sim, 
                 control_freq, 
                 reverse_omega,
                 # tf frame id 
                 map_frame, 
                 map_peer_frame,
                 base_link_frame_leader,
                 base_link_frame_follow,
                 big_car_frame_leader, 
                 big_car_frame_follow,
                 # Topic name
                 cmd_vel_topic_leader,
                 cmd_vel_topic_follow):
        # Store argument
        self.robot_name = robot_name
        # fself.role = role
        self.sim = sim
        self.control_freq = control_freq
        self.reverse_omega = reverse_omega
        # Tf frame id 
        self.map_frame = map_frame
        self.map_peer_frame = map_peer_frame
        self.base_link_frame_leader = base_link_frame_leader
        self.base_link_frame_follow = base_link_frame_follow
        self.big_car_frame_leader   = big_car_frame_leader
        self.big_car_frame_follow   = big_car_frame_follow
        # Subscriber
        rospy.Subscriber("/"+robot_name+"/"+"rap_cmd", Twist, self.cmd_cb)
        # Publisher
        self.pub_cmd_vel_leader = rospy.Publisher(cmd_vel_topic_leader, Twist,queue_size = 1,latch=False)
        self.pub_cmd_vel_follow = rospy.Publisher(cmd_vel_topic_follow, Twist,queue_size = 1,latch=False)
        # Parameters
        self.dt = 1.0 / self.control_freq
        # Tf listner
        '''
        self.tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tfBuffer)
        self.base_link_xyt_L = None # (x,y,theta)
        self.base_link_xyt_F = None # (x,y,theta)
        self.big_car_xyt_L = None  # (x,y,theta)
        self.big_car_xyt_F = None  # (x,y,theta)
        '''
        rospy.Subscriber("/car1/theta", Float64, self.theta_car1_cb)
        rospy.Subscriber("/car2/theta", Float64, self.theta_car2_cb)
        self.theta_L = None
        self.theta_F = None
        # Kinematics 
        self.Vx = 0
        self.Vy = 0
        self.Wz = 0
        # self.ref_ang = 0
        # Output 
        self.v_out_L = None
        self.w_out_L = None
        self.v_out_F = None
        self.w_out_F = None
        # Flags
        self.mode = "diff"
        self.is_transit = False
        self.next_mode = None
        # PID
        self.cmd_last = 0
        self.error_last = 0 
        self.sum_term = 0
        # Markers
        self.viz_mark = Marker_Manager("/rap_controller/markers")
        
        self.viz_mark.register_marker("leader_ref", 4, "/carB/base_link",# self.big_car_frame_leader,
                                      RGB = (0,0,255), size = 0.03)
        self.viz_mark.register_marker("leader_cur", 4, "/car1/base_link",# self.base_link_frame_leader,
                                      RGB = (102,178,255), size = 0.03)
        self.viz_mark.register_marker("follow_ref", 4, "/carB/base_link",# self.big_car_frame_leader,
                                      RGB = (0,0,255), size = 0.03)
        self.viz_mark.register_marker("follow_cur", 4, "/carB/base_link",# self.big_car_frame_leader,
                                      RGB = (102,178,255), size = 0.03)
    
    def theta_car1_cb(self, data):
        '''
        '''
        self.theta_L = data.data

    def theta_car2_cb(self, data):
        '''
        '''
        self.theta_F = data.data
        send_tf((-TOW_CAR_LENGTH/2.0, 0, data.data) ,"carB/base_link", "car2/base_link")
    
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
                float64 x
                float64 y - 1: crab , 0: diff, 2:rota
                float64 z
        '''
        if data.angular.y == 1.0: # Crab mode
            self.set_cmd(data.linear.x, data.linear.y, 0.0, "crab")
        elif data.angular.y == 0.0: # Diff mode
            self.set_cmd(data.linear.x, 0.0, data.angular.z, "diff")
        elif data.angular.y == 2.0: # Rota mode
            self.set_cmd(0.0, 0.0, data.angular.z, "rota")
        elif data.angular.y == 3.0: # XX->Rota mode
            self.next_mode = "rota"
            self.set_cmd(0.0, 0.0, 0.1, "tran")
    
    def set_cmd(self, vx, vy, wz, mode):
        '''
        Set cmd come from rap_planner
        '''
        self.Vx = vx
        self.Vy = vy
        self.Wz = wz
        self.mode = mode

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

    def head_controller(self,error):
        '''
        big car don't move, only adjust car1 car2 heading
        '''
        v_con = 0.0
        w_con = self.pi_controller(KP_crab, KI, error)
        return (v_con, w_con)

    def crab_controller(self,vx,vy,error,is_forward):
        '''
        Return leader crab controller result
        '''
        # v_con = sign(vx) * sqrt(vx**2 + vy**2) * abs(cos(error))
        v_con = sqrt(vx**2 + vy**2) * abs(cos(error))
        if not is_forward:
            v_con *= -1.0
        w_con = self.pi_controller(KP_crab, KI, error)
        return (v_con, w_con)
    
    def diff_controller(self,vx,wz,error,ref_ang):
        '''
        Return leader crab controller result
        '''
        R = self.get_radius_of_rotation(vx, wz)
        if R == float("inf"):# Go straight
            v_con = vx
            w_con = self.pi_controller(KP_diff, KI, error)
        else:
            if abs(R) < INPLACE_ROTATION_R:
                (v_con, w_con) = self.rota_controller(wz, error, ref_ang)
            else:
                v_con = (sqrt(R**2 + (TOW_CAR_LENGTH/2.0)**2)*wz) *abs(cos(error))
                if not is_same_sign(vx,v_con):
                    v_con *= -1
                w_con = sign(vx)*wz*abs(cos(error)) + self.pi_controller(KP_diff, KI, error)
        return (v_con, w_con)
    
    def rota_controller(self,wz,error,ref_ang):
        '''
        Inplace rotation controller
        '''

        ''' stupid 
        if abs(error) > ROTA_ABS_TOLERANCE/2:
            # ABS
            v_con = 0.0 #  (TOW_CAR_LENGTH/2.0)*wz*abs(cos(error))
            w_con = self.pi_controller(KP_rota_abs, KI, error)
        else:
        '''
        #v_con = (TOW_CAR_LENGTH/2.0)*wz*abs(cos(error))
        #w_con = wz*abs(cos(error)) + self.pi_controller(KP_diff, KI, error)
        '''
        WHEEL_RADIUS =  0.075
        WHEEL_SEPERATE_L = 0.33
        VEL_TOLERANCE = 0.3        
        v_left  = (v_con - w_con * WHEEL_SEPERATE_L / 2.0) / WHEEL_RADIUS
        v_right = (v_con + w_con * WHEEL_SEPERATE_L / 2.0) / WHEEL_RADIUS
        if (abs(v_left) < VEL_TOLERANCE or abs(v_right) < VEL_TOLERANCE ) and wz != 0:
            print ("BOOST!!!")
            rospy.loginfo("v_left = " + str(v_left) + ", v_right = " + str(v_right))
            v_con = 0
            w_con = self.pi_controller(5, KI, error)

        # rospy.loginfo("v_left = " + str(v_left) + ", v_right = " + str(v_right))
        '''
        # Better than original 
        ERROR_LIMIT = pi/18.0
        percentage = abs(error) / ERROR_LIMIT
        if percentage >= 1.0:
            v_con = 0
            w_con = self.pi_controller(KP_diff, KI, error) * percentage
            print (str(percentage))
        else:
            v_con = (TOW_CAR_LENGTH/2.0)*wz*abs(cos(error)) * (1-percentage)
            w_con = wz*abs(cos(error)) * (1-percentage) + self.pi_controller(KP_diff, KI, error)
        

        ''' Original 
        v_con = (TOW_CAR_LENGTH/2.0)*wz*abs(cos(error))
        w_con = wz*abs(cos(error)) + self.pi_controller(KP_diff, KI, error)
        '''
        if ref_ang < 0: # ref_ang == -pi/2
            v_con = -v_con
        return (v_con, w_con)

    '''
    def rota_controller_test(self,wz,error,ref_ang, inner_side):
        WHEEL_RADIUS =  0.075  
        WHEEL_SEPERATE_L = 0.33
        if inner_side == "left":
            v_left  = -self.pi_controller(10, KI, error)
            v_right = (TOW_CAR_LENGTH/2.0 + WHEEL_SEPERATE_L/2.0)*wz*10 # *abs(cos(error))
            
        elif inner_side == "right":
            v_left  = -(TOW_CAR_LENGTH/2.0 + WHEEL_SEPERATE_L/2.0)*wz*10 # *abs(cos(error))
            v_right =  self.pi_controller(10, KI, error)
            rospy.loginfo("error = " + str(error))
            rospy.loginfo("v_left = " + str(v_left) + ", v_right = " + str(v_right))
        
        #if ref_ang < 0: # ref_ang == -pi/2
        #    v_left = -v_left
        
        v_con = WHEEL_RADIUS*(v_left + v_right)/2.0
        w_con = WHEEL_RADIUS*(v_right - v_left)/WHEEL_SEPERATE_L

        if inner_side == "right":
            v_con = -v_con

        return (v_con, w_con)
    '''
    def get_radius_of_rotation(self,v,w):
        try:
            radius = v / w
        except ZeroDivisionError:
            return float("inf")
        else:
            return radius 

    def diff_get_error_angle(self):
        '''
        Input:
            self.Vy
            self.Wz
            self.theta
        Return ( (ref_ang, error_theta) , (ref_ang, error_theta) )
                 ^ leader                  ^ follower
        '''    
        # Get refenrence angle
        R = self.get_radius_of_rotation(self.Vx, self.Wz)
        ref_ang_L = atan2(TOW_CAR_LENGTH/2.0, abs(R))
        if not is_same_sign(R, self.Vx):
            ref_ang_L *= -1
        
        # Follower reverse ref_ang
        ref_ang_F = normalize_angle(pi - ref_ang_L)
        
        # Get error theta
        error_theta_L = normalize_angle(ref_ang_L - self.theta_L)
        error_theta_F = normalize_angle(ref_ang_F - self.theta_F)

        # In-place rotation, Find the nearest 90 degree to ref.
        if  abs(R) < INPLACE_ROTATION_R and\
            abs(error_theta_L) > pi/2 and\
            abs(error_theta_F) > pi/2:
            # ref_ang_L and ref_ang_F must have same sign
            ref_ang_L *= -1
            ref_ang_F *= -1
            error_theta_L = normalize_angle(ref_ang_L - self.theta_L)
            error_theta_F = normalize_angle(ref_ang_F - self.theta_F)

        return ((ref_ang_L, error_theta_L), (ref_ang_F, error_theta_F))

    def crab_get_error_angle(self):
        '''
        Input:
            self.Vx
            self.Vy
            self.theta
        Return ( (ref_ang, error_theta) , (ref_ang, error_theta) , T/F)
                 ^ leader                  ^ follower
        '''        
        # Get refenrence angle
        ref_ang_L = atan2(self.Vy, self.Vx)
        ref_ang_F = normalize_angle(ref_ang_L + pi)
        
        # Get error angle
        error_theta_L = normalize_angle(ref_ang_L - self.theta_L)
        error_theta_F = normalize_angle(ref_ang_F - self.theta_F)

        # Go backward or forward? 
        is_forward = True
        if  abs(error_theta_L) > pi/2 and\
            abs(error_theta_F) > pi/2 and\
            (not(self.Vx == 0 and self.Vy == 0)):
            is_forward = False
            ref_ang_L = normalize_angle(ref_ang_L + pi)
            ref_ang_F = normalize_angle(ref_ang_F + pi)
            error_theta_L = normalize_angle(ref_ang_L - self.theta_L)
            error_theta_F = normalize_angle(ref_ang_F - self.theta_F)
        
        return ((ref_ang_L, error_theta_L), (ref_ang_F, error_theta_F), is_forward)

    def run_once(self):
        '''
        call by main loop, execute every loop.
        Return: NO, return ref_ang
            True - Calculate successfully, need publish
            False - Can't finish calculation, don't publish
        '''
        '''
        # Update tf
        t_base_link_L = get_tf(self.tfBuffer, self.map_frame, self.base_link_frame_leader)
        t_base_link_F = get_tf(self.tfBuffer, self.map_peer_frame, self.base_link_frame_follow)
        t_big_car_L   = get_tf(self.tfBuffer, self.map_frame, self.big_car_frame_leader)
        t_big_car_F   = get_tf(self.tfBuffer, self.map_peer_frame, self.big_car_frame_follow)

        if t_base_link_L != None:
            self.base_link_xyt_L = t_base_link_L
        if t_big_car_L != None:
            self.big_car_xyt_L = t_big_car_L
        if t_base_link_F != None:
            self.base_link_xyt_F = t_base_link_F
        if t_big_car_F != None:
            self.big_car_xyt_F = t_big_car_F
        
        if  self.base_link_xyt_L == None or self.big_car_xyt_L == None or\
            self.base_link_xyt_F == None or self.big_car_xyt_F == None: #tf is invalid
            time.sleep(1)
            return False
        '''
        # Use theta Instead of /tf # TODO need test
        if self.theta_L == None or self.theta_F == None:
            rospy.logwarn("[rap_controller] Can't get /car1/theta or /car2/theta")
            time.sleep(1)
            return False
        # Get current theta
        '''
        self.theta_L = normalize_angle(normalize_angle(self.base_link_xyt_L[2])
                                     - normalize_angle(self.big_car_xyt_L[2]))
        self.theta_F = normalize_angle(normalize_angle(self.base_link_xyt_F[2])
                                     - normalize_angle(self.big_car_xyt_F[2]))
        '''
        #########################
        #### Get Errror angle ###
        #########################
        if self.mode == "crab":
            ((ref_ang_L, error_theta_L),
             (ref_ang_F, error_theta_F), is_forward) = self.crab_get_error_angle()

        elif self.mode == "rota":
            ((ref_ang_L, error_theta_L),
             (ref_ang_F, error_theta_F)) = self.diff_get_error_angle()

        elif self.mode == "tran":
            if self.next_mode == "crab":
                ((ref_ang_L, error_theta_L),
                 (ref_ang_F, error_theta_F), _) = self.crab_get_error_angle()
            elif self.next_mode == "diff":
                ((ref_ang_L, error_theta_L),
                (ref_ang_F, error_theta_F)) = self.diff_get_error_angle()
            elif self.next_mode == "rota":
                ((ref_ang_L, error_theta_L),
                (ref_ang_F, error_theta_F)) = self.diff_get_error_angle()
        
        elif self.mode == "diff":
            ((ref_ang_L, error_theta_L),
             (ref_ang_F, error_theta_F)) = self.diff_get_error_angle()

        ####################
        ### Execute Mode ###
        ####################
        if self.mode == "crab":
            # Get v_out, w_out
            (self.v_out_L, self.w_out_L) =  self.crab_controller(
                                            self.Vx, self.Vy, error_theta_L, is_forward)
            (self.v_out_F, self.w_out_F) =  self.crab_controller(
                                            self.Vx, self.Vy, error_theta_F, is_forward)
        elif self.mode == "rota":
            (self.v_out_L, self.w_out_L) =  self.rota_controller(
                                            self.Wz,error_theta_L, ref_ang_L)
            (self.v_out_F, self.w_out_F) =  self.rota_controller(
                                            self.Wz,error_theta_F, ref_ang_F)
            # (self.v_out_L, self.w_out_L) =  self.rota_controller_test(
            #                                 self.Wz,error_theta_L, ref_ang_L, "left")
            # (self.v_out_F, self.w_out_F) =  self.rota_controller_test(
            #                                 self.Wz,error_theta_F, ref_ang_F, "right")
        elif self.mode == "tran":
            if abs(error_theta_L) > (TRANSITION_ANG_TOLERANCE/2.0) or\
               abs(error_theta_F) > (TRANSITION_ANG_TOLERANCE/2.0):
                (self.v_out_L, self.w_out_L) = self.head_controller(error_theta_L)
                (self.v_out_F, self.w_out_F) = self.head_controller(error_theta_F)
            else:
                self.is_transit = False # heading adjust completed
            
        elif self.mode == "diff":
            (self.v_out_L, self.w_out_L) =  self.diff_controller(
                                            self.Vx, self.Wz, error_theta_L, ref_ang_L)
            (self.v_out_F, self.w_out_F) =  self.diff_controller(
                                            self.Vx, self.Wz, error_theta_F, ref_ang_F)
        else:
            rospy.logerr("[rap_planner] Invalid mode " + str(self.mode))
        
        # Reverse follower heading velocity
        self.v_out_F *= -1

        # Saturation velocity, for safty
        self.v_out_L = self.saturation(self.v_out_L, V_MAX)
        self.w_out_L = self.saturation(self.w_out_L, W_MAX)
        self.v_out_F = self.saturation(self.v_out_F, V_MAX)
        self.w_out_F = self.saturation(self.w_out_F, W_MAX)
        
        # Set marker line
        # Leader ref_ang
        p = (0.6*cos(ref_ang_L) + TOW_CAR_LENGTH/2.0
            ,0.6*sin(ref_ang_L))
        self.viz_mark.update_marker("leader_ref", ((TOW_CAR_LENGTH/2,0), p))
        # Leader current ang
        self.viz_mark.update_marker("leader_cur", ((0,0), (0.6,0)))
        p = (0.6*cos(ref_ang_F) - TOW_CAR_LENGTH/2.0
            ,0.6*sin(ref_ang_F))
        self.viz_mark.update_marker("follow_ref", ((-TOW_CAR_LENGTH/2,0), p))
        # Follower current ang
        p = (0.6*cos(self.theta_F) - TOW_CAR_LENGTH/2.0
            ,0.6*sin(self.theta_F))
        self.viz_mark.update_marker("follow_cur", ((-TOW_CAR_LENGTH/2,0), p))
        return True # Enable publish

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

    def publish(self):
        '''
        Publish topics
        '''
        # Publish cmd_vel
        cmd_vel = Twist()
        # Publish Leader cmd
        cmd_vel.linear.x  = self.v_out_L
        cmd_vel.angular.z = self.w_out_L
        if self.reverse_omega: # This is for weird simulation bug
            cmd_vel.angular.z *= -1
        self.pub_cmd_vel_leader.publish(cmd_vel)
        # Publish Follower cmd
        cmd_vel.linear.x  = self.v_out_F
        cmd_vel.angular.z = self.w_out_F
        if self.reverse_omega: # This is for weird simulation bug
            cmd_vel.angular.z *= -1
        self.pub_cmd_vel_follow.publish(cmd_vel)
        
        # Publish marker, for debug
        self.viz_mark.publish()
        
        # Debug print
        rospy.logdebug("Leader" + " : V=" + str(round(self.v_out_L, 3))+
                                   ", W=" + str(round(self.w_out_L, 3)))


if __name__ == '__main__':
    rospy.init_node('rap_controller',anonymous=False)
    # Get launch file parameters
    ROBOT_NAME    = rospy.get_param(param_name="~robot_name", default="car1")
    ROLE          = rospy.get_param(param_name="~role", default="leader")
    SIM           = rospy.get_param(param_name="~sim", default="false")
    REVERSE_OMEGA = rospy.get_param(param_name="~reverse_omega", default="false")
    CONTROL_FREQ  = rospy.get_param(param_name="~ctl_frequency", default="10")
    # TF frame id 
    MAP_FRAME     = rospy.get_param(param_name="~map_frame", default="map")
    MAP_PEER_FRAME = rospy.get_param(param_name="~map_peer_frame", default="map")
    BIG_CAR_FRAME = rospy.get_param(param_name="~big_car_frame", default="/car1/center_big_car")
    BIG_CAR_PEER_FRAME = rospy.get_param(param_name="~big_car_peer_frame", default="/car2/center_big_car")
    BASE_LINK_FRAME = rospy.get_param(param_name="~base_link_frame", default="base_link")
    BASE_PEER_FRAME = rospy.get_param(param_name="~base_peer_frame", default="base_peer")
    # Topic name
    CMD_VEL_TOPIC_LEADER = rospy.get_param(param_name="~cmd_vel_topic_leader", default="/car1/cmd_vel")
    CMD_VEL_TOPIC_FOLLOW = rospy.get_param(param_name="~cmd_vel_topic_follower", default="/car2/cmd_vel")
    
    # Init rap controller
    RAP_CTL = Rap_controller("car1", 
                             SIM,
                             CONTROL_FREQ,
                             REVERSE_OMEGA,
                             # Tf frame id 
                             MAP_FRAME,
                             MAP_PEER_FRAME,
                             BASE_LINK_FRAME,
                             BASE_PEER_FRAME,
                             BIG_CAR_FRAME,
                             BIG_CAR_PEER_FRAME,
                             # Topic name
                             CMD_VEL_TOPIC_LEADER,
                             CMD_VEL_TOPIC_FOLLOW)
    
    rate = rospy.Rate(CONTROL_FREQ)
    while not rospy.is_shutdown():
        if RAP_CTL.run_once():
            RAP_CTL.publish()
        rate.sleep()
