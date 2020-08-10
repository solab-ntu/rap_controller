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
from nav_msgs.msg import Path 

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
LOOK_AHEAD_DIST = 0.5 # Look ahead distance

class Rap_controller():
    def __init__(self):
        # Subscriber
        rospy.Subscriber("/move_base/GlobalPlanner/plan", Path, self.path_cb)
        self.global_path = None #
        # Publisher
        self.pub_cmd_vel_leader   = rospy.Publisher(CMD_TOPIC_LEADER, Twist,queue_size = 1,latch=False)
        self.pub_cmd_vel_follower = rospy.Publisher(CMD_TOPIC_FOLLOWER, Twist,queue_size = 1,latch=False)
        self.pub_marker_line = rospy.Publisher("/rap_angle_marker_line", MarkerArray,queue_size = 1,latch=False)
        self.pub_marker_point = rospy.Publisher("/local_goal", MarkerArray,queue_size = 1,latch=False)
        self.pub_global_path = rospy.Publisher("/rap_planner/global_path", Path,queue_size = 1,latch=False)
        # Parameters
        self.dt = 1.0 / CONTROL_FREQ
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
        self.ref_ang_leader   = 0
        self.ref_ang_follower = 0
        # Output 
        self.v_out_leader = None
        self.w_out_leader = None
        self.v_out_follower = None
        self.w_out_follower = None 
        # PID
        self.cmd_last = 0
        self.error_last = 0 
        self.sum_term = 0
    
    def path_cb(self, data):
        '''
        std_msgs/Header header
            uint32 seq
            time stamp
            string frame_id
        geometry_msgs/PoseStamped[] poses
            std_msgs/Header header
                uint32 seq
                time stamp
                string frame_id
            geometry_msgs/Pose pose
                geometry_msgs/Point position
                float64 x
                float64 y
                float64 z
                geometry_msgs/Quaternion orientation
                float64 x
                float64 y
                float64 z
                float64 w
        '''
        self.global_path = data

    def get_local_goal(self):
        '''
        Return (x,y)
        '''

        if self.big_car_xyt == None or self.global_path == None:
            return 
        # Find a local goal on global_path
        min_d_dist = float("inf")
        local_goal = None # (x,y)
        for pose in self.global_path.poses:
            dx = pose.pose.position.x - self.big_car_xyt[0]
            dy = pose.pose.position.y - self.big_car_xyt[1]
            d_dist = abs(dx**2 + dy**2 - LOOK_AHEAD_DIST**2)
            if d_dist < min_d_dist:
                local_goal = (pose.pose.position.x, pose.pose.position.y)
                min_d_dist = d_dist
        return local_goal
    
    def prune_global_path(self):
        '''
        Return T/F, True: prune successfully, False: can't prune path
        '''
        if self.big_car_xyt == None or self.global_path == None:
            return False
        # Find point on global_path that nearest to base_link
        min_d_dist = float("inf")
        prune_point = None
        for idx in range(len(self.global_path.poses)):
        # for pose in self.global_path.poses:
            dx = self.global_path.poses[idx].pose.position.x - self.big_car_xyt[0]
            dy = self.global_path.poses[idx].pose.position.y - self.big_car_xyt[1]
            d_dist = dx**2 + dy**2
            if d_dist < min_d_dist:
                prune_point = idx# (pose.pose.position.x, pose.pose.position.y)
                min_d_dist = d_dist
        
        self.global_path.poses = self.global_path.poses[prune_point:]
        return True
        

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
        t_base_link = self.get_tf(MAP_FRAME, BASE_LINK_FRAME)
        t_big_car   = self.get_tf(MAP_FRAME, BIG_CAR_FRAME)

        if t_base_link != None:
            self.base_link_xyt = t_base_link
        if t_big_car != None:
            self.big_car_xyt = t_big_car
        
        if self.base_link_xyt == None or self.big_car_xyt == None: #tf is invalid
            return False
        
        # TODO need to prune global path
        self.prune_global_path()
        # Get Local goal
        local_goal = self.get_local_goal()
        if local_goal == None:
            rospy.logwarn("[rap_planner] Can't get local goal from global path.")
            return False
        
        
        # transform local goal to /base_link frame
        p_dif = (local_goal[0] - self.base_link_xyt[0],
                 local_goal[1] - self.base_link_xyt[1])
        t = self.base_link_xyt[2]
        x_out = cos(t)*p_dif[0] + sin(t)*p_dif[1]
        y_out =-sin(t)*p_dif[0] + cos(t)*p_dif[1]
        local_goal_base = (x_out, y_out)
        # Debug points
        set_sphere(local_goal_base , BASE_LINK_FRAME, (0,255,255)  , 0.1, 0)
        
        # TODO get self.vc,wc, No get R 
        alpha = atan2(local_goal_base[1], local_goal_base[0])
        print ("Alpha : " + str(alpha))
        R = sqrt( (tan(pi/2 - alpha)*LOOK_AHEAD_DIST/2)**2 + (LOOK_AHEAD_DIST/2.0)**2 )
        
        if alpha < 0: # alpha = [0,-pi]
            R = -R
        
        self.Vc = LOOK_AHEAD_DIST/10.0 # TODO 
        if abs(alpha) < pi/2: # Go forward
            pass
        else: # Go backward
            self.Vc *= -1.0
        self.Wc = self.Vc / R

        # Get current theta
        self.theta = self.normalize_angle(self.normalize_angle(self.base_link_xyt[2])
                                         -self.normalize_angle(self.big_car_xyt[2]))
        
        #################
        ### DIFF MODE ###
        #################
        # Get refenrence angle
        # R = self.get_radius_of_rotation(self.Vc, self.Wc)
        self.ref_ang_leader = atan2(TOW_CAR_LENGTH/2.0, abs(R))
        if not self.is_same_sign(R, self.Vc):
            self.ref_ang_leader *= -1
        # if self.Vc == 0:# In-place rotation
        
        if abs(R) < 0.1:  # TODO 
            # Choose a nearest ref_ang to prsue, two possibility: (-pi/2, pi/2)
            if  abs(self.ref_ang_leader - self.theta) > abs(-self.ref_ang_leader - self.theta) and\
                self.theta < -0.2:
                self.ref_ang_leader *= -1
        
        # Get error
        self.ref_ang_follower = self.normalize_angle(pi - self.ref_ang_leader)
        error_theta_leader   = self.normalize_angle(self.ref_ang_leader - self.theta)
        error_theta_follower = self.normalize_angle(self.ref_ang_follower - self.theta)

        # Get v_out, w_out
        (self.v_out_leader, self.w_out_leader) = self.diff_controller(self.Vc,
                                                                      self.Wc,
                                                                      error_theta_leader,
                                                                      self.ref_ang_leader)
        (self.v_out_follower, self.w_out_follower) = self.diff_controller(self.Vc,
                                                                          self.Wc,
                                                                          error_theta_follower,
                                                                          self.ref_ang_follower)
        '''
        if abs(R) < 0.1:  # TODO 
            (self.v_out, self.w_out) = self.rota_controller(self.Wc, error_theta, self.ref_ang)
        else:
            (self.v_out, self.w_out) = self.diff_controller(self.Vc, self.Wc, error_theta)
        '''
        # Reverse follower heading velocity
        self.v_out_follower = -self.v_out_follower
        
        '''
        #################
        ### CRAB MODE ###
        #################
        # Get refenrence angle
        if self.Vc >= 0: # Go forward
            self.ref_ang = atan2(self.Vy, self.Vc)
        elif self.Vc < 0: # Go backward
            self.ref_ang = self.normalize_angle(atan2(self.Vy, self.Vc) + pi)
        
        # Get error
        if ROLE == "follower":
            self.ref_ang += pi
        error_theta = self.normalize_angle(self.ref_ang - self.theta)

        # Get v_out, w_out
        (self.v_out, self.w_out) = self.crab_controller(self.Vc, self.Vy, error_theta)
        
        # Reverse follower heading velocity
        if ROLE == "follower":
            self.v_out = -self.v_out
        '''
        # Saturation velocity, for safty
        self.v_out_leader = self.saturation(self.v_out_leader, V_MAX)
        self.w_out_leader = self.saturation(self.w_out_leader, W_MAX)
        self.v_out_follower = self.saturation(self.v_out_follower, V_MAX)
        self.w_out_follower = self.saturation(self.w_out_follower, W_MAX)
        print ("V=" + str(self.v_out_leader)+ ", W=" + str(self.w_out_leader))
        # Set marker line
        # Reference ang
        p1 = self.base_link_xyt[:2]
        p2 = (p1[0] + TOW_CAR_LENGTH/1.5 * cos(self.ref_ang_leader + self.big_car_xyt[2]),
              p1[1] + TOW_CAR_LENGTH/1.5 * sin(self.ref_ang_leader + self.big_car_xyt[2]))
        set_line([p1, p2], RGB = (0,0,255), size = 0.03 ,id = 0)
        # Current ang
        p2 = (p1[0] + TOW_CAR_LENGTH/1.5 * cos(self.base_link_xyt[2]),
              p1[1] + TOW_CAR_LENGTH/1.5 * sin(self.base_link_xyt[2]))
        set_line([p1, p2], RGB = (102,178,255), size = 0.03 ,id = 1)
        # Follower
        '''
        # Reference ang
        p1 = self.base_link_xyt[:2]
        p2 = (p1[0] - TOW_CAR_LENGTH/1.5 * cos(self.ref_ang_follower + self.big_car_xyt[2]),
              p1[1] - TOW_CAR_LENGTH/1.5 * sin(self.ref_ang_follower + self.big_car_xyt[2]))
        set_line([p1, p2], RGB = (0,0,255), size = 0.03 ,id = 2)
        # Current ang
        p2 = (p1[0] - TOW_CAR_LENGTH/1.5 * cos(self.base_link_xyt[2]),
              p1[1] - TOW_CAR_LENGTH/1.5 * sin(self.base_link_xyt[2]))
        set_line([p1, p2], RGB = (102,178,255), size = 0.03 ,id = 3)
        '''
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
def set_line(points, RGB = None , size = 0.2, id = 0):
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
    marker.header.frame_id = MAP_FRAME
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

def set_sphere(point, frame_id , RGB = None  , size = 0.05, id = 0):
    '''
    Set Point at MarkArray 
    Input : 
        point - (x,y)
        RGB - (r,g,b)
    '''
    global MARKER_POINT
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.id = id
    marker.ns = "tiles"
    marker.header.stamp = rospy.get_rostime()
    marker.type = marker.SPHERE
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
    (marker.pose.position.x , marker.pose.position.y) = point
    MARKER_POINT.markers.append(marker)

if __name__ == '__main__':
    rospy.init_node('rap_planner',anonymous=False)
    # Get launch file parameters
    SIM           = rospy.get_param(param_name="~sim", default="false")
    REVERSE_OMEGA = rospy.get_param(param_name="~reverse_omega", default="false")
    CONTROL_FREQ  = rospy.get_param(param_name="~ctl_frequency", default="10")
    MAP_FRAME     = rospy.get_param(param_name="~map_frame", default="map")
    BASE_LINK_FRAME = rospy.get_param(param_name="~base_link_frame", default="base_link")
    BIG_CAR_FRAME   = rospy.get_param(param_name="~big_car_frame", default="big_car")
    CMD_TOPIC_LEADER   = rospy.get_param(param_name="~cmd_topic_leader", default="/car1/cmd_vel")
    CMD_TOPIC_FOLLOWER = rospy.get_param(param_name="~cmd_topic_follower", default="/car2/cmd_vel")
    # Global variable
    MARKER_LINE = MarkerArray()# Line markers show on RVIZ
    MARKER_POINT = MarkerArray()# Line markers show on RVIZ
    # Init naive controller
    rap_controller = Rap_controller()
    
    rate = rospy.Rate(CONTROL_FREQ)
    while not rospy.is_shutdown():
        if rap_controller.run_once():
            # Leader cmd vel 
            cmd_vel = Twist()
            cmd_vel.linear.x  = rap_controller.v_out_leader
            cmd_vel.angular.z = rap_controller.w_out_leader
            if REVERSE_OMEGA: # This is for weird simulation bug
                cmd_vel.angular.z = -cmd_vel.angular.z
            rap_controller.pub_cmd_vel_leader.publish(cmd_vel)

            # Follower cmd vel 
            cmd_vel_follower = Twist()
            cmd_vel_follower.linear.x  = rap_controller.v_out_follower
            cmd_vel_follower.angular.z = rap_controller.w_out_follower
            if REVERSE_OMEGA: # This is for weird simulation bug
                cmd_vel_follower.angular.z = -cmd_vel_follower.angular.z

            rap_controller.pub_cmd_vel_follower.publish(cmd_vel_follower)
            # Publish marker, for debug
            rap_controller.pub_marker_line.publish(MARKER_LINE)
            rap_controller.pub_marker_point.publish(MARKER_POINT)
            # Debug print
            # rospy.logdebug(ROLE + " : V=" + str(round(rap_controller.v_out, 3)) +
            #                       ", W=" + str(round(rap_controller.w_out, 3)))
            rap_controller.pub_global_path.publish(rap_controller.global_path)
            MARKER_LINE = MarkerArray()
            MARKER_POINT = MarkerArray()
        rate.sleep()
