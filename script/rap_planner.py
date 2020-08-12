#!/usr/bin/env python
import rospy
import sys
import tf2_ros
import tf # conversion euler
import random
from math import atan2,acos,sqrt,pi,sin,cos,tan
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker, MarkerArray # Debug drawing
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Path

import time # for testing 
from rap_controller import Rap_controller

#########################
### Global parameters ###
#########################
LOOK_AHEAD_DIST = 0.8 # Look ahead distance
GOAL_TOLERANCE = 0.1 # Consider goal reach if distance to goal is less then GOAL_TOLERANCE
CRAB_REGION = pi/6 # radian
IGNORE_HEADING = True 

class Rap_planner():
    def __init__(self):
        rospy.Subscriber(GLOBAL_PATH_TOPIC, Path, self.path_cb)
        self.global_path = None #
        #self.pub_rap_cmd_car1 = rospy.Publisher("/car1/rap_cmd", Twist,queue_size = 1,latch=False)
        #self.pub_rap_cmd_car2 = rospy.Publisher("/car2/rap_cmd", Twist,queue_size = 1,latch=False)
        self.pub_marker_point = rospy.Publisher("/local_goal", MarkerArray,queue_size = 1,latch=False)
        self.pub_marker_line = rospy.Publisher("/rap_planner/angles", MarkerArray,queue_size = 1,latch=False)
        self.pub_global_path = rospy.Publisher("/rap_planner/global_path", Path,queue_size = 1,latch=False)
        # 
        self.marker_point = MarkerArray()
        self.marker_line = MarkerArray()# Line markers show on RVIZ
        # Output
        self.vx_out = None
        self.vy_out = None
        self.wz_out = None
        self.mode = "diff" # "crab"
        # tf 
        # Tf listner
        self.tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tfBuffer)
        self.big_car_xyt = None 

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

    def get_local_goal(self):
        '''
        Return (x,y,theta)
        if theta == None, then this goal don't have heading demand.
        '''
        if self.big_car_xyt == None or self.global_path == None:
            return None
        # Find a local goal on global_path
        min_d_dist = float("inf")
        local_goal = None # (x,y)
        for pose in self.global_path.poses:
            dx = pose.pose.position.x - self.big_car_xyt[0]
            dy = pose.pose.position.y - self.big_car_xyt[1]
            d_dist = abs(dx**2 + dy**2 - LOOK_AHEAD_DIST**2)
            if d_dist < min_d_dist:
                local_goal = (pose.pose.position.x, pose.pose.position.y, None)
                min_d_dist = d_dist
        
        if local_goal == None:
            return None
        # Get goal heading
        # Currently only adjust heading on last goalstamped
        if sqrt((self.global_path.poses[-1].pose.position.x - self.big_car_xyt[0])**2+
                (self.global_path.poses[-1].pose.position.y - self.big_car_xyt[1])**2) < LOOK_AHEAD_DIST:
            quaternion = (
                self.global_path.poses[-1].pose.orientation.x,
                self.global_path.poses[-1].pose.orientation.y,
                self.global_path.poses[-1].pose.orientation.z,
                self.global_path.poses[-1].pose.orientation.w)
            (_,_,yaw) = tf.transformations.euler_from_quaternion(quaternion)
            local_goal = (local_goal[0], local_goal[1], yaw)

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

    def publish(self):
        '''
        Publish debug thing
        '''
        # Debug Markers
        self.pub_marker_point.publish(self.marker_point)
        self.pub_marker_line.publish(self.marker_line)
        self.marker_line = MarkerArray()
        self.marker_point = MarkerArray()

        # Global path
        if self.global_path != None:
            self.pub_global_path.publish(self.global_path)

    def run_once(self):
        global rap_ctl_leader, rap_ctl_follow # TODO need this?

        # Update tf
        t_big_car   = self.get_tf(MAP_FRAME, BIG_CAR_FRAME)
        if t_big_car != None:
            self.big_car_xyt = t_big_car
        if self.big_car_xyt == None: #tf is invalid
            return False
        
        # Get Local goal
        self.prune_global_path()
        local_goal = self.get_local_goal()
        if local_goal == None:
            rospy.logdebug("[rap_planner] Can't get local goal from global path.")
            return False

        # transform local goal to /base_link frame
        p_dif = (local_goal[0] - self.big_car_xyt[0],
                 local_goal[1] - self.big_car_xyt[1])
        t = self.big_car_xyt[2]
        x_goal = cos(t)*p_dif[0] + sin(t)*p_dif[1]
        y_goal =-sin(t)*p_dif[0] + cos(t)*p_dif[1]

        # Check goal reached 
        if sqrt(x_goal**2 + y_goal**2) < GOAL_TOLERANCE:
            self.vx_out = 0
            self.vy_out = 0
            self.wz_out = 0
            self.global_path = None
            rospy.loginfo("[rap_planner] Goal Reached")
            return True
        
        # Get alpha 
        alpha = atan2(y_goal, x_goal)
        
        # Get beta
        if local_goal[2] != None:
            beta = normalize_angle(local_goal[2] - alpha - self.big_car_xyt[2])
            if abs(alpha) > pi/2:# Go backward
                beta = normalize_angle(beta - pi)
        else:
            beta = 0
        if IGNORE_HEADING:
            beta = 0
        
        # Get pursu_angle
        pursu_angle = alpha + beta

        rospy.loginfo("[rap_planner] Alpha=" + str(round(alpha,3)) + ", Beta=" + str(round(beta,3)))
        
        # Debug markers
        # Local goal
        self.set_sphere((x_goal, y_goal) , BIG_CAR_FRAME, (0,255,255)  , 0.1, 0)
        # pursu_angle
        self.set_sphere((cos(pursu_angle)*LOOK_AHEAD_DIST,
                         sin(pursu_angle)*LOOK_AHEAD_DIST,),
                        BIG_CAR_FRAME, (255,0,255), 0.1, 1)
        
        self.set_line(((0,0), (0.3,0)), BIG_CAR_FRAME,
                      RGB = (255,255,0), size = 0.02, id = 0)
        self.set_line(((0,0), (cos(alpha)*LOOK_AHEAD_DIST, 
                               sin(alpha)*LOOK_AHEAD_DIST,)),
                      BIG_CAR_FRAME, RGB = (255,255,255), size = 0.02, id = 1)
        if local_goal[2] != None:
            self.set_line(((x_goal, y_goal), (x_goal + cos(pursu_angle)*0.3,
                                              y_goal + sin(pursu_angle)*0.3)),
                        BIG_CAR_FRAME, RGB = (255,0,255), size = 0.02, id = 2)
        
        # The circle
        NUM_CIRCLE_POINT = 200
        p_list = [["diff",[(LOOK_AHEAD_DIST,0)]]] # [[["crab",(x,y),...]],[],[],...]
        ang = 0.0
        while ang <= 2*pi:
            belong = ""
            if abs(abs(normalize_angle(ang)) - pi/2) < CRAB_REGION:
                belong = "crab"
            else:
                belong = "diff"
            # Find nearest line-segment to append
            x = LOOK_AHEAD_DIST*cos(ang)
            y = LOOK_AHEAD_DIST*sin(ang)
            is_found = False
            for seg in p_list:
                if  seg[0] == belong and \
                    (seg[1][-1][0] - x)**2 + \
                    (seg[1][-1][1] - y)**2 <= \
                     LOOK_AHEAD_DIST*2*pi/NUM_CIRCLE_POINT:
                     is_found = True
                     seg[1].append((x,y))
            if not is_found: # Creat a new segment
                p_list.append([belong, [(x,y)]])
            ang += 2*pi/NUM_CIRCLE_POINT
        
        # Draw circle
        for idx in range(len(p_list)):
            belong = p_list[idx][0]
            if belong == "diff":
                color = (255,255,0)
            else:
                color = (255,0,0)
            self.set_line(p_list[idx][1], BIG_CAR_FRAME, RGB = color, size = 0.02, id = idx+3)
        
        if abs(abs(alpha) - pi/2) < CRAB_REGION:
            # print ("CRAB MODE")
            self.vx_out = cos(alpha) * KP_VEL
            self.vy_out = sin(alpha) * KP_VEL
            self.wz_out = 0.0

            # Mode decision
            if self.mode == "diff":
                rap_ctl_leader.is_transit = True # Need to check global varibable
                rap_ctl_follow.is_transit = True
                self.mode = "diff->crab"
            elif self.mode == "diff->crab":
                # wait for controller finish trasition
                if (not rap_ctl_leader.is_transit) and (not rap_ctl_follow.is_transit):
                    self.mode = "crab"
            elif self.mode == "crab->diff": # TODO this may cause shaking
                self.mode = "diff->crab"
            elif self.mode == "crab":
                pass
            else:
                rospy.logerr("[rap_planner] Invalid mode " + str(self.mode))
            
        else:
            # print ("DIFF MODE")
            # Get R 
            R = sqrt( (tan(pi/2 - (pursu_angle))*LOOK_AHEAD_DIST/2)**2 +
                    (LOOK_AHEAD_DIST/2.0)**2 )
            # if alpha < 0: # alpha = [0,-pi]
            if pursu_angle < 0: # alpha = [0,-pi]
                R = -R
            
            self.vx_out = sqrt(x_goal**2 + y_goal**2) * KP_VEL
            self.vy_out = 0.0
            self.wz_out = self.vx_out / R
            # if abs(alpha) > pi/2: # Go backward
            if abs(pursu_angle) > pi/2: # Go backward
                self.vx_out *= -1.0


            # Mode decistion
            if self.mode == "diff":
                pass
            elif self.mode == "diff->crab":# TODO this may cause shaking
                self.mode = "crab->diff"
            elif self.mode == "crab->diff": 
                # wait for controller finish trasition
                if (not rap_ctl_leader.is_transit) and (not rap_ctl_follow.is_transit):
                    self.mode = "diff"
            elif self.mode == "crab":
                rap_ctl_leader.is_transit = True # TODO Need to check global varibable
                rap_ctl_follow.is_transit = True
                self.mode = "crab->diff"
            else:
                rospy.logerr("[rap_planner] Invalid mode " + str(self.mode))
        rospy.loginfo("[rap_planner] " + self.mode)

        return True 

    def set_line(self, points,frame_id, RGB = None , size = 0.2, id = 0):
        '''
        Set line at MarkArray
        Input : 
            points = [p1,p2....]
            RGB - tuple : (255,255,255)
            size - float: width of line
            id - int
        '''
        marker = Marker()
        marker.header.frame_id = frame_id
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
        self.marker_line.markers.append(marker)

    def set_sphere(self, point, frame_id , RGB = None  , size = 0.05, id = 0):
        '''
        Set Point at MarkArray 
        Input : 
            point - (x,y)
            RGB - (r,g,b)
        '''
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
        self.marker_point.markers.append(marker)

#######################
### Global function ###
#######################
def sign(value):
    if value >= 0:
        return 1 
    if value < 0:
        return -1

def normalize_angle(angle):
    '''
    Make angle inside range [-pi, pi]
    Arguments:
        angle - flaot
    Return:
        float
    '''
    ans = (abs(angle) % (2*pi))*sign(angle)
    if ans < -pi: # [-2pi, -pi]
        ans += 2*pi
    elif ans > pi: # [pi, 2pi]
        ans -= 2*pi
    return ans

if __name__ == '__main__':
    rospy.init_node('rap_planner',anonymous=False)
    # Get launch file parameters
    # Kinematic
    KP_VEL = rospy.get_param(param_name="~kp_vel", default="1")
    LOOK_AHEAD_DIST = rospy.get_param(param_name="~look_ahead_dist", default="0.8")
    GOAL_TOLERANCE = rospy.get_param(param_name="~goal_tolerance", default="0.1")
    # System 
    CONTROL_FREQ  = rospy.get_param(param_name="~ctl_frequency", default="10")
    SIM  = rospy.get_param(param_name="~sim", default="true")
    REVERSE_OMEGA = rospy.get_param(param_name="~reverse_omega", default="false")
    # Tf frame
    MAP_FRAME     = rospy.get_param(param_name="~map_frame", default="map")
    BIG_CAR_FRAME = rospy.get_param(param_name="~big_car_frame", default="/car1/center_big_car")
    BIG_CAR_PEER_FRAME = rospy.get_param(param_name="~big_car_peer_frame", default="/car2/center_big_car")
    BASE_LINK_FRAME = rospy.get_param(param_name="~base_link_frame", default="base_link")
    BASE_PEER_FRAME = rospy.get_param(param_name="~base_peer_frame", default="base_peer")
    # Topic
    GLOBAL_PATH_TOPIC = rospy.get_param(param_name="~global_path_topic", default="/move_base/GlobalPlanner/plan")
    CMD_VEL_TOPIC_LEADER = rospy.get_param(param_name="~cmd_vel_topic_leader", default="/car1/cmd_vel")
    CMD_VEL_TOPIC_FOLLOW = rospy.get_param(param_name="~cmd_vel_topic_follower", default="/car2/cmd_vel")
    
    # Global variable
    # Init naive controller
    rap_planner   = Rap_planner()
    rap_ctl_leader = Rap_controller("car1", "leader", SIM, CONTROL_FREQ, REVERSE_OMEGA,
                                    MAP_FRAME, BASE_LINK_FRAME, BIG_CAR_FRAME,
                                    CMD_VEL_TOPIC_LEADER)
    rap_ctl_follow = Rap_controller("car2", "follower", SIM, CONTROL_FREQ, REVERSE_OMEGA,
                                    MAP_FRAME, BASE_PEER_FRAME, BIG_CAR_PEER_FRAME,
                                    CMD_VEL_TOPIC_FOLLOW)
    rate = rospy.Rate(CONTROL_FREQ)
    while not rospy.is_shutdown():
        # Set naive cmd
        if rap_planner.run_once():
            # Publish rap_cmd to rap_controller
            rap_planner.publish()
            rap_ctl_leader.set_cmd(rap_planner.vx_out, rap_planner.vy_out,
                                   rap_planner.wz_out, rap_planner.mode)
            rap_ctl_follow.set_cmd(rap_planner.vx_out, rap_planner.vy_out,
                                   rap_planner.wz_out, rap_planner.mode)
        if rap_ctl_leader.run_once():
            rap_ctl_leader.publish()
        if rap_ctl_follow.run_once():
            rap_ctl_follow.publish()
        rate.sleep()