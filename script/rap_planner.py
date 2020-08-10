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
from rap_controller import Rap_controller

#########################
### Global parameters ###
#########################
LOOK_AHEAD_DIST = 0.8 # Look ahead distance
GOAL_TOLERANCE = 0.1 # Consider goal reach if distance to goal is less then GOAL_TOLERANCE

class Rap_planner():
    def __init__(self):
        rospy.Subscriber("/move_base/GlobalPlanner/plan", Path, self.path_cb)
        self.global_path = None #
        self.pub_marker_point = rospy.Publisher("/local_goal", MarkerArray,queue_size = 1,latch=False)
        self.pub_global_path = rospy.Publisher("/rap_planner/global_path", Path,queue_size = 1,latch=False)
        # 
        self.marker_point = MarkerArray()
        self.marker_line = MarkerArray()# Line markers show on RVIZ
        # Output
        self.v_out = None
        self.w_out = None
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
            return 
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
    
    def run_once(self):
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
        # Debug points
        self.set_sphere((x_goal, y_goal) , BIG_CAR_FRAME, (0,255,255)  , 0.1, 0)

        # Check goal reached 
        if sqrt(x_goal**2 + y_goal**2) < GOAL_TOLERANCE:
            self.v_out = 0
            self.w_out = 0
            self.global_path = None
            rospy.loginfo("[rap_planner] Goal Reached")
            return True
        
        # Get alpha 
        alpha = atan2(y_goal, x_goal)
        if local_goal[2] != None:
            beta  = normalize_angle( local_goal[2] - alpha + self.big_car_xyt[2])
        else:
            beta = 0


        print (beta)
        # Get R 
        R = sqrt( (tan(pi/2 - alpha)*LOOK_AHEAD_DIST/2)**2 + (LOOK_AHEAD_DIST/2.0)**2 )
        if alpha < 0: # alpha = [0,-pi]
            R = -R
        
        self.v_out = sqrt(x_goal**2 + y_goal**2)
        self.w_out = self.v_out / R
        if abs(alpha) > pi/2: # Go backward
            self.v_out *= -1.0
        
        return True 

    def set_line(self, points, RGB = None , size = 0.2, id = 0):
        '''
        Set line at MarkArray
        Input : 
            points = [p1,p2....]
            RGB - tuple : (255,255,255)
            size - float: width of line
            id - int
        '''
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
    SIM           = rospy.get_param(param_name="~sim", default="false")
    CONTROL_FREQ  = rospy.get_param(param_name="~ctl_frequency", default="10")
    REVERSE_OMEGA = rospy.get_param(param_name="~reverse_omega", default="false")
    MAP_FRAME     = rospy.get_param(param_name="~map_frame", default="map")
    BASE_FRAME_LEADER   = rospy.get_param(param_name="~base_frame_leader", default="base_link")
    BASE_FRAME_FOLLOWER = rospy.get_param(param_name="~base_frame_follower", default="base_link")
    BIG_CAR_FRAME   = rospy.get_param(param_name="~big_car_frame", default="big_car")
    CMD_TOPIC_LEADER   = rospy.get_param(param_name="~cmd_topic_leader", default="/car1/cmd_vel")
    CMD_TOPIC_FOLLOWER = rospy.get_param(param_name="~cmd_topic_follower", default="/car2/cmd_vel")
    # Global variable
    
    # Init naive controller
    rap_planner   = Rap_planner()
    rap_ctl_leader = Rap_controller("car1", "leader", SIM, CONTROL_FREQ, MAP_FRAME,
                                    BASE_FRAME_LEADER, BIG_CAR_FRAME, CMD_TOPIC_LEADER)
    rap_ctl_follower = Rap_controller("car2", "follower", SIM, CONTROL_FREQ, MAP_FRAME,
                                    BASE_FRAME_FOLLOWER, BIG_CAR_FRAME, CMD_TOPIC_FOLLOWER)
    
    rate = rospy.Rate(CONTROL_FREQ)
    while not rospy.is_shutdown():
        # Set naive cmd 
        if rap_planner.run_once():
            rap_ctl_leader.set_cmd(rap_planner.v_out, 0, rap_planner.w_out)
            rap_ctl_follower.set_cmd(rap_planner.v_out, 0, rap_planner.w_out)
            # Debug path
            if rap_planner.global_path != None:
                rap_planner.pub_global_path.publish(rap_planner.global_path)
            rap_planner.pub_marker_point.publish(rap_planner.marker_point)
            rap_planner.marker_point = MarkerArray()
            rap_planner.marker_line = MarkerArray()
            
        if rap_ctl_leader.run_once(): 
            # Leader cmd vel 
            cmd_vel = Twist()
            cmd_vel.linear.x  = rap_ctl_leader.v_out
            cmd_vel.angular.z = rap_ctl_leader.w_out
            if REVERSE_OMEGA: # This is for weird simulation bug
                cmd_vel.angular.z = -cmd_vel.angular.z
            rap_ctl_leader.pub_cmd_vel.publish(cmd_vel)
            rap_ctl_leader.pub_marker_line.publish(rap_ctl_leader.marker_line)
        
        if rap_ctl_follower.run_once():
            # Follower cmd vel 
            cmd_vel = Twist()
            cmd_vel.linear.x  = rap_ctl_follower.v_out
            cmd_vel.angular.z = rap_ctl_follower.w_out
            if REVERSE_OMEGA: # This is for weird simulation bug
                cmd_vel.angular.z = -cmd_vel.angular.z
            rap_ctl_follower.pub_cmd_vel.publish(cmd_vel)
            rap_ctl_follower.pub_marker_line.publish(rap_ctl_follower.marker_line)
        rate.sleep()
