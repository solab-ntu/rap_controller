#!/usr/bin/env python
import rospy
import sys
import tf2_ros
import tf # conversion euler
import random
from math import atan2,acos,sqrt,pi,sin,cos,tan
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker, MarkerArray # Debug drawing
from geometry_msgs.msg import Point, Twist, PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
import time # for testing 
from rap_controller import Rap_controller

NUM_CIRCLE_POINT = 100
USE_CRAB_FOR_HEADING = True
USE_COSTMAP = False # TODO deving 

class Rap_planner():
    def __init__(self):
        # Subscriber
        rospy.Subscriber(GLOBAL_PATH_TOPIC, Path, self.path_cb)
        rospy.Subscriber(GOAL_TOPIC, PoseStamped, self.goal_cb)
        if USE_COSTMAP:
            rospy.Subscriber(COSTMAP_TOPIC,OccupancyGrid ,self.costmap_cb)
            self.costmap = None
        self.global_path = None #
        self.simple_goal = None #
        # Publisher
        self.pub_marker_point = rospy.Publisher("/local_goal", MarkerArray,queue_size = 1,latch=False)
        self.pub_marker_line = rospy.Publisher("/rap_planner/angles", MarkerArray,queue_size = 1,latch=False)
        self.pub_marker_text = rospy.Publisher("/rap_planner/text", MarkerArray,queue_size = 1,latch=False)
        self.pub_global_path = rospy.Publisher("/rap_planner/global_path", Path,queue_size = 1,latch=False)
        # Debug publisher
        self.pub_alpha = rospy.Publisher("/alpha", Float64,queue_size = 1,latch=False)
        self.alpha = 0.0
        self.pub_beta  = rospy.Publisher("/beta", Float64,queue_size = 1,latch=False)
        self.beta = 0.0
        self.pub_angle = rospy.Publisher("/angle", Float64,queue_size = 1,latch=False)
        self.angle = 0.0
        # Debug markers
        self.marker_point = MarkerArray()
        self.marker_line = MarkerArray()# Line markers show on RVIZ
        self.marker_text = MarkerArray()
        # Output
        self.vx_out = None
        self.vy_out = None
        self.wz_out = None
        self.mode = "diff" # "crab"
        self.rho = 0.0
        # Tf listner
        self.tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tfBuffer)
        self.big_car_xyt = None 
        # Flags
        self.previous_mode = self.mode
        self.next_mode = None
        self.latch_xy = False
        self.mode_latch_counter = 0
        # Init marker circle
        self.point_list = [["diff",[(LOOK_AHEAD_DIST,0)]]] # [[["crab",(x,y),...]],[],[],...]
        ang = 0.0
        while ang <= 2*pi:
            belong = ""
            if abs(abs(normalize_angle(ang)) - pi/2) < (ASIDE_GOAL_ANG/2.0):
                belong = "crab"
            else:
                belong = "diff"
            # Find nearest line-segment to append
            x = LOOK_AHEAD_DIST*cos(ang)
            y = LOOK_AHEAD_DIST*sin(ang)
            is_found = False
            for seg in self.point_list:
                if  seg[0] == belong and \
                    (seg[1][-1][0] - x)**2 + \
                    (seg[1][-1][1] - y)**2 <= \
                     LOOK_AHEAD_DIST*2*pi/NUM_CIRCLE_POINT:
                     is_found = True
                     seg[1].append((x,y))
            if not is_found: # Creat a new segment
                self.point_list.append([belong, [(x,y)]])
            ang += 2*pi/NUM_CIRCLE_POINT

    def goal_cb(self, data):
        '''
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
        self.reset_plan()
        quaternion = (
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w)
        (_,_,yaw) = tf.transformations.euler_from_quaternion(quaternion)
        self.simple_goal = (data.pose.position.x, data.pose.position.y, yaw)

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

    def costmap_cb(self, data):
        '''
        std_msgs/Header header
            uint32 seq
            time stamp
            string frame_id
        nav_msgs/MapMetaData info
            time map_load_time
            float32 resolution
            uint32 width
            uint32 height
            geometry_msgs/Pose origin
                geometry_msgs/Point position
                float64 x
                float64 y
                float64 z
                geometry_msgs/Quaternion orientation
                float64 x
                float64 y
                float64 z
                float64 w
        int8[] data - 0~100 , 0: free space , 100: obstacle
        '''
        self.costmap = data
        # print (self.costmap.info)
        # print (data.data[self.costmap.info.width/2 +  (self.costmap.info.height/2)*self.costmap.info.width])

    def idx2XY (self, idx):
        '''
        transfer map idx into (x,y) coordinate
        Input : 
            idx - must be interger , idx must inside map index.
        Output: 
            (x,y) - turple 
        '''
        width = self.costmap.info.width
        reso  = self.costmap.info.resolution
        origin = (self.costmap.info.origin.position.x , self.costmap.info.origin.position.y)

        x = (idx % width) * reso + origin[0] + reso/2 # Center of point 
        y = math.floor(idx / width) * reso + origin[1] + reso/2 
        return (round(x,3), round(y,3))

    def XY2idx(self, XY_coor):
        '''
        transfer (x,y) coordinate into  map index 
        Input : 
            XY_coor : (x,y) - turple 
        Output: 
            idx
        '''
        width = self.costmap.info.width
        reso  = self.costmap.info.resolution
        origin = (self.costmap.info.origin.position.x , self.costmap.info.origin.position.y)
        # Y 
        idx =  round((XY_coor[1] - origin[1]) / reso - 0.5) * width
        # X 
        idx += round((XY_coor[0] - origin[0]) / reso - 0.5)
        return int(idx)

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
        for pose in self.global_path.poses: # TODO This can do some performance improvement
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
        try:
            for idx in range(len(self.global_path.poses)):
                dx = self.global_path.poses[idx].pose.position.x - self.big_car_xyt[0]
                dy = self.global_path.poses[idx].pose.position.y - self.big_car_xyt[1]
                d_dist = dx**2 + dy**2
                if d_dist < min_d_dist:
                    prune_point = idx# (pose.pose.position.x, pose.pose.position.y)
                    min_d_dist = d_dist
        except IndexError:
            print (self.global_path.poses)
            return False
        self.global_path.poses = self.global_path.poses[prune_point:]


        # TODO costmap test
        if USE_COSTMAP:
            costmap_idx = self.XY2idx((self.big_car_xyt[:2]))
            value = self.costmap.data[costmap_idx]
            print (value)

            tmp_id = 100
            for idx in range(len(self.global_path.poses)):
                x = self.global_path.poses[idx].pose.position.x
                y = self.global_path.poses[idx].pose.position.y
                costmap_idx = self.XY2idx((x,y))
                value = self.costmap.data[costmap_idx]
                if value != 0:
                    self.set_sphere((x, y) , MAP_FRAME, (0,255,255)  , 0.1, tmp_id)
                    tmp_id += 1
        return True

    def publish(self):
        '''
        Publish debug thing
        '''
        # Debug Markers
        self.pub_marker_point.publish(self.marker_point)
        self.pub_marker_line.publish(self.marker_line)
        self.pub_marker_text.publish(self.marker_text)
        self.marker_line = MarkerArray()
        self.marker_point = MarkerArray()
        self.marker_text = MarkerArray()
        # Dubug msg
        self.pub_alpha.publish(self.alpha)
        self.pub_beta.publish(self.beta)
        self.pub_angle.publish(self.angle)
        
        # Global path
        if self.global_path != None:
            self.pub_global_path.publish(self.global_path)

    def set_tran_mode(self, next_mode):
        '''
        Transit from current mode to next_mode
        '''
        if self.mode_latch_counter > 0:
            rospy.loginfo("[rap_planner] Switch mode rejected by latch. ("\
                          + str(self.mode_latch_counter) + "/" + str(MODE_SWITCH_LATCH))
            return False
        else:
            self.previous_mode = self.mode
            self.next_mode = next_mode
            self.mode = "tran"
            RAP_CTL.is_transit = True
            RAP_CTL.next_mode = next_mode
            rospy.loginfo("[rap_planner] Start transit")
            return True
    
    def reset_plan(self):
        '''
        '''
        self.vx_out = 0.0
        self.vy_out = 0.0
        self.wz_out = 0.0
        self.mode = "diff"
        self.previous_mode = "diff"
        self.next_mode = None
        self.global_path = None
        self.simple_goal = None
        self.latch_xy = False

    def run_once(self):
        # Check simple goal is already reached
        if self.simple_goal == None:
            return False
        
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

        self.rho = sqrt((self.simple_goal[0] - self.big_car_xyt[0])**2 + \
                        (self.simple_goal[1] - self.big_car_xyt[1])**2)

        # Check xy_goal reached
        if  (not self.latch_xy) and self.rho < GOAL_TOLERANCE_XY:
            rospy.loginfo("[rap_planner] Goal xy Reached")
            if IGNORE_HEADING:
                self.reset_plan()
                return True
            else:
                self.latch_xy = True
        
        # Check goal_heading reached
        if self.latch_xy:
            if abs(normalize_angle(self.simple_goal[2] - self.big_car_xyt[2])) <\
                (GOAL_TOLERANCE_T/2.0):
                self.reset_plan()
                rospy.loginfo("[rap_planner] Goal Heading Reached")
                return True
        
        # Get alpha 
        alpha = atan2(y_goal, x_goal)
        
        # Get beta
        '''
        #  This is Benson's legacy, but it's not very helpful
        if (not IGNORE_HEADING) and local_goal[2] != None:
            beta = normalize_angle(local_goal[2] - alpha - self.big_car_xyt[2])
            if abs(alpha) > pi/2:# Go backward
                beta = normalize_angle(beta - pi)
        else:
            beta = 0
        '''
        pursu_angle = alpha
        self.alpha = alpha
        # self.beta = beta
        # self.angle = pursu_angle

        # Debug markers
        # Local goal
        self.set_sphere((x_goal, y_goal) , BIG_CAR_FRAME, (0,255,255)  , 0.1, 0)
        # pursu_angle
        # self.set_sphere((cos(pursu_angle)*LOOK_AHEAD_DIST,
        #                  sin(pursu_angle)*LOOK_AHEAD_DIST,),
        #                 BIG_CAR_FRAME, (255,0,255), 0.1, 1)
        
        self.set_line(((0,0), (0.3,0)), BIG_CAR_FRAME,
                      RGB = (255,255,0), size = 0.02, id = 0)
        self.set_line(((0,0), (cos(alpha)*LOOK_AHEAD_DIST, 
                               sin(alpha)*LOOK_AHEAD_DIST,)),
                      BIG_CAR_FRAME, RGB = (255,255,255), size = 0.02, id = 1)
        # if local_goal[2] != None:
        #     self.set_line(((x_goal, y_goal), (x_goal + cos(pursu_angle)*0.3,
        #                                       y_goal + sin(pursu_angle)*0.3)),
        #                 BIG_CAR_FRAME, RGB = (255,0,255), size = 0.02, id = 2)
        
        # Draw circle
        for idx in range(len(self.point_list)):
            belong = self.point_list[idx][0]
            if belong == "diff":
                color = (255,255,0)
            else:
                color = (255,0,0)
            self.set_line(self.point_list[idx][1], BIG_CAR_FRAME, RGB = color,
                          size = 0.02, id = idx+3)
        
        ##################
        ###  Get Flags ###
        ##################
        # Check where is the goal
        is_aside_goal = False
        if abs(abs(alpha) - pi/2) < (ASIDE_GOAL_ANG/2.0):
            is_aside_goal = True
        # Check is need to consider heading
        d_head = normalize_angle(self.simple_goal[2] - self.big_car_xyt[2])
        need_consider_heading = False
        if (not IGNORE_HEADING) and local_goal[2] != None:
            
            need_consider_heading = True
        # Check need to switch to rota
        is_need_rota = False # current rota only when heading adjment
        if  self.latch_xy: #or\
            #(USE_CRAB_FOR_HEADING and\
            #need_consider_heading and\
            #abs(d_head) > (GOAL_TOLERANCE_T/2.0)): # TODO # ROTA, cause occlication
            # need to adjust heading
            is_need_rota = True
        
        ############################
        ### Finite State Machine ###
        ############################
        # Flags: 
        # States: crab, diff, rota, tran
        if self.mode == "crab":
            if self.latch_xy and (not IGNORE_HEADING):
                self.set_tran_mode("rota")
            else:
                if need_consider_heading:
                    if is_need_rota: 
                        if USE_CRAB_FOR_HEADING:
                            self.set_tran_mode("rota")
                        else:
                            self.set_tran_mode("diff")
                    else:
                        pass # Stay crab
                else:
                    if is_aside_goal:
                        pass # Stay crab
                    else:
                        # Transit to diff mode
                        self.set_tran_mode("diff")
        
        elif self.mode == "diff":
            if self.latch_xy and (not IGNORE_HEADING):
                self.set_tran_mode("rota")
            else:
                if need_consider_heading:
                    if USE_CRAB_FOR_HEADING:
                        # Current diff can't heading adj
                        # self.set_tran_mode("rota")
                        pass
                    else:
                        pass # Stay here
                else:
                    if is_aside_goal:
                        self.set_tran_mode("crab")
                    else:
                        pass
        
        elif self.mode == "rota":
            if need_consider_heading:
                if is_need_rota:
                    pass# Stay rota
                else: # Finish rota
                    if USE_CRAB_FOR_HEADING:
                        self.set_tran_mode("crab") # Go to goal
                    else:
                        self.set_tran_mode("diff") # Go to goal
            else:
                self.set_tran_mode("crab") # Leave heading adj
        
        elif self.mode == "tran":
            if (not RAP_CTL.is_transit):
               rospy.loginfo("[rap_planner] transit finish, switch to " + self.next_mode)
               self.mode_latch_counter = MODE_SWITCH_LATCH
               self.mode = self.next_mode

        else:
            rospy.logerr("[rap_planner] Invalid mode " + str(self.mode))

        ####################
        ### Execute mode ###
        ####################
        if self.mode == "crab" or (self.mode == "tran" and self.next_mode == "crab"):
            self.vx_out = cos(alpha) * KP_VEL
            self.vy_out = sin(alpha) * KP_VEL
            self.wz_out = 0.0
        elif self.mode == "rota" or (self.mode == "tran" and self.next_mode == "rota"):
            self.vx_out = 0.0
            self.vy_out = 0.0
            self.wz_out = ROTA_ANGULAR_VEL*sign(d_head)
        elif self.mode == "diff" or (self.mode == "tran" and self.next_mode == "diff"):
            # Get R
            if self.rho < LOOK_AHEAD_DIST:
                R = sqrt((tan(pi/2 - (pursu_angle))*self.rho/2)**2 +
                        (self.rho/2.0)**2 )
            else:
                R = sqrt((tan(pi/2 - (pursu_angle))*LOOK_AHEAD_DIST/2)**2 +
                        (LOOK_AHEAD_DIST/2.0)**2 )
            if pursu_angle < 0: # alpha = [0,-pi]
                R = -R
            self.vx_out = sqrt(x_goal**2 + y_goal**2) * KP_VEL
            self.vy_out = 0.0
            self.wz_out = self.vx_out / R
            if abs(pursu_angle) > pi/2: # Go backward
                self.vx_out *= -1.0
        else:
            rospy.logerr("[rap_planner] Invalid mode " + str(self.mode))

        # Debug marker text
        if self.mode == "tran":
            text = str(self.previous_mode) + "->" + self.next_mode
        else:
            text = self.mode
        self.set_text((0,-0.5), BIG_CAR_FRAME, text, (0,0,0), 0.2, 0)

        # Latch count 
        if self.mode_latch_counter > 0:
            self.mode_latch_counter -= 1

        return True 

    def set_line(self, points, frame_id, RGB = (255,0,0), size = 0.2, id = 0):
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

    def set_sphere(self, point, frame_id, RGB = (255,0,0), size = 0.05, id = 0):
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
        marker.color.r = RGB[0]/255.0
        marker.color.g = RGB[1]/255.0
        marker.color.b = RGB[2]/255.0
        marker.pose.orientation.w = 1.0
        (marker.pose.position.x , marker.pose.position.y) = point
        self.marker_point.markers.append(marker)

    def set_text(self, point, frame_id, text, RGB = (255,0,0), size = 0.2, id = 0):
        '''
        Set Point at MarkArray 
        Input : 
            point - (x,y) or idx 
            RGB - (r,g,b)
        '''
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.id = id
        marker.ns = "tiles"
        marker.header.stamp = rospy.get_rostime()
        marker.type = marker.TEXT_VIEW_FACING
        marker.action = marker.ADD
        marker.scale.x = size
        marker.scale.y = size
        marker.scale.z = size
        marker.color.a = 1.0
        marker.text = text 
        marker.color.r = RGB[0]/255.0
        marker.color.g = RGB[1]/255.0
        marker.color.b = RGB[2]/255.0
        marker.pose.orientation.w = 1.0
        (marker.pose.position.x , marker.pose.position.y) = point
        self.marker_text.markers.append(marker)

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
    
    #########################
    ### Global parameters ###
    #########################
    # Get launch file parameters
    # Kinematic
    KP_VEL = rospy.get_param(param_name="~kp_vel", default="1")
    LOOK_AHEAD_DIST = rospy.get_param(param_name="~look_ahead_dist", default="0.8")
    GOAL_TOLERANCE_XY = rospy.get_param(param_name="~goal_tolerance_xy", default="0.1")
    GOAL_TOLERANCE_T  = rospy.get_param(param_name="~goal_tolerance_t", default="10")*pi/180
    ASIDE_GOAL_ANG = rospy.get_param(param_name="~aside_goal_ang", default="60")*pi/180 # Degree
    ROTA_ANGULAR_VEL = rospy.get_param(param_name="~rota_angular_vel", default="0.2") # radian/s

    # System
    CONTROL_FREQ  = rospy.get_param(param_name="~ctl_frequency", default="10")
    SIM  = rospy.get_param(param_name="~sim", default="true")
    REVERSE_OMEGA = rospy.get_param(param_name="~reverse_omega", default="false")
    IGNORE_HEADING = rospy.get_param(param_name="~ignore_heading", default="false")
    MODE_SWITCH_LATCH = rospy.get_param(param_name="~mode_switch_latch", default="2")*CONTROL_FREQ# Sec
    # Tf frame
    MAP_FRAME = rospy.get_param(param_name="~map_frame", default="map")
    MAP_PEER_FRAME = rospy.get_param(param_name="~map_peer_frame", default="map")
    BIG_CAR_FRAME = rospy.get_param(param_name="~big_car_frame", default="/car1/center_big_car")
    BIG_CAR_PEER_FRAME = rospy.get_param(param_name="~big_car_peer_frame", default="/car2/center_big_car")
    BASE_LINK_FRAME = rospy.get_param(param_name="~base_link_frame", default="base_link")
    BASE_PEER_FRAME = rospy.get_param(param_name="~base_peer_frame", default="base_peer")
    # Topic
    GLOBAL_PATH_TOPIC = rospy.get_param(param_name="~global_path_topic", default="/move_base/GlobalPlanner/plan")
    CMD_VEL_TOPIC_LEADER = rospy.get_param(param_name="~cmd_vel_topic_leader", default="/car1/cmd_vel")
    CMD_VEL_TOPIC_FOLLOW = rospy.get_param(param_name="~cmd_vel_topic_follower", default="/car2/cmd_vel")
    GOAL_TOPIC = rospy.get_param(param_name="~goal_topic", default="/move_base_simple/goal")
    COSTMAP_TOPIC = rospy.get_param(param_name="~costmap_topic", default="/move_base/local_costmap/costmap")
    # Global variable
    # Init naive controller
    rap_planner   = Rap_planner()
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
        # Set naive cmd
        if rap_planner.run_once():
            # Publish rap_cmd to rap_controller
            rap_planner.publish()
            RAP_CTL.set_cmd(rap_planner.vx_out, rap_planner.vy_out,
                            rap_planner.wz_out, rap_planner.mode)
        if RAP_CTL.run_once():
            RAP_CTL.publish()
        rate.sleep()