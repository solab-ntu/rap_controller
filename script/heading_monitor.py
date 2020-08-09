#!/usr/bin/env python
import rospy
import sys
import tf2_ros
import tf # conversion euler
import tf_conversions
import random
from geometry_msgs.msg import Twist # topic /cmd_vel
from math import atan2,acos,sqrt,pi,sin,cos,tan
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker, MarkerArray # Debug drawing
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Path, Odometry
import time # for testin

class Heading_monitor():
    def __init__(self):
        rospy.Subscriber("/move_base/GlobalPlanner/plan", Path, self.path_cb)
        rospy.Subscriber("/base_link_odom", Odometry, self.odom_cb)
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_cb)
        # Tf listner
        self.tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tfBuffer)
        self.base_link_xyt = None
        #
        self.is_need_publish = False
        self.voting_result = None # ("forword", 7)

        self.pub_marker_sphere = rospy.Publisher('marker_sphere', MarkerArray,queue_size = 1,latch=False)
        self.pub_odom = rospy.Publisher('/base_link_odom_revise', Odometry,queue_size = 1,latch=False)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel_revise', Twist,queue_size = 1,latch=False)
    def cmd_vel_cb(self, data):
        '''
        geometry_msgs/Vector3 linear
            float64 x
            float64 y
            float64 z
        geometry_msgs/Vector3 angular
            float64 x
            float64 y
            float64 z
        '''
        cmd_vel_new = data
        if self.voting_result != None and self.voting_result[0] == "backword":
            cmd_vel_new.linear.x = -data.linear.x
            cmd_vel_new.angular.z = -data.angular.z
        self.pub_cmd_vel.publish(cmd_vel_new)
    
    def odom_cb(self, data):
        '''
        std_msgs/Header header
            uint32 seq
            time stamp
            string frame_id
            string child_frame_id
        geometry_msgs/PoseWithCovariance pose
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
            float64[36] covariance
        geometry_msgs/TwistWithCovariance twist
            geometry_msgs/Twist twist
                geometry_msgs/Vector3 linear
                    float64 x
                    float64 y
                    float64 z
                geometry_msgs/Vector3 angular
                    float64 x
                    float64 y
                    float64 z
            float64[36] covariance
        '''
        odom = data
        if self.voting_result != None and self.voting_result[0] == "backword":# TODO 
        # if True:
            odom.child_frame_id = "base_link_back"
            q = (
                data.pose.pose.orientation.x,
                data.pose.pose.orientation.y,
                data.pose.pose.orientation.z,
                data.pose.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(q)
            q_new = tf_conversions.transformations.quaternion_from_euler(0, 0, euler[2] + pi)
            odom.pose.pose.orientation.x = q[0]
            odom.pose.pose.orientation.y = q[1]
            odom.pose.pose.orientation.z = q[2]
            odom.pose.pose.orientation.w = q[3]
            odom.twist.twist.linear.x = -data.twist.twist.linear.x
        self.pub_odom.publish(odom)

    def path_cb(self,data):
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
        if self.base_link_xyt == None:
            rospy.logerror("[heading_monitor] Can't get base_link tf, ignoring path")
        else:
            sphere_id = 0
            voting_box = []
            for p in data.poses[:NUM_VOTER]:
                p_dif = (p.pose.position.x - self.base_link_xyt[0],
                         p.pose.position.y - self.base_link_xyt[1])
                t = self.base_link_xyt[2]
                x_out = cos(t)*p_dif[0] + sin(t)*p_dif[1]
                y_out =-sin(t)*p_dif[0] + cos(t)*p_dif[1]

                # Debug markers
                self.set_sphere((x_out, y_out), (255,255,0), 0.02, sphere_id)
                sphere_id += 1

                heading_diff = atan2(y_out, x_out)
                if heading_diff > pi/2 or heading_diff < -pi/2:
                    # print ("Backword, " + str((x_out, y_out)))
                    voting_box.append("backword")
                else: 
                    # print ("Forward, " + str((x_out, y_out)))
                    voting_box.append("forword")

            # Count Finally voting
            back_vote = 0
            front_vote = 0
            for vote in voting_box:
                if vote == "backword":
                    back_vote += 1
                elif vote == "forword":
                    front_vote += 1
            if back_vote > front_vote:
                self.voting_result = ("backword", back_vote)
            elif back_vote < front_vote:
                self.voting_result = ("forword", front_vote)
            else:
                self.voting_result = ("draw", front_vote)
            print ("[heading_monitor]" + self.voting_result[0] + "("+ str(self.voting_result[1]) + 
                    "/" + str(NUM_VOTER) + ")")

            self.is_need_publish = True

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

    def set_sphere(self, point , RGB = None  , size = 0.05, id = 0):
        '''
        Set Point at MarkArray 
        Input : 
            point - (x,y)
            RGB - (r,g,b)
        '''
        global MARKER_SPHERE
        marker = Marker()
        marker.header.frame_id = "base_link"
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
        MARKER_SPHERE.markers.append(marker)

    def run_once(self):
        '''
        Return True: need publish
        Return False: don't need publish
        '''
        global MARKER_SPHERE
        self.base_link_xyt = self.get_tf('map', 'base_link')

        if self.is_need_publish:
            self.pub_marker_sphere.publish(MARKER_SPHERE)
            MARKER_SPHERE = MarkerArray()

if __name__ == '__main__':
    rospy.init_node('heading_monitor',anonymous=False)
    # Get launch file parameters
    # BIG_CAR_FRAME   = rospy.get_param(param_name="~big_car_frame", default="big_car")
    # Global variable
    MARKER_SPHERE = MarkerArray()
    NUM_VOTER = 10
    # Init naive controller
    heading_monitor = Heading_monitor()
    
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        heading_monitor.run_once()
        rate.sleep()
