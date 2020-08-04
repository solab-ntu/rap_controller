#!/usr/bin/env python

"""
Convert ros-kinetic-joy inputs.
Caution:
    The parameter deadzone of ros-kinetic-joy should not equal to zero.
    Make sure the release state of joy can publish the zero command.
"""

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

def cb_joy(data):
    '''
    Call back function of /joy
    '''
    if abs(data.axes[SPEED_AXE]) > 0.1:
        global VX_MAX
        VX_MAX = abs(VX_MAX*(data.axes[SPEED_AXE] + 0.1))  # increase speed by 10%
        rospy.loginfo("current max linear speed: %f", VX_MAX)
    elif abs(data.axes[TURN_AXE]) > 0.1:
        global WZ_MAX
        WZ_MAX = abs(WZ_MAX*(data.axes[TURN_AXE] + 0.1))  # increase turn by 10%
        rospy.loginfo("current max angular speed: %f", WZ_MAX)
    elif data.buttons[SWITCH_MODE_BUTTON] == 1.0:
        global MODE
        if MODE == "crab":
            rospy.loginfo("Switch to diff mode")
            MODE = "diff"
        elif MODE == "diff":
            rospy.loginfo("Switch to crab mode")
            MODE = "crab"

    else:
        global VX_LAST
        global VY_LAST
        global WZ_LAST
        twist = Twist()
        vx = data.axes[VX_AXE] * VX_MAX
        vy = data.axes[VY_AXE] * VY_MAX
        wz = data.axes[WZ_AXE] * WZ_MAX
        if  vx == 0.0 and wz == 0.0 and vy == 0.0 and\
            VX_LAST == 0.0  and VY_LAST == 0.0 and WZ_LAST == 0.0:
            pass
        else:
            twist.linear.x = vx
            twist.linear.y = vy
            twist.angular.z = wz
            if MODE == "crab":
                twist.angular.y = 1.0
            elif MODE == "diff":
                twist.angular.y = 0.0
            
            # Don't send -0.0
            if twist.linear.x == 0.0:
                twist.linear.x = 0.0
            if twist.linear.y == 0.0:
                twist.linear.y = 0.0
            if twist.angular.z == 0.0:
                twist.angular.z = 0.0
            PUB_CAR1.publish(twist)
            PUB_CAR2.publish(twist)
            rospy.loginfo("Vx: %f; Vy: %f; W %f", twist.linear.x, twist.linear.y, twist.angular.z)
        VX_LAST = vx
        VY_LAST = vy
        WZ_LAST = wz

if __name__ == '__main__':

    rospy.init_node(name="teleop_joy", anonymous=True)

    VX_MAX = rospy.get_param(param_name="~vx_max")
    VY_MAX = rospy.get_param(param_name="~vy_max")
    WZ_MAX = rospy.get_param(param_name="~wz_max")
    SWITCH_MODE_BUTTON = rospy.get_param(param_name="~switch_mode_button")
    VX_AXE = rospy.get_param(param_name="~vx_axe")
    VY_AXE = rospy.get_param(param_name="~vy_axe")
    WZ_AXE = rospy.get_param(param_name="~wz_axe")
    SPEED_AXE = rospy.get_param(param_name="~speed_axe")
    TURN_AXE  = rospy.get_param(param_name="~turn_axe")



    VX_LAST = 0.0
    VY_LAST = 0.0
    WZ_LAST = 0.0
    MODE = "diff"
    rospy.loginfo("Switch to DIFF MODE")

    PUB_CAR1 = rospy.Publisher(name="/car1/naive_cmd", data_class=Twist, queue_size=1)
    PUB_CAR2 = rospy.Publisher(name="/car2/naive_cmd", data_class=Twist, queue_size=1)
    rospy.Subscriber(name="joy", data_class=Joy, callback=cb_joy, queue_size=1)
    rospy.spin()
