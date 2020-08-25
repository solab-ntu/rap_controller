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
    global MODE
    if abs(data.axes[SPEED_AXE]) > 0.1:
        global VX_MAX
        VX_MAX = abs(VX_MAX*(data.axes[SPEED_AXE] + 0.1))  # increase speed by 10%
        rospy.loginfo("current max linear speed: %f", VX_MAX)
    elif abs(data.axes[TURN_AXE]) > 0.1:
        global WZ_MAX
        WZ_MAX = abs(WZ_MAX*(data.axes[TURN_AXE] + 0.1))  # increase turn by 10%
        rospy.loginfo("current max angular speed: %f", WZ_MAX)
    
    # Mode Switch
    if data.buttons[SWITCH_MODE_BUTTON] == 1.0:
        if MODE == "crab":
            rospy.loginfo("Switch to diff mode")
            MODE = "diff"
        else:  # MODE == "diff":
            rospy.loginfo("Switch to crab mode")
            MODE = "crab"
    if data.buttons[TRANSITION_MODE_BUTTON] == 1.0:
        if MODE == "XX->rota":
            rospy.loginfo("Switch to rota mode")
            MODE = "rota"
        else:
            rospy.loginfo("Switch to XX->rota mode")
            MODE = "XX->rota"

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
            elif MODE == "rota":
                twist.angular.y = 2.0
            elif MODE == "XX->rota":
                twist.angular.y = 3.0
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
    IS_SPI = rospy.get_param(param_name="~use_spiderkiller_joy_stick", default="false")
    if IS_SPI:
        VX_AXE = 4
        VY_AXE = 3
        WZ_AXE = 0
        SPEED_AXE = 7
        TURN_AXE  = 6
        SWITCH_MODE_BUTTON = 0
    else:
        VX_AXE = 3
        VY_AXE = 2
        WZ_AXE = 0
        SPEED_AXE = 5
        TURN_AXE  = 4
        SWITCH_MODE_BUTTON = 1
        TRANSITION_MODE_BUTTON = 3

    VX_MAX = rospy.get_param(param_name="~vx_max")
    VY_MAX = rospy.get_param(param_name="~vy_max")
    WZ_MAX = rospy.get_param(param_name="~wz_max")
    
    VX_LAST = 0.0
    VY_LAST = 0.0
    WZ_LAST = 0.0
    MODE = "diff"
    rospy.loginfo("Switch to DIFF MODE")

    PUB_CAR1 = rospy.Publisher(name="/car1/rap_cmd", data_class=Twist, queue_size=1)
    PUB_CAR2 = rospy.Publisher(name="/car2/rap_cmd", data_class=Twist, queue_size=1)
    rospy.Subscriber(name="joy", data_class=Joy, callback=cb_joy, queue_size=1)
    rospy.spin()
