#!/usr/bin/env python

# from __future__ import print_function

from ros_pkg_example.srv import AddTwoInts
import rospy
import sys

def service_cb(req):
    print ("Returning " + str(req.a) +" + "+ str(req.a) +" = "+ str(req.a + req.b))
    return req.a + req.b

def main(args):
    rospy.init_node('hello_service')
    service = rospy.Service('add_two_ints', AddTwoInts, service_cb)
    print("service ready")
    r = rospy.Rate(10) #call at 10HZ
    while (not rospy.is_shutdown()):
        r.sleep()

if __name__ == '__main__':
    try:
       main(sys.argv)
    except rospy.ROSInterruptException:
        pass
