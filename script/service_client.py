#!/usr/bin/env python

import sys
import rospy
from ros_pkg_example.srv import AddTwoInts

def add_two_ints_client(x, y):
    rospy.wait_for_service('add_two_ints')
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        resp1 = add_two_ints(x, y) # call service
        return resp1.sum
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y]"%sys.argv[0]

def main(args):
    if len(args) == 3:
        x = int(args[1])
        y = int(args[2])
    else:
        print (str(sys.argv[0]) + " [x y]" )
        sys.exit(1)
    print("Requesting %s+%s"%(x, y))
    print("%s + %s = %s"%(x, y, add_two_ints_client(x, y)))

if __name__ == "__main__":
    try:
       main(sys.argv)
    except rospy.ROSInterruptException:
        pass




