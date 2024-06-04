#!/usr/bin/env python3
import rospy
from hsr_navigation_module import HSRNavigationLibraryPython
import sys

def test():
    rospy.init_node('test')
    r = rospy.Rate(1) # 10hz
    ang = 0.8
    args = sys.argv
    nav_lib = HSRNavigationLibraryPython(args[0]) # args[0] : C++上でros::init()を行うための引数

    nav_lib.move2Location( "goal", False )
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rospy.loginfo("move2Location")
        r.sleep()


if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException: pass
