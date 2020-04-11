#!/usr/bin/env python
# A simple node to publish a pre-written waypoint path
from rosplane_msgs.msg import Waypoint
import rospy
def dubins():
    pub = rospy.Publisher('waypoint_path', Waypoint, queue_size=10)
    rospy.init_node('dubins_test', anonymous=True)
    rate = rospy.Rate(1)
    dub_msg = Waypoint()
    dub_msg.chi_valid = True
    dub_msg.Va_d = 15.0
    dub_msg.clear_wp_list = False
    dub_msg.set_current = False
    i = 1
    stop = False
    while not rospy.is_shutdown():
        if i == 1:
            dub_msg.w = [20, 270, -40]
            dub_msg.chi_d = 3.0
        elif i == 2:
            dub_msg.w = [-200, 0, -40]
            dub_msg.chi_d = -1.6
        elif i == 3:
            dub_msg.w = [-170, -240, -40]
            dub_msg.chi_d = 0
        elif i == 4:
            dub_msg.w = [160, -200, -40]
            dub_msg.chi_d = 1.6
        elif i == 5:
            dub_msg.w = [140, -10, -40]
            dub_msg.chi_d = 3.1
        elif i == 6:
            dub_msg.w = [-20, 0, -40]
            dub_msg.chi_d = 1.5
        elif i == 7:
            dub_msg.w = [-10, 140, -40]
            dub_msg.chi_d = -0.7
        elif i == 8:
            dub_msg.w = [100, 60, -40]
            dub_msg.chi_d = 0.7
        elif i == 9:
            dub_msg.w = [200, 250, -40]
            dub_msg.chi_d = -2.5
        else:
            stop = True
        if stop == False:
            pub.publish(dub_msg)
        i = i+1
        rate.sleep()
if __name__ == '__main__':
    try:
        dubins()
    except rospy.ROSInterruptException:
        pass