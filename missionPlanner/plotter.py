#!/usr/bin/env python 
import numpy as np
from matplotlib import pyplot as plt
import rospy
from rosplane_msgs.msg import State

def plot_x(msg):

    print(msg)
    print(msg.position[1], msg.position[0])
    plt.plot(msg.position[1], msg.position[0], 'r.')
    plt.axis("equal")
    plt.draw()
    plt.pause(0.00000000001)


if __name__ == '__main__':
    counter = 0

    rospy.init_node("plotter")
    rospy.Subscriber("/truth_slow", State, plot_x)
    plt.ion()
    plt.show()
    rospy.spin()