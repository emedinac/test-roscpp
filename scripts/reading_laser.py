#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np

def clbk_laser(msg):
    # 720/5 = 144
    regions = [ 
        np.min([msg.ranges[0:143],10]),
        np.min([msg.ranges[144:287],10]),
        np.min([msg.ranges[288:431],10]),
        np.min([msg.ranges[432:575],10]),
        np.min([msg.ranges[576:713],10]),
    ]
    rospy.loginfo(regions)

def main():
    rospy.init_node('reading_laser')
    sub= rospy.Subscriber("/scan", LaserScan, clbk_laser)

    rospy.spin()

if __name__ == '__main__':
    main()
