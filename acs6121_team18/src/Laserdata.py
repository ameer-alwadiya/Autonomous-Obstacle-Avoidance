#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from scipy.ndimage import gaussian_filter1d, median_filter


class LaserData(object):
    def laserscan_cb(self, msg):
        ranges = np.array(msg.ranges)
        # ranges = self.gaussian_filter(ranges)
        ranges = [r if r > 0.1 else float("inf") for r in msg.ranges]
        ranges = [min(max(r, 0.1), 3.5) for r in ranges]
        ranges = self.median_filter(ranges)
        # print("ranges",ranges)
        self.front_dis = round(np.min(ranges[0:15].tolist() + ranges[353:359].tolist()), 2)
        self.left_dis = round(np.min(ranges[75:105]), 2)
        self.right_dis = round(np.min(ranges[255:285]), 2)
        self.back_dis = round(np.min(ranges[145:215]), 2)
        self.right_upper = round(np.min(ranges[282:285]), 2) 
        self.front_obs = round(np.min(ranges[0:35].tolist() + ranges[325:359].tolist()), 2)#180 turning
        #initial
        self.front_dis_initial=round(np.min(ranges[0:35].tolist() + ranges[325:359].tolist()), 2)
        self.left_dis_initial = round(np.min(ranges[55:125]), 2)
        self.right_dis_initial = round(np.min(ranges[235:305]), 2)
    def gaussian_filter(self, data, sigma=2):
        return gaussian_filter1d(data, sigma)

    def median_filter(self, data, size=3):
        return median_filter(data, size)

    def __init__(self):
        self.front_dis = 0
        self.left_dis = 0
        self.right_dis = 0
        self.back_dis = 0
        self.right_lower = 0
        self.right_upper = 0
        self.front_obs = 0
        self.front_dis_initial=0
        self.left_dis_initial=0
        self.right_dis_initial=0
        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.laserscan_cb)

if __name__ == '__main__':
    rospy.init_node('laser_data_node', anonymous=True)
    laser_data = LaserData()
    rospy.spin()
