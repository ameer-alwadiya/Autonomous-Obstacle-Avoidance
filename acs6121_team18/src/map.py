#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from subprocess import call
import subprocess

def map():
    rospy.init_node('map', anonymous=True)
    rospy.on_shutdown(save_map)

    rospy.spin()

def save_map():
    rospy.loginfo("Saving map before shutdown...")
    subprocess.Popen(["rosrun", "map_server", "map_saver", "-f", "/home/student/catkin_ws/src/acs6121_team18/maps/explore_map"])
    subprocess.Popen(["eog", "/home/student/catkin_ws/src/acs6121_team18/maps/explore_map.pgm"])

if __name__ == '__main__':
    try:
        map()
    except rospy.ROSInterruptException:
        pass
