#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time 
import signal

class TurtlebotController:
    def __init__(self):
        self.node_name = "obs_avoid" 
        topic_name = "cmd_vel"
        rospy.init_node(self.node_name, anonymous=True) 
        self.pub = rospy.Publisher(topic_name, Twist, queue_size=10) 
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.move_cmd = Twist()
        self.rate = rospy.Rate(20)
        self.obstacle_detected = False
        self.num = None
        self.range = None
        signal.signal(signal.SIGINT, self.shutdown_handler)
        
    def shutdown_handler(self, sig, frame):
        """
        Signal handler for SIGINT (Ctrl+C).
        Performs cleanup and shuts down the ROS node.
        """
        print("Ctrl+C pressed. Shutting down...")
        # Send a stop command to the robot
        self.move_cmd.linear.x = 0
        self.move_cmd.angular.z = 0
        self.pub.publish(self.move_cmd)
        # Perform any additional cleanup here...
        rospy.signal_shutdown("Ctrl+C pressed")    

    def laser_callback(self, data):
        # Define the angular range for the front sector (in radians)
        front_sector_start_angle = 0  # Starting angle of the front sector
        front_sector_end_angle = 6.28    # Ending angle of the front sector
        
        # Convert the start and end angles to indices in the laser scan data array
        num_readings = len(data.ranges)
        start_index = int((front_sector_start_angle - data.angle_min) / data.angle_increment)
        end_index = int((front_sector_end_angle - data.angle_min) / data.angle_increment)

        #print(start_index)
        #print(end_index)

        # Ensure the indices are within valid range
        start_index = max(0, min(start_index, num_readings - 1))
        end_index = max(0, min(end_index, num_readings - 1))

        #print(start_index)
        #print(end_index)


        # Determine if there's an obstacle within the specified sector
        for i in range(start_index, end_index + 1):
            #print('angle :')
            #print(i)
            #print('distance :')
            #print(data.ranges[i])

            if data.ranges[i] == 0:
                data.ranges[i] = 3.5


            if (i<60 or i>300) and (data.ranges[i] < 0.3):  # Threshold distance for obstacle detection
             #   
                #print('angle :')
                #print(i)
                #print('distance :')
                #print(data.ranges[i])

                self.obstacle_detected = True
                break
            else:
                self.obstacle_detected = False


        init = 0
        for j in range(start_index, end_index + 1):

            if (40<j<80 or 320>j>280):

                if init < data.ranges[j]:
                    init = data.ranges[j] 
                    self.num = j
                    self.range = init       

        return
       
       

    def move(self):
        while not rospy.is_shutdown():
            if  self.obstacle_detected == True:
                # Stop and turn to avoid obstacle
                
                print('stop')
                print(self.num)
                #print(self.range)
                self.move_cmd.linear.x = -0.1
                self.move_cmd.angular.z = 0
                self.pub.publish(self.move_cmd)
                time.sleep(1)

                self.move_cmd.linear.x = 0
                if self.num < 180:
                    self.move_cmd.angular.z = 0.785
                else:
                    self.move_cmd.angular.z = -0.785

                self.pub.publish(self.move_cmd)
                if self.num < 180:
                    time.sleep(8*(self.num)/360)
                else:
                    time.sleep(8*(360-self.num)/360)
                
            else:
              # Move forward
                self.move_cmd.linear.x = 0.2  # Move forward at 0.2 m/s
                self.move_cmd.angular.z = 0.0
                self.pub.publish(self.move_cmd)  
                

            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = TurtlebotController()

        controller.move()

    except rospy.ROSInterruptException:
        pass
