#!/usr/bin/env python3

from cmath import cos
from turtle import distance
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
from Laserdata import LaserData
import numpy as np

class Explorer:
    def __init__(self):
        rospy.init_node('explorer')
        # set up subscriber to get laser scan data
        # self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.process_scan)

        # set up publisher to send movement commands
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # set the linear and angular speeds for movement
        self.rate=10
        self.linear_speed = 0.22
        self.angular_speed = 1
        self.angular_speed_HalfCircle=0.6
        self.threshold= 1
        self.thres_obstacle = 0.48
        self.thres_obstacle_turning = 0.3

        self.idx=[]
        self.flag=0
        self.initail_direction_flag=0
        self.flag_break=0
        self.turn_finish=0
        self.isobj=0
        self.distance=[]
        self.distance_initial=[]
        #1st PID
        self.SideWallDistance = 0.35
        self.Kp_SideWall=1
        self.Ki_SideWall=0
        self.Kd_SideWall=50
        self.error_SideWall=0
        self.errorSum_SideWall=0

        # initialize the movement command
        self.move_cmd = Twist()
        self.LaserData = LaserData()
        # set up state machine
        self.states = {"stop_moving":self.stop_moving,"Heading_Wall":self.Heading_Wall,"obstacle_avoidance":self.obstacle_avoidance}
        self.current_state = 'stop_moving'

        # start exploring
        self.explore()
    def PID_WallFollower(self,SideDist):
        #   PID loop
        errorOld_SideWall = self.error_SideWall;       # Save the old error for differential component
        self.error_SideWall = self.SideWallDistance - SideDist  # Calculate the error in position
        self.errorSum_SideWall = self.errorSum_SideWall+self.error_SideWall
        proportional = self.error_SideWall * self.Kp_SideWall  # Calculates Proportional Error

        integral = self.errorSum_SideWall * self.Ki_SideWall # Calculates Steady State Error
        # print("integral",self.constrain(integral,-0.1,0.1))
        differential = (self.error_SideWall - errorOld_SideWall) * self.Kd_SideWall # Calculates Rate of Error Change

        output_SideWall = proportional + self.constrain(integral,-0.3,0.3) + differential  # Calculate the result
        return output_SideWall

    def PID_Sec(self,SideDist):
        #   PID loop
        errorOld_SideWall = self.error_SideWall_Sec;       # Save the old error for differential component
        self.error_SideWall_Sec = self.SideWallDistance_Sec - SideDist  # Calculate the error in position
        self.errorSum_SideWall_Sec = self.errorSum_SideWall_Sec+self.error_SideWall_Sec
        proportional = self.error_SideWall_Sec * self.Kp_SideWall_Sec  # Calculates Proportional Error

        integral = self.errorSum_SideWall_Sec * self.Ki_SideWall_Sec # Calculates Steady State Error

        differential = (self.error_SideWall_Sec - errorOld_SideWall) * self.Kd_SideWall_Sec # Calculates Rate of Error Change

        output_SideWall = proportional + self.constrain(integral,-50,0) + differential  # Calculate the result
        return output_SideWall
        
    def constrain(self,val, min_val, max_val):
        return min(max_val, max(min_val, val))

    def arena_dis(self, distance):
        indices = []
        for i, x in enumerate(distance):
            if x > self.threshold:
                indices.append(i+1)
        return indices
    def obtain_dis(self):
        front_dis= self.LaserData.front_dis
        left_dis = self.LaserData.left_dis
        right_dis= self.LaserData.right_dis
        back_dis = self.LaserData.back_dis
        distance=[front_dis,left_dis,right_dis,back_dis]
        idx=self.arena_dis(distance)
        return idx,distance
    def obtain_dis_initial(self):
        front_dis_initial= self.LaserData.front_dis_initial
        left_dis_initial = self.LaserData.left_dis_initial
        right_dis_initial= self.LaserData.right_dis_initial
        back_dis_initial = self.LaserData.back_dis
        distance_initial=[front_dis_initial,left_dis_initial,right_dis_initial,back_dis_initial]
        idx_initial=self.arena_dis(distance_initial)
        return idx_initial,distance_initial

    def start_direction(self,idx):

        if idx[0]==2:
            self.turnleft()

        elif idx[0]==3:
            self.turnright()

        elif idx[0]==4:
            self.turn180()

    def move(self):

        # set movement command for moving forward
        self.move_cmd.linear.x = self.linear_speed
        self.move_cmd.angular.z = 0
        self.cmd_vel_pub.publish(self.move_cmd)

    def stop_moving(self):

        # set movement command for moving forward
        self.move_cmd.linear.x = 0
        self.move_cmd.angular.z = 0
        self.cmd_vel_pub.publish(self.move_cmd)

    def turnleft(self):
        rate = rospy.Rate(self.rate) # 10 Hz
        # set movement command for turning away from obstacle
        self.move_cmd.linear.x = 0
        self.move_cmd.angular.z = self.angular_speed
        turn_duration = math.pi / 2 / self.angular_speed # 90 degrees in radians
        t0 = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - t0 < turn_duration:
            self.cmd_vel_pub.publish(self.move_cmd)
            rate.sleep()
            print("uuuuu")
        print("finsih")
        self.move_cmd.angular.z = 0
        self.cmd_vel_pub.publish(self.move_cmd)
        # rate.sleep()
        
    def turnright(self):
        rate = rospy.Rate(self.rate) # 10 Hz
        # set movement command for turning away from obstacle
        self.move_cmd.linear.x = 0
        self.move_cmd.angular.z = -self.angular_speed
        turn_duration = math.pi  /2/ self.angular_speed# 90 degrees in radians
        t0 = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - t0 < turn_duration:
            self.cmd_vel_pub.publish(self.move_cmd)
            rate.sleep()
        print("finish")
        self.move_cmd.angular.z = 0
        self.cmd_vel_pub.publish(self.move_cmd)

    def turn180(self):
        rate = rospy.Rate(self.rate) # 10 Hz
        # set movement command for turning away from obstacle
        self.move_cmd.linear.x = 0
        self.move_cmd.angular.z = self.angular_speed
        turn_duration = math.pi/ self.angular_speed # 90 degrees in radians
        t0 = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - t0 < turn_duration:
            self.cmd_vel_pub.publish(self.move_cmd)
            rate.sleep()
            # print("180")
            # print(f"180", end="\r")
        self.move_cmd.angular.z = 0
        self.cmd_vel_pub.publish(self.move_cmd)
    def wall_follower(self,distance):

        turn_output=round(self.PID_WallFollower(distance),3)
        # print("turn_output",turn_output)
        # set movement command for wall following
        self.move_cmd.linear.x = self.linear_speed
        self.move_cmd.angular.z = self.constrain(turn_output,-1.82,1.82)
        print(f"z=:{self.move_cmd.angular.z}", end="\r")
        self.cmd_vel_pub.publish(self.move_cmd)
    def wall_turning(self,distance):

        turn_output=round(self.PID_Sec(distance),3)
        # print("SEC_output",turn_output)
        
        # set movement command for wall following
        self.move_cmd.linear.x = self.linear_speed
        self.move_cmd.angular.z = self.constrain(turn_output,-1.82,1.82)
        self.cmd_vel_pub.publish(self.move_cmd)
        # print("sec_z=",self.move_cmd.angular.z)


    def Initial_direction(self):
        # self.idx,self.distance=self.obtain_dis()
        self.idx,self.distance_initial=self.obtain_dis_initial() 
        # print(self.idx)   
        print(self.distance_initial)     
        while len(self.idx)==0:
            rate1=rospy.Rate(1)
            # self.current_state = 'moving' 
            index_of_max=np.argmax(self.distance_initial)+1
            print("index_of_max",index_of_max)
            self.start_direction([index_of_max])
            self.move()
            rate1.sleep()
            self.stop_moving()
            self.idx,self.distance_initial=self.obtain_dis_initial()
            # print(self.distance)
            # print(self.idx)
        self.start_direction(self.idx)
        self.initail_direction_flag=1
    def Heading_Wall(self):
        if self.initail_direction_flag==0:
           self.Initial_direction() 
        # self.states[self.current_state]()``
        self.idx,self.distance=self.obtain_dis()
        if self.distance[0]>self.thres_obstacle:
            # print("move")
            print("move")
            # self.current_state = 'moving'
            self.move()
        else:
            # self.current_state = 'turning_left'  
            print("turn_left") 
            self.turnleft()
            # self.current_state = 'stop'                 
            self.flag=1
    def obstacle_avoidance(self):
        self.idx,self.distance=self.obtain_dis()
        right_upper=self.LaserData.right_upper
        degrees = 12
        radians = degrees * math.pi / 180
        cos_degrees = math.cos(radians)

        self.isobj=bool(right_upper<(self.SideWallDistance/cos_degrees+1.4))
        if self.distance[0]>self.thres_obstacle:
            # print(f"right_upper:{self.LaserData.right_upper}", end="\r")
            # print(f"right_upper_thres:{self.SideWallDistance/cos_degrees+1.4}", end="\r")
            if self.isobj==1:
                self.wall_follower(self.distance[2])
            else:
                self.HalfCircle()
        else: 
             print("self.distance[0]",self.distance[0])
             self.turnleft()      
                
    def HalfCircle(self):

        rate = rospy.Rate(self.rate) # 10 Hz
        # self.move_cmd.linear.x = self.angular_speed_HalfCircle*self.SideWallDistance

        self.move_cmd.angular.z = -self.angular_speed_HalfCircle
        turn_duration = math.pi*1/ self.angular_speed_HalfCircle # 90 degrees in radians
        self.move_cmd.linear.x = round(self.angular_speed_HalfCircle*(0.3+2*self.SideWallDistance)/math.pi,2)######ralated with real robot size and sidewall dis need adjust
        t0 = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - t0 < turn_duration*0.9:######duration rate need adjust
            self.idx,self.distance=self.obtain_dis()
            print(f"vel:{self.move_cmd.linear.x}", end="\r")
            # if (self.LaserData.front_obs>self.thres_obstacle_turning)&(self.LaserData.right_upper>self.SideWallDistance):
            if (self.LaserData.front_obs>self.thres_obstacle_turning):
                self.cmd_vel_pub.publish(self.move_cmd)
                rate.sleep()
                # print(f"1800", end="\r")
                print("1800")
            else:
                print("break")
                self.flag_break=1
                break 
        if self.flag_break==0:
            print("line")
            self.move_cmd.angular.z = 0
            self.move_cmd.linear.x=self.linear_speed
            self.cmd_vel_pub.publish(self.move_cmd) 
            rate1=rospy.Rate(1)
            rate1.sleep()
        else:
            self.move_cmd.angular.z = 0
            self.move_cmd.linear.x=0
            self.cmd_vel_pub.publish(self.move_cmd)
            self.turnleft()
            self.flag_break=0

              
    def explore(self):
        # continue exploring until the node is stopped
        rate = rospy.Rate(self.rate)  # 10 Hz
        while not rospy.is_shutdown():
            if self.flag==0:
                # self.Heading_Wall()
                self.current_state = 'Heading_Wall'
            elif self.flag==1:
                self.current_state = 'obstacle_avoidance'

            self.states[self.current_state]()

            rate.sleep()

if __name__ == '__main__':
    Explorer()

