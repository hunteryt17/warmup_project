#!/usr/bin/env python3

import rospy

# msg needed for /scan.
from sensor_msgs.msg import LaserScan

# msgs needed for /cmd_vel.
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

# How close we will get to wall.
distance = 0.4

class WallFollower(object):
    """ This node makes bot follow wall and turn appropriately """

    def __init__(self):
        # Start rospy node.
        rospy.init_node("walk_to_wall")

        # Declare Subscriber for scan topic (see distance to wall)
        rospy.Subscriber("/scan", LaserScan, self.process_scan)

        # Publisher for cmd_vel topic (set velocity of bot)
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Default Twist (all values 0)
        lin = Vector3()
        ang = Vector3()
        self.twist = Twist(linear=lin,angular=ang)
    

    def find_wall(self, data):
        """finds wall closest to bot and goes toward it"""
        minimum = min(data.ranges)
        min_index = data.ranges.index(minimum)

        # convert angle to be centered around front laser 
        # set differences and constants
        if min_index <= 180:
                diff_ang = min_index
        else:
                diff_ang = min_index - 360
        diff_dist = minimum - distance
        k_d_dist = 0.5
        k_d_ang = 0.01
        
        if min_index < 5 or min_index > 355:
            # Go forward until it reaches close to wall
            self.twist.angular.z = k_d_ang * diff_ang
            self.twist.linear.x = k_d_dist * diff_dist
        else:
            # keep turning until it faces closest wall
            self.twist.angular.z =k_d_ang * diff_ang
            if self.twist.linear.x > 0:
                self.twist.linear.x = k_d_dist * diff_dist
    
    def follow_wall(self, data):
        """follows wall"""
        minimum = min(data.ranges)
        min_index = data.ranges.index(minimum)
        
        # convert angle to be centered around 90 degrees
        # set differnces and constants
        if min_index <= 270:
            diff_ang = min_index - 90
        else:
            diff_ang = min_index - 450
        diff_dist = data.ranges[0] - distance
        k_d_dist = 0.3
        k_d_ang = 0.01

        #check to see if parallel to wall
        if min_index < 95 or min_index > 85:
            # Go forward until it reaches a wall
            self.twist.angular.z = k_d_ang * diff_ang
            self.twist.linear.x = max(diff_dist * k_d_dist, 0.2)
        else:
            # keep turning until close to being parallel to wall
            self.twist.angular.z = k_d_ang * diff_ang
            self.twist.linear.x = 0
    
    def turn(self, data):
        """Turns robot at corner"""
        minimum = min(data.ranges)
        min_index = data.ranges.index(minimum)

        # set constants
        vel_ang = -0.3
        vel_linear = 0.01

        #set speeds while turning
        self.twist.linear.x= vel_linear
        self.twist.angular.z = vel_ang 

    def process_scan(self, data):
        """Fully processes the scan"""
         # check to see if bot far from a wall
        if min(data.ranges) > distance + distance/2:
            self.find_wall(data)

        # bot is close enough to corner to turn
        elif data.ranges[90] < 1.5* distance and data.ranges[0] < 1.5 * distance:
            self.turn(data)
        
        else:
            self.follow_wall(data)
            

        # Publish msg to cmd_vel.
        self.twist_pub.publish(self.twist)

    def run(self):
        """runs the code"""
        rospy.spin()

if __name__ == '__main__':
    node = WallFollower()
    node.run()