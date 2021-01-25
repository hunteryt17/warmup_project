#!/usr/bin/env python3

import rospy

# msg needed for /scan.
from sensor_msgs.msg import LaserScan

# msgs needed for /cmd_vel.
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

# How close we will get to wall.
distance = 0.4


class PersonFollower(object):

    def __init__(self):
        #start rospy node
        rospy.init_node("follow_person")

        #Declare Scan Subscriber
        rospy.Subscriber("/scan", LaserScan, self.process_data)

        #Publisher for cmd_vel
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        #Set default Twist to 0s
        lin = Vector3()
        ang = Vector3()
        self.twist = Twist(linear=lin, angular=ang)
    

    def process_data(self, data):
        """Process scan data to determine velocity"""
        minimum = min(data.ranges)
        min_index = data.ranges.index(minimum)
        k_d_ang = 0.01
        k_d_dist = 0.5

        # set max velocity that car can move at
        max_velocity = 3

        # sets the angle to be centered around where bot faces
        if min_index <= 180:
                diff_ang = min_index
        else:
                diff_ang = min_index - 360

        diff_dist = minimum - distance
        

        # no object within area robot stops moving
        if diff_dist > 3.5:
            self.twist.angular.z = 0
            self.twist.angular.x = 0
        
        elif abs(diff_ang) < 5:
            # Go forward until it reaches closest object
            self.twist.angular.z = k_d_ang * diff_ang
            self.twist.linear.x = min(k_d_dist * diff_dist, max_velocity)

        else:
            # keep turning until it points to closest object
            self.twist.angular.z = k_d_ang * diff_ang
            self.twist.linear.x = min(k_d_dist * diff_dist * 1/abs(diff_ang),
                                    max_velocity)

        # Publish msg to cmd_vel.
        self.vel_pub.publish(self.twist)

    def run(self):
        """runs the code"""
        rospy.spin()


if __name__ == '__main__':
    node = PersonFollower()
    node.run()

