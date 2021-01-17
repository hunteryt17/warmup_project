#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

class SquareWalk(object):
    """Makes Turtlebot walk in a square!"""

    def __init__(self):
        rospy.init_node('walk_in_square')
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def run(self):
        linear_forward_vel = Vector3(0.3, 0, 0)
        zero_vel = Vector3(0, 0, 0)
        forward_vel = Twist(linear= linear_forward_vel,
            angular=zero_vel)

        angular_turning_vel = Vector3(0, 0, 0.2)
        turn_vel = Twist(linear=zero_vel,
            angular=angular_turning_vel)

        stop_vel = Twist(linear=zero_vel, angular=zero_vel)

        time_forward = 3
        time_turning = 7
        # tried sleeping to help with some of the error when turning
        #time_sleep = 2

        while self.vel_pub.get_num_connections() < 1:
            pass

        for i in range(4):
            self.vel_pub.publish(forward_vel)
            rospy.sleep(time_forward)
            #self.vel_pub.publish(stop_vel)
            #rospy.sleep(time_sleep)
            self.vel_pub.publish(turn_vel)
            rospy.sleep(time_turning)

        self.vel_pub.publish(stop_vel)



if __name__ == '__main__':
    walker = SquareWalk()
    walker.run()
