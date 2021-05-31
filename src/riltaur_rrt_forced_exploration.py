#!/usr/bin/env python

import rospy
from std_msgs.msg import String
#from geometry_msgs.msg import PoseWithCovarianceStamped
#from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
#from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
#import tf_conversions
#from tf import transformations
#import tf2_ros
#import geometry_msgs.msg
#import math



def lasercallback(data):
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=3)

    front_value = data.ranges[0]
    front_value_left = data.ranges[15]
    front_value_right = data.ranges[345]
    left_value = data.ranges[42]
    right_value = data.ranges[318]
    hard_left_value = data.ranges[90]
    hard_right_value = data.ranges[270]

    print('front: {}, right: {}, hard_right: {}, left: {}, hard_left: {}'.format(front_value,right_value,hard_right_value,left_value,hard_left_value))

    move = Twist()
    r = rospy.Rate(10) 

    if front_value > 0.65 and front_value_left > 0.65 and front_value_right > 0.65 and left_value > 0.890 and right_value > 0.878:
        move.linear.x = 0.1
        move.angular.z = 0.0
        pub.publish(move)
        print('Forward')
        r.sleep()

    elif (front_value < 0.65 and hard_right_value > 0.65) or (left_value < 0.878 and hard_right_value > 0.65):
        move.linear.x = 0.0
        move.angular.z = 0.0
        pub.publish(move)
        r.sleep()

        move.linear.x = 0.0
        move.angular.z = -1*0.8
        pub.publish(move)
        print('Rotate Right')
        r.sleep()

    elif (front_value < 0.65 and hard_left_value > 0.65) or (right_value < 0.878 and hard_left_value > 0.65):
        move.linear.x = 0.0
        move.angular.z = 0.0
        pub.publish(move)
        r.sleep()

        move.linear.x = 0.0
        move.angular.z = 0.8
        pub.publish(move)
        print('Rotate Left')
        r.sleep()

    elif front_value < 0.65 and left_value < 0.878 and right_value < 0.878:
        move.linear.x = 0.0
        move.angular.z = 0.0
        pub.publish(move)
        r.sleep()

        move.linear.x = -1*0.1
        move.angular.z = 0.0
        pub.publish(move)
        print('Backward')
        r.sleep()

    else:
        print('No velocity command is being sent to riltaur')

def main():
    rospy.init_node('forced_exploration_node')
    rospy.Subscriber("/scan", LaserScan, lasercallback)
    rate = rospy.Rate(10)  # 10hz
    rate.sleep()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass