#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
import tf_conversions
from tf import transformations
import tf2_ros
import geometry_msgs.msg
import math

def velocitycallback(data):
    move = Twist()
    move = data

    pub2 = rospy.Publisher('/vision60_1/twist', Twist, queue_size=10)
    #pub2 = rospy.Publisher('/vision60/remote/twist', Twist, queue_size=10)

    pub2.publish(move)


def main():
    rospy.init_node('velocity_converter_vision60_1', anonymous=True)
    rospy.Subscriber("/cmd_vel", Twist, velocitycallback)
    rate = rospy.Rate(10)  # 10hz
    rate.sleep()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass