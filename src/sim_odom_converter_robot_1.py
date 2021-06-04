#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
import tf_conversions
from tf import transformations
import tf2_ros
import geometry_msgs.msg
import math

index = 0


def posecallback(data):
    odom = Odometry()
    odom_box = Odometry()
    global index
    index = 0
    for k in range(0, len(data.name)):
        if (data.name[k] == "robot_1::base_footprint"):
        #if (data.name[k] == "riltaur::dummy_link"):
            index = k
    if (len(data.pose) > 1):
        rot_angle = math.pi / 2

        odom.pose.pose = data.pose[index]
        q_orig = (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
                  odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)
        q_rot = transformations.quaternion_from_euler(0, 0, rot_angle)
        q_new = transformations.quaternion_multiply(q_rot, q_orig)
        odom.pose.pose.orientation.x = q_new[0]
        odom.pose.pose.orientation.y = q_new[1]
        odom.pose.pose.orientation.z = q_new[2]
        odom.pose.pose.orientation.w = q_new[3]

        odom.pose.pose.position.x += 2.0000486
        odom.pose.pose.position.y -= 0.9995944
        odom.pose.pose.position.z += 0.02183627

	
        robot_rpy = transformations.euler_from_quaternion(q_new)
        robot_rpy_msg = Vector3()
        robot_rpy_msg.x = robot_rpy[0]
        robot_rpy_msg.y = robot_rpy[1]
        robot_rpy_msg.z = robot_rpy[2]
	
        rpy_pub = rospy.Publisher('/riltaur/rpy', Vector3, queue_size=10)
        rpy_pub.publish(robot_rpy_msg)
        # odom.pose.pose.orientation.x = 0
        # odom.pose.pose.orientation.y = 0
        # odom.pose.pose.orientation.z = 0
        # odom.pose.pose.orientation.w = 1

        odom.twist.twist = data.twist[index]
        angularx = odom.twist.twist.angular.x
        angulary = odom.twist.twist.angular.y
        # rospy.loginfo("bef: {} {}".format(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y))
        odom.twist.twist.angular.x = angularx * math.cos(rot_angle)-angulary*math.sin(rot_angle)
        odom.twist.twist.angular.y = angularx * math.sin(rot_angle)+angulary*math.cos(rot_angle)
        # rospy.loginfo("aft: {} {}".format(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y))

        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "robot_1/odom"
        odom.child_frame_id = "robot_1/base_footprint"
        #odom.child_frame_id = "dummy_link"

        pub = rospy.Publisher('/robot_1/odom', Odometry, queue_size=10)

        pub.publish(odom)

        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "robot_1/odom"
        t.child_frame_id = "robot_1/base_footprint"
        #t.child_frame_id = "dummy_link"
        t.transform.translation.x = odom.pose.pose.position.x
        t.transform.translation.y = odom.pose.pose.position.y
        t.transform.translation.z = odom.pose.pose.position.z

        t.transform.rotation.x = odom.pose.pose.orientation.x
        t.transform.rotation.y = odom.pose.pose.orientation.y
        t.transform.rotation.z = odom.pose.pose.orientation.z
        t.transform.rotation.w = odom.pose.pose.orientation.w

        br.sendTransform(t)

        # print(odom)


def main():
    rospy.init_node('sim_odom_converter', anonymous=True)
    rospy.Subscriber("/gazebo/link_states", ModelStates, posecallback)
    rate = rospy.Rate(10)  # 10hz
    rate.sleep()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass