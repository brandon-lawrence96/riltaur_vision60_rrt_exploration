#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from gazebo_msgs.msg import ModelStates
import tf_conversions
from tf import transformations
import tf2_ros
import geometry_msgs.msg
import math

vision60_1_map = None
vision60_2_map = None

def vision60_1_mapCallback(data):
    #vision60_1_map = OccupancyGrid()
    global vision60_1_map
    vision60_1_map = OccupancyGrid()
    # vision60_1_map.header = data.header
    # vision60_1_map.info = data.info
    # x = data.data
    vision60_1_map = data
    #print(vision60_1_map)
    
    #return vision60_1_map.data
    
  
def vision60_2_mapCallback(data):
    #vision60_2_map = OccupancyGrid()
    global vision60_2_map
    vision60_2_map = OccupancyGrid()
    # vision60_2_map.header = data.header
    # vision60_2_map.info = data.info
    # y = data.data
    vision60_2_map = data

    #return vision60_2_map.data

def main():
    global vision60_1_map
    global vision60_2_map

    rospy.init_node('map_merger_node', anonymous=True)
    global_map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
    rospy.Subscriber("/new_vision60_1/map", OccupancyGrid, vision60_1_mapCallback)
    rospy.Subscriber("/new_vision60_2/map", OccupancyGrid, vision60_2_mapCallback)

    

    while not rospy.is_shutdown():

        vision60_global_map = OccupancyGrid()
        vision60_global_map.header.frame_id = "map"
        vision60_global_map.info.resolution = 0.05
        vision60_global_map.info.origin.position.x =  -10.0
        vision60_global_map.info.origin.position.y = -10.0
        vision60_global_map.info.origin.position.z = 0.0
        vision60_global_map.info.origin.orientation.w = 0.0

        width_ = 384
        height_ = 384
        vision60_global_map.info.width = width_
        vision60_global_map.info.height = height_

        #print(vision60_1_map)
        if vision60_1_map != None and vision60_2_map != None:
            for i in range(len(vision60_1_map.data)):
                #print(i)
                vision60_global_map.data.append(vision60_1_map.data[i] + vision60_2_map.data[i])
                 

                if vision60_global_map.data[i] == -1:
                    vision60_global_map.data[i] = 0
                elif vision60_global_map.data[i] == -2:
                    vision60_global_map.data[i] = -1
                elif vision60_global_map.data[i] == 99 or vision60_global_map.data[i] == 200:
                    vision60_global_map.data[i] = 100

                #print(vision60_global_map.data)
        
            #print("hi")
            # Publish the new maps
            #print(vision60_global_map.data)
            global_map_pub.publish(vision60_global_map)
            

        rate = rospy.Rate(10)  # 10hz
        rate.sleep()
        #rospy.spin()
        


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass