
#include "ros/ros.h"
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include <std_msgs/Header.h>
#include <nav_msgs/MapMetaData.h>

// Define global message 
nav_msgs::OccupancyGrid vision60_1_map, vision60_2_map;


void vision60_1_mapCallback(const nav_msgs::OccupancyGrid::ConstPtr & msg)
{
  vision60_1_map.header = msg->header;
  vision60_1_map.info = msg->info;
  vision60_1_map.data = msg->data;
  std::cout << "Go to vision60_1_map callback" << std::endl;
}

void vision60_2_mapCallback(const nav_msgs::OccupancyGrid::ConstPtr & msg)
{
  vision60_2_map.header = msg->header;
  vision60_2_map.info = msg->info;
  vision60_2_map.data = msg->data;
}

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "map_merger_node");
  ros::NodeHandle nh;

  // Create the publisher and subscriber
  const auto global_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 100);
  const auto local_map_1_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/new_vision60_1/map", 100, vision60_1_mapCallback);
  const auto local_map_2_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/new_vision60_2/map", 100, vision60_2_mapCallback);

  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    nav_msgs::OccupancyGrid vision60_global_map;
    vision60_global_map.header.frame_id = "map";
    vision60_global_map.info.resolution = 0.05;
    vision60_global_map.info.origin.position.x =  -10.0;
    vision60_global_map.info.origin.position.y = -10.0;
    vision60_global_map.info.origin.position.z = 0.0;
    vision60_global_map.info.origin.orientation.w = 0.0;

    const size_t width_ = 384;
    const size_t height_ = 384;
    vision60_global_map.info.width = width_;
    vision60_global_map.info.height = height_;

    for(int i=0; i<147456; i++)
    {
        vision60_global_map.data[i] = vision60_1_map.data[i] + vision60_2_map.data[i];

        if (vision60_global_map.data[i] == -1)
        {
            vision60_global_map.data[i] = 0;
        }
        else if (vision60_global_map.data[i] == -2)
        {
            vision60_global_map.data[i] = -1;
        }
        else if ((vision60_global_map.data[i] == 99) || (vision60_global_map.data[i] == 200))
        {
            vision60_global_map.data[i] = 100;
        }
    }

    // Publish the new maps
    global_map_pub.publish(vision60_global_map);

    ros::spinOnce();
    loop_rate.sleep();
  }

   return 0; 
}

  
