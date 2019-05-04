/*!
 *
 * Created by Neil Nie on 4/1/19.
 *
 * \author Neil Nie - contact@neilnie.com
 * \date May 3, 2019
 */


#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <grid_map_msgs/GridMap.h>
#include "cv_bridge/cv_bridge.h"
#include <nav_msgs/OccupancyGrid.h>
#include <costmap_2d/costmap_2d_publisher.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/crop_box.h>
#include <chrono>
// Gridmap stuff
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

ros::Publisher map_pub_local;
grid_map::GridMap local_map;
tf::StampedTransform transform;

/*
 * This method is very rough. The purpose is to round the point cloud position values
 * to grid_map position values that the grid_map can recognize. Remember, based on my
 * map configeration, the grid_map starts at 0.05 and goes up in 0.1, some example
 * positions are: 0.05, 0.15...4.95. If you want to index the value 0.822,
 * the system will crash!
 *
 */
float round_float(float var){

    // 37.66666 * 100 =3766.66
    // 3766.66 + .5 =37.6716    for rounding off value
    // then type cast to int so value is 3766
    // then divided by 100 so the value converted into 37.66
//    float value = (int)(var * 100.0f + 0.5f);
//    value = value + (0.50f - float(((int(value) % 10))));
//    return (float)value / 100;

    return ceil(var * 200.0) / 200.0;
}

void cloud_callback (const sensor_msgs::PointCloud2ConstPtr& input){

//    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

    // Create a container for the data.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

    // Run voxel filter
    // leaf size is 12cm. This effects the resolution of the map.
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (temp_cloud);
    sor.setLeafSize (0.10f, 0.10f, 0.10f);
    sor.filter (*temp_cloud);

    // run a box filter.
    // front: [0, 10]; side: [-5...0...+5]
    pcl::CropBox<pcl::PointXYZRGB> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(0, -5, -5, 0.0));
    boxFilter.setMax(Eigen::Vector4f(12, 5, +5, 0.0));
    boxFilter.setInputCloud(temp_cloud);
    boxFilter.filter(*temp_cloud);

    local_map.clearAll();

//    std::cout << std::to_string(temp_cloud->points.size()) << std::endl;

    for (int i = 0; i < temp_cloud->points.size(); i+=1) {

        auto point = temp_cloud->points[i];

        float pos_x = (round_float(point.x)) <= (12.0f) ? (round_float(point.x)) : (12.0);
        float pos_y = round_float(point.y) <= (5.0f)? round_float(point.y) : (5.0);

//        std::cout << std::to_string(pos_x) << std::endl;
//        std::cout << std::to_string(pos_y) << std::endl;
//        std::cout << "-------" << std::endl;

        if (point.r == 0.0f && point.g == 255.0f && point.b == 0.0f)
            local_map.atPosition("elevation", grid_map::Position(pos_x, pos_y)) = 0.0;
        else if (point.z > 0.30)
             local_map.atPosition("elevation", grid_map::Position(pos_x, pos_y)) = 255;
        else
            local_map.atPosition("elevation", grid_map::Position(pos_x, pos_y)) = point.z;
    }

    // publish local map
    ros::Time time = ros::Time::now();
    local_map.setTimestamp(time.toNSec());
    nav_msgs::OccupancyGrid message_local;
    grid_map::GridMapRosConverter::toOccupancyGrid(local_map, "elevation", -100.0, 300.0, message_local);
    map_pub_local.publish(message_local);

    // timing test
//    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
//    auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
//    std::cout << "time" << std::endl;
//    std::cout << std::to_string(duration) << std::endl;
}

int main (int argc, char** argv) {

    // Initialize ROS
    ROS_INFO("Node started");

    ros::init(argc, argv, "build_map");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/point_cloud/ground_segmentation", 3, cloud_callback);

//    map_pub = nh.advertise<nav_msgs::OccupancyGrid> ("/grid_map", 1);
    map_pub_local = nh.advertise<nav_msgs::OccupancyGrid> ("/grid_map", 3);

    // Create local grid map
    local_map = grid_map::GridMap({"elevation"});
    local_map.setFrameId("base_link");
    local_map.setGeometry(grid_map::Length(12, 10), 0.05);
    local_map.setPosition(grid_map::Position(6, 0));
    ROS_INFO("Created map with size %f x %f m (%i x %i cells).\n The center of the map is located at (%f, %f) in the %s frame.",
             local_map.getLength().x(), local_map.getLength().y(),
             local_map.getSize()(0), local_map.getSize()(1),
             local_map.getPosition().x(), local_map.getPosition().y(), local_map.getFrameId().c_str());

//    for (grid_map::GridMapIterator it(local_map); !it.isPastEnd(); ++it) {
//        grid_map::Position position;
//        local_map.getPosition(*it, position);
//        std::cout << std::to_string(position.x()) << std::endl;
//        std::cout << std::to_string(position.y()) << std::endl;
//        std::cout << "-------" << std::endl;
//    }

    // get robot transform

    tf::TransformListener listener;

    ros::Rate rate(50.0);
    while (nh.ok()){

        try {
            listener.lookupTransform("map", "base_link", ros::Time(), transform);
        }catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
        }
        ros::spinOnce();
        rate.sleep();
    }

    ros::spin();
    return 0;
}
