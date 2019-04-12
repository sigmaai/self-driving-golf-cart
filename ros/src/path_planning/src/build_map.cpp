//
// Created by neil on 4/1/19.
//


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
// Gridmap stuff
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

ros::Publisher pub, map_pub;
grid_map::GridMap map;

// Find the last digit
int lastDigit(int n){
    return (n % 10);
}

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
    float value = (int)(var * 100.0f + 0.5f);
    value = value + (5.0f - float(lastDigit(int(value))));
    return (float)value / 100;
}

void cloud_callback (const sensor_msgs::PointCloud2ConstPtr& input){

    // Create a container for the data.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

    // Run voxel filter
    // leaf size is 12cm. This effects the resolution of the map.
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (temp_cloud);
    sor.setLeafSize (0.12f, 0.12f, 0.12f);
    sor.filter (*temp_cloud);

    // run a box filter.
    // front: [0, 10]
    // sizes: [-5...0...+5]
    pcl::CropBox<pcl::PointXYZRGB> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(0, -5, -5, 0.0));
    boxFilter.setMax(Eigen::Vector4f(10, 5, +5, 0.0));
    boxFilter.setInputCloud(temp_cloud);
    boxFilter.filter(*temp_cloud);

    for(auto point: temp_cloud->points){

        float pos_x = (round_float(point.x)) <= 10.0f ? (round_float(point.x)) : 10.0;
        float pos_y = round_float(point.y) <= 5.0f ? round_float(point.y) : 5.0;

//        std::cout << std::to_string(pos_x) << std::endl;
//        std::cout << std::to_string(pos_y) << std::endl;
//        std::cout << "-------" << std::endl;

        if (point.r == 0.0f && point.g == 255.0f && point.b == 0.0f)
            map.atPosition("elevation", grid_map::Position(pos_x, pos_y)) = 0.0;
        else if (point.z > 0.75)
            map.atPosition("elevation", grid_map::Position(pos_x, pos_y)) = 253;
        else{
            map.atPosition("elevation", grid_map::Position(pos_x, pos_y)) = point.z;
        }
    }

//    // publish point cloud for debugging purposes
//    sensor_msgs::PointCloud2 cloud_publish;
//    pcl::toROSMsg(*temp_cloud,cloud_publish);
//    cloud_publish.header = input->header;
//    pub.publish(cloud_publish);

    ros::Time time = ros::Time::now();
    map.setTimestamp(time.toNSec());
    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(map, message);
    map_pub.publish(message);

}

int main (int argc, char** argv) {

    // Initialize ROS
    ROS_INFO("Node started");

    ros::init(argc, argv, "build_map");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/point_cloud/ground_segmentation", 1, cloud_callback);

    pub = nh.advertise<sensor_msgs::PointCloud2> ("/point_cloud/exp_1", 1);
    map_pub = nh.advertise<grid_map_msgs::GridMap> ("/grid_map", 1);

    // Create grid map.
    map = grid_map::GridMap({"elevation"});
    map.setFrameId("base_link");
    map.setGeometry(grid_map::Length(10, 10), 0.10);
    map.setPosition(grid_map::Position(5, 0));
    ROS_INFO("Created map with size %f x %f m (%i x %i cells).\n The center of the map is located at (%f, %f) in the %s frame.",
             map.getLength().x(), map.getLength().y(),
             map.getSize()(0), map.getSize()(1),
             map.getPosition().x(), map.getPosition().y(), map.getFrameId().c_str());

//    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
//        grid_map::Position position;
//        map.getPosition(*it, position);
//
//        std::cout << std::to_string(position.x()) << std::endl;
//        std::cout << std::to_string(position.y()) << std::endl;
//        std::cout << "-------" << std::endl;
//
//        usleep(2000);
//    }

    ros::spin();
    return 0;

}
