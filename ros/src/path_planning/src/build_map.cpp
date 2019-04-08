//
// Created by neil on 4/1/19.
//


#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <grid_map_msgs/GridMap.h>
#include "cv_bridge/cv_bridge.h"
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/crop_box.h>
// Gridmap stuff
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

ros::Publisher pub, map_pub;
grid_map::GridMap map;

// Find the last digit
int lastDigit(int n)
{
    // return the last digit
    return (n % 10);
}


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

//     Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (temp_cloud);
    sor.setLeafSize (0.12f, 0.12f, 0.12f);
    sor.filter (*temp_cloud);

    pcl::CropBox<pcl::PointXYZRGB> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(0, -5, -5, 0.0));
    boxFilter.setMax(Eigen::Vector4f(10, 5, +5, 0.0));
    boxFilter.setInputCloud(temp_cloud);
    boxFilter.filter(*temp_cloud);

//    float min_x = 0;
//    float max_x = 0;
//    float min_y = 0;
//    float max_y = 0;

    for(auto point: temp_cloud->points){

//        if (point.x < min_x)
//            min_x = point.x;
//        if (point.x > max_x)
//            max_x = point.x;
//        if (point.y < min_y)
//            min_y = point.y;
//        if (point.y > max_y)
//            max_y = point.y;

        float pos_x = (round_float(point.x - 5.0f)) <= 5.0f ? (round_float(point.x - 5.0f)) : 5.0;
        float pos_y = round_float(point.y) <= 5.0f ? round_float(point.y) : 5.0;

//        std::cout << std::to_string(pos_x) << std::endl;
//        std::cout << std::to_string(pos_y) << std::endl;
//        std::cout << "-------" << std::endl;

        map.atPosition("elevation", grid_map::Position(pos_x, pos_y)) = point.z;
    }

    // publish point cloud for debugging purposes
    sensor_msgs::PointCloud2 cloud_publish;
    pcl::toROSMsg(*temp_cloud,cloud_publish);
    cloud_publish.header = input->header;
    pub.publish(cloud_publish);

    // Publish grid map.
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
    ros::Subscriber sub = nh.subscribe("/zed/point_cloud/cloud_registered", 1, cloud_callback);

    pub = nh.advertise<sensor_msgs::PointCloud2> ("/point_cloud/exp_1", 1);
    map_pub = nh.advertise<grid_map_msgs::GridMap> ("/grid_map", 1);

    // Create grid map.
    map = grid_map::GridMap({"elevation"});
    map.setFrameId("map");
    map.setGeometry(grid_map::Length(10, 10), 0.10);
    ROS_INFO("Created map with size %f x %f m (%i x %i cells).\n The center of the map is located at (%f, %f) in the %s frame.",
             map.getLength().x(), map.getLength().y(),
             map.getSize()(0), map.getSize()(1),
             map.getPosition().x(), map.getPosition().y(), map.getFrameId().c_str());

//    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
//        grid_map::Position position;
//        map.getPosition(*it, position);
//        std::cout << std::to_string(position.x()) << std::endl;
//        std::cout << std::to_string(position.y()) << std::endl;
//        std::cout << "-------" << std::endl;    }
//
//    std::cout << std::to_string(round_float(1.677)) << std::endl;
//    std::cout << std::to_string(round_float(2.627)) << std::endl;
//    std::cout << std::to_string(round_float(0.627)) << std::endl;

    ros::spin();
    return 0;

}
