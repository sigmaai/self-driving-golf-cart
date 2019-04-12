//
// Created by neil on 4/1/19.
//


#include <ros/ros.h>
#include <ros/console.h>
#include "cv_bridge/cv_bridge.h"
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/transforms.h>
// opencv
#include <opencv2/opencv.hpp>

ros::Publisher pub;
tf::StampedTransform transform;
cv::Mat segmentation_image;

void image_callback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat dst;
    cv::resize(cv_ptr->image, dst, cv::Size(1920, 1080));
    segmentation_image = dst;
}

void cloud_callback (const sensor_msgs::PointCloud2ConstPtr& input){

    // Create a container for the data.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_output(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    *cloud_output = *temp_cloud;

//    std::cout << std::to_string(temp_cloud->width) << std::endl;
//    std::cout << std::to_string(temp_cloud->height) << std::endl;
//    std::cout << "-------" << std::endl;
//
//    std::cout << std::to_string(segmentation_image.cols) << std::endl;
//    std::cout << std::to_string(segmentation_image.rows) << std::endl;
//    std::cout << "-------" << std::endl;

    auto* pixelPtr = (uint8_t*)segmentation_image.data;

    for(int i = 0; i < segmentation_image.rows; i++){

        for(int j = 0; j < segmentation_image.cols; j++){

            uint8_t value = pixelPtr[i*segmentation_image.cols + j];
            if (value == 255) {

                cloud_output->points[i*segmentation_image.cols + j].r = 0;
                cloud_output->points[i*segmentation_image.cols + j].g = 255;
                cloud_output->points[i*segmentation_image.cols + j].b = 0;
            }
        }
    }


//    auto* pixelPtr = (uint8_t*)segmentation_image.data;
//
//    for(int i = 0; i < segmentation_image.rows; i++){
//
//        for(int j = 0; j < segmentation_image.cols; j++){
//
//            if (j > 1000) {
//
//                cloud_output->points[i*segmentation_image.cols + j].r = 0;
//                cloud_output->points[i*segmentation_image.cols + j].g = 255;
//                cloud_output->points[i*segmentation_image.cols + j].b = 0;
//            }
//        }
//    }

    sensor_msgs::PointCloud2 cloud_publish;
    pcl::toROSMsg(*cloud_output,cloud_publish);
    cloud_publish.header = input->header;

    pub.publish(cloud_publish);
}

int main (int argc, char** argv) {

    // Initialize ROS
    ROS_INFO("Node started");

    ros::init(argc, argv, "rgb_segmentation_point_cloud");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/zed/point_cloud/cloud_registered", 1, cloud_callback);
    ros::Subscriber sub_img = nh.subscribe("/segmentation/output/road", 1, image_callback);

    pub = nh.advertise<sensor_msgs::PointCloud2> ("/point_cloud/ground_segmentation", 1);

    ros::spin();
    return 0;

}
